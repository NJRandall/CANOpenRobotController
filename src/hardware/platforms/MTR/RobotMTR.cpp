#include "RobotMTR.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "CO_command.h"   // cancomm_socketFree() — used directly by checkDriveAlive()

using namespace Eigen;
using namespace std;


// ═══════════════════════════════════════════════════════════════════════════════
// Construction / destruction
// ═══════════════════════════════════════════════════════════════════════════════

// New overload constructs joints based on explicit drive_node_ids
RobotMTR::RobotMTR(const string &robot_name, const string &yaml_config_file,
                   const std::vector<int> &drive_node_ids, bool isMaster)
    : Robot(robot_name, yaml_config_file), isMaster_(isMaster), driveNodeIds_(drive_node_ids) {

    // Load YAML overrides before joints are constructed so limits are correct.
    initialiseFromYAML(yaml_config_file);

    // Explicit constructor arguments override YAML; an empty list selects the
    // node list loaded for this robot from the shared parameter file.
    std::vector<int> configuredDriveNodeIds = drive_node_ids.empty()
                                            ? driveNodeIds
                                            : drive_node_ids;
    driveNodeIds_ = configuredDriveNodeIds;   // probe/liveness checks use the resolved list
    for (size_t i = 0; i < configuredDriveNodeIds.size(); ++i) {
        int node = configuredDriveNodeIds[i];
        double qmin = qLimits.size() > 2*i ? qLimits[2*i] : qLimits[0];
        double qmax = qLimits.size() > 2*i+1 ? qLimits[2*i+1] : qLimits[1];
        double iPeak = iPeakDrives.size() > i ? iPeakDrives[i] : iPeakDrives[0];
        double kt = motorCstt.size() > i ? motorCstt[i] : motorCstt[0];
        double ratio = reductionRatios.size() > i ? reductionRatios[i] : reductionRatios[0];
        // A wrong ratio silently mis-scales every position, velocity and torque, so refuse to start.
        if (!(ratio > 0.0) || !std::isfinite(ratio)) {
            spdlog::critical("RobotMTR {}: reductionRatios[{}] = {} is invalid (must be > 0).",
                             robot_name, i, ratio);
            throw std::invalid_argument("RobotMTR: invalid reductionRatios in " + yaml_config_file);
        }
        int counts = (int)(encoderCounts.size() > i ? encoderCounts[i] : encoderCounts[0]);
        if (counts <= 0) {
            spdlog::critical("RobotMTR {}: encoderCounts[{}] = {} is invalid (must be > 0).",
                             robot_name, i, counts);
            throw std::invalid_argument("RobotMTR: invalid encoderCounts in " + yaml_config_file);
        }
        short int qsign = qSigns.size() > i ? (short int)qSigns[i] : (short int)qSigns[0];
        std::string jname = "q" + std::to_string(i+1);
        addJoint(new JointMT((int)i,
                             qmin, qmax,
                             qsign,
                             -dqMax, dqMax,
                             -tauMax, tauMax,
                             iPeak, kt, ratio,
                             new CopleyDrive(node), jname,
                             qDampingWidth, qDampingGain, counts));
    }

    // UI hardware (keyboard/encoder/LCD) is master-only: a non-master unit has none of
    // this physically wired — it only drives its motors over CAN — so keyboard/encoder/lcd
    // stay nullptr (their in-class initialisers) rather than aliasing the master's hardware.
    if (isMaster_) {
        addInput(keyboard = new Keyboard());

        // KY-040 rotary encoder: S1(CLK)=P8_11, S2(DT)=P8_12, Key(SW)=P8_15
        addInput(encoder = new RotaryEncoder(8, 11, 8, 12, 8, 15));

        // 16x2 I2C LCD: PCF8574 at 0x27 on /dev/i2c-2. There is one physical LCD per
        // BeagleBone, so every unit on this board shares one LCD1602 object (and its single
        // flush thread). Two objects would interleave half-byte writes and garble the display.
        static std::weak_ptr<LCD1602> sharedLcd;
        lcdOwner_ = sharedLcd.lock();
        if (!lcdOwner_) {
            lcdOwner_ = std::make_shared<LCD1602>(0x27, 2);
            lcdOwner_->init();
            sharedLcd = lcdOwner_;
        }
        lcd = lcdOwner_.get();

        // IMU disabled — not in use for now. Uncomment to re-enable.
        // // MPU-6050 IMU for Left/Right orientation detection — master only (see armSide()
        // // comment in RobotMTR.h for why the listener's side is inferred rather than measured).
        // // TODO: confirm address/bus against the physical wiring. Default MPU-6050 address is
        // // 0x68 (AD0 pin low); 0x69 if AD0 is tied to 3.3V. Bus 1 is free (LCD above uses bus 2).
        // addInput(imu = new I2CMPU6050IMU(0x68, 1));

        // Optional local USB mouse — live reference for MOUSE_MIMIC/MOUSE_MIRROR, used if
        // present; isConnected() is false (not a crash) when no mouse is plugged in.
        addInput(mouse = new USBMouse());
    }

    last_update_time = chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}

// Original two-argument constructor delegates to the drive-node overload with default nodes 1 and 3
RobotMTR::RobotMTR(const string &robot_name, const string &yaml_config_file)
    : RobotMTR(robot_name, yaml_config_file, std::vector<int>{}) {}

RobotMTR::~RobotMTR() {
    for (auto p : joints) delete p;
    joints.clear();
    delete keyboard;
    delete encoder;
    lcd = nullptr;   // shared LCD is released by lcdOwner_ when the last unit goes
    // delete imu;
    delete mouse;
    inputs.clear();
}


// ═══════════════════════════════════════════════════════════════════════════════
// YAML parameter loading
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::fillParam(YAML::Node node, vector<double> &vec) {
    if (node) {
        unsigned int n = std::min((unsigned int)vec.size(), (unsigned int)node.size());
        for (unsigned int i = 0; i < n; i++)
            vec[i] = node[i].as<double>();
    }
}

bool RobotMTR::loadParametersFromYAML(YAML::Node params) {
    YAML::Node p = params[robotName];
    if (!p) {
        spdlog::warn("RobotMTR: No parameters for '{}' in YAML — using defaults.",
                     robotName);
        return false;
    }

    if (params["auto_calibration"])
        automaticCalibrationEnabled_ = params["auto_calibration"].as<bool>();
    if (params["collision_avoidance"])
        collisionUseHitboxes_ = params["collision_avoidance"].as<bool>();
    if (params["collision_log_only"])
        collisionLogOnly_ = params["collision_log_only"].as<bool>();
    if (params["collision_prediction_horizon"])
        collisionPredictionHorizon_ = max(0., params["collision_prediction_horizon"].as<double>());
    if (params["collision_influence_distance"])
        collisionInfluenceDistance_ = max(0., params["collision_influence_distance"].as<double>());
    if (params["collision_spring_gain"])
        collisionSpringGain_ = max(0., params["collision_spring_gain"].as<double>());
    if (params["collision_decay_length"])
        collisionDecayLength_ = max(1e-6, params["collision_decay_length"].as<double>());
    if (params["collision_damping_gain"])
        collisionDampingGain_ = max(0., params["collision_damping_gain"].as<double>());

    // Left/Right mounting — set directly from YAML while IMU-based detection stays
    // disabled (see RobotMTR.h); swap "L"/"R" here if the physical setup changes.
    if (p["arm_side"]) {
        const std::string side = p["arm_side"].as<std::string>();
        if (side == "L" || side == "LEFT")       armSide_ = ArmSide::LEFT;
        else if (side == "R" || side == "RIGHT") armSide_ = ArmSide::RIGHT;
        else spdlog::warn("RobotMTR: Unrecognised arm_side '{}' for '{}'; leaving UNKNOWN.",
                           side, robotName);
    }

    // Geometry — loaded from YAML so that tuning never requires a recompile
    if (p["L1"])             L1             = p["L1"].as<double>();
    if (p["L2"])             L2             = p["L2"].as<double>();
    if (p["parallel_ratio"]) parallel_ratio = p["parallel_ratio"].as<double>();

    if (p["drive_node_ids"] && p["drive_node_ids"].size() > 0) {
        driveNodeIds.clear();
        for (unsigned int i = 0; i < p["drive_node_ids"].size(); ++i)
            driveNodeIds.push_back(p["drive_node_ids"][i].as<int>());
    }

    // Drive envelope (hard-constrained for safety)
    if (p["dqMax"])       dqMax       = min(max(0., p["dqMax"].as<double>()), 3600.) * M_PI / 180.;
    if (p["tauMax"])      tauMax      = min(max(0., p["tauMax"].as<double>()), 80.);
    if (p["tauSafetyMax"]) tauSafetyMax = max(tauMax, p["tauSafetyMax"].as<double>());
    YAML::Node calibrationDirection = p["calibration_direction"];
    YAML::Node calibrationTorque = p["calibration_torque"];
    if (calibrationDirection || calibrationTorque) {
        bool valid = calibrationDirection && calibrationTorque &&
                     calibrationDirection.size() == 2 && calibrationTorque.size() == 2;
        if (valid) {
            for (int joint = 0; joint < 2; ++joint) {
                const double direction = calibrationDirection[joint].as<double>();
                const double torque = calibrationTorque[joint].as<double>();
                if ((direction != -1.0 && direction != 1.0) ||
                    !std::isfinite(torque) || torque <= 0.0 || torque > tauMax) {
                    valid = false;
                    break;
                }
            }
        }
        if (valid) {
            for (int joint = 0; joint < 2; ++joint) {
                calibrationDirection_[joint] = calibrationDirection[joint].as<double>();
                calibrationTorque_[joint] = calibrationTorque[joint].as<double>();
            }
        } else {
            automaticCalibrationEnabled_ = false;
            spdlog::error("RobotMTR: Invalid auto-calibration settings for '{}'; "
                          "automatic calibration disabled.", robotName);
        }
    }
    if (p["qDampingWidth"]) qDampingWidth = max(0., p["qDampingWidth"].as<double>()) * M_PI / 180.;
    if (p["qDampingGain"])  qDampingGain  = max(0., p["qDampingGain"].as<double>());
    if (p["singularityDampingAngleExtended"])
        singularityDampingAngleExtended = max(0., p["singularityDampingAngleExtended"].as<double>()) * M_PI / 180.;
    if (p["singularityDampingAngleFolded"])
        singularityDampingAngleFolded = max(0., p["singularityDampingAngleFolded"].as<double>()) * M_PI / 180.;
    if (p["singularityDampingGain"])
        singularityDampingGain = max(0., p["singularityDampingGain"].as<double>());

    YAML::Node collision = p["collision"];
    if (collision) {
        if (collision["base_x"]) collisionBaseX = collision["base_x"].as<double>();
        if (collision["base_y"]) collisionBaseY = collision["base_y"].as<double>();
        if (collision["base_yaw"])
            collisionBaseYaw = collision["base_yaw"].as<double>() * M_PI / 180.;
        if (collision["link_thickness"])
            collisionLinkThickness = max(0., collision["link_thickness"].as<double>());
        if (collision["boundary"])
            collisionBoundary = max(0., collision["boundary"].as<double>());
        if (collision["link_A_length"])
            collisionLinkALength = max(0., collision["link_A_length"].as<double>());
        if (collision["joint_radius"])
            collisionJointRadius = max(0., collision["joint_radius"].as<double>());
        if (collision["end_effector_radius"])
            collisionEndEffectorRadius = max(0., collision["end_effector_radius"].as<double>());
        YAML::Node frame = collision["frame"];
        if (frame) {
            if (frame["center_x"])
                collisionFrameCenterX = frame["center_x"].as<double>();
            if (frame["center_y"])
                collisionFrameCenterY = frame["center_y"].as<double>();
            if (frame["width"])
                collisionFrameWidth = max(0., frame["width"].as<double>());
            if (frame["height"])
                collisionFrameHeight = max(0., frame["height"].as<double>());
            if (frame["boundary"])
                collisionFrameBoundary = max(0., frame["boundary"].as<double>());
        }
    }

    // Safety envelope
    if (p["maxEndEffForce"]) maxEndEffForce = max(0., p["maxEndEffForce"].as<double>());

    fillParam(p["qSigns"],       qSigns);
    fillParam(p["iPeakDrives"],  iPeakDrives);
    fillParam(p["motorCstt"],    motorCstt);
    fillParam(p["reductionRatios"], reductionRatios);
    fillParam(p["encoderCounts"], encoderCounts);
    fillParam(p["frictionVis"],  frictionVis);
    fillParam(p["frictionCoul"], frictionCoul);

    if (p["qLimits"]) {
        unsigned int n = std::min((unsigned int)qLimits.size(), (unsigned int)p["qLimits"].size());
        for (unsigned int i = 0; i < n; i++)
            qLimits[i] = p["qLimits"][i].as<double>() * M_PI / 180.;
    }

    if (p["qCalibration"]) {
        if (p["qCalibration"].size() > 0)
            qCalibration[0] = p["qCalibration"][0].as<double>() * M_PI / 180.;
        if (p["qCalibration"].size() > 1)
            qCalibration[1] = p["qCalibration"][1].as<double>() * M_PI / 180.;
    }

    spdlog::info("RobotMTR: Loaded YAML parameters for '{}'"
                 " (L1={:.3f} m, L2={:.3f} m, parallel_ratio={:.3f}).",
                 robotName, L1, L2, parallel_ratio);
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════════
// CORC initialisation
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotMTR::initialiseJoints() {
    return true;   // joints are created in the constructor
}

bool RobotMTR::initialiseNetwork() {
    spdlog::debug("RobotMTR::initialiseNetwork()");

    for (auto joint : joints) {
        if (!joint->initNetwork()) return false;
    }

    // Allow PDO initialisation to settle
    for (int i = 0; i < 5; i++) { spdlog::debug("."); usleep(10000); }

    for (auto joint : joints) joint->start();

    // Enable drives
    int n = 0;
    for (auto joint : joints) {
        bool ready = false;
        for (int i = 0; i < 10 && !ready; i++) {
            joint->readyToSwitchOn();
            usleep(10000);
            ready = ((joint->getDriveStatus() & 0x01) == 0x01);
        }
#ifndef NOROBOT
        if (!ready) {
            spdlog::error("MTR: Failed to enable joint {} (status: 0x{:02x})",
                          n, joint->getDriveStatus());
            return false;
        }
#endif
        n++;
    }

    updateRobot();
    return true;
}

bool RobotMTR::initialiseInputs() {
    return true;   // encoder is registered via addInput() in constructor
}

std::size_t RobotMTR::probeDriveNodes() const {
    std::size_t responsiveNodes = 0;
    for (int nodeID : driveNodeIds_) {
        if (checkDriveAlive(nodeID))
            ++responsiveNodes;
    }
    return responsiveNodes;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Drive mode initialisation (same pattern as RobotM3)
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotMTR::initTorqueControl() {
    spdlog::debug("RobotMTR: Initialising torque control on all joints.");
    bool ok = true;
    for (auto p : joints) {
        if (((JointMT *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            spdlog::error("RobotMTR: Joint {} failed to enter torque control.",
                          p->getId());
            ok = false;
        }
        ((JointMT *)p)->readyToSwitchOn();
    }
    usleep(2000);
    for (auto p : joints) ((JointMT *)p)->enable();
    return ok;
}

bool RobotMTR::initPositionControl() {
    spdlog::debug("RobotMTR: Initialising position control on all joints.");
    bool ok = true;
    for (auto p : joints) {
        if (((JointMT *)p)->setMode(CM_POSITION_CONTROL) != CM_POSITION_CONTROL) {
            spdlog::error("RobotMTR: Joint {} failed to enter position control.",
                          p->getId());
            ok = false;
        }
        ((JointMT *)p)->readyToSwitchOn();
    }
    usleep(2000);
    for (auto p : joints) ((JointMT *)p)->enable();
    return ok;
}

bool RobotMTR::initVelocityControl() {
    spdlog::debug("RobotMTR: Initialising velocity control on all joints.");
    bool ok = true;
    for (auto p : joints) {
        if (((JointMT *)p)->setMode(CM_VELOCITY_CONTROL) != CM_VELOCITY_CONTROL) {
            spdlog::error("RobotMTR: Joint {} failed to enter velocity control.",
                          p->getId());
            ok = false;
        }
        ((JointMT *)p)->readyToSwitchOn();
    }
    usleep(2000);
    for (auto p : joints) ((JointMT *)p)->enable();
    return ok;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Calibration
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::applyCalibration() {
    for (unsigned int i = 0; i < joints.size(); i++)
        ((JointMT *)joints[i])->setPositionOffset(qCalibration[i]);
    calibGraceCycles_ = 10;   // ~20 ms at 500 Hz for the drive's 0x6064 to reflect the new offset
    calibrated = true;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Kinematics
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Forward kinematics ───────────────────────────────────────────────────────
// Parallel linkage: θ₂ is the absolute world-frame angle of link 2 (same origin
// as θ₁ at the shoulder), not the angle relative to link 1.
// x = L1·cos(θ₁) + L2·cos(θ₂)
// y = L1·sin(θ₁) + L2·sin(θ₂)
VM2 RobotMTR::directKinematic(VM2 q) {
    return VM2(
        L1 * cos(q[0]) + L2 * cos(q[1]),
        L1 * sin(q[0]) + L2 * sin(q[1])
    );
}

// ─── Jacobian ─────────────────────────────────────────────────────────────────
// 2×2 planar Jacobian for parallel linkage (θ₂ absolute).
// det(J) = L1·L2·sin(θ₂ - θ₁) — zero only at singular configs (arm fully extended/folded).
// getPosition() returns joint-space radians (per-joint reductionRatios applied in JointMT).
Matrix2d RobotMTR::J() {
    double t1 = joints[0]->getPosition();
    double t2 = joints.size() > 1 ? joints[1]->getPosition() : 0.0;

    Matrix2d Jac;
    Jac(0, 0) = -L1 * sin(t1);    Jac(0, 1) = -L2 * sin(t2);
    Jac(1, 0) =  L1 * cos(t1);    Jac(1, 1) =  L2 * cos(t2);

    return Jac;
}

// ─── Gravity ──────────────────────────────────────────────────────────────────
// Zero — the arm moves in a horizontal plane; gravity acts normal to the workspace.
VM2 RobotMTR::calculateGravityTorques() {
    return VM2::Zero();
}

MTRCollisionHitboxes RobotMTR::getCollisionHitboxes() const {
    bool two = joints.size() > 1;
    VM2 q(joints[0]->getPosition(), two ? joints[1]->getPosition() : 0.0);
    return getCollisionHitboxes(q);
}

MTRCollisionHitboxes RobotMTR::getCollisionHitboxes(VM2 q) const {
    double c = std::cos(collisionBaseYaw);
    double s = std::sin(collisionBaseYaw);
    auto toWorkspace = [this, c, s](VM2 localPoint) {
        return VM2(c * localPoint[0] - s * localPoint[1] + collisionBaseX,
                   s * localPoint[0] + c * localPoint[1] + collisionBaseY);
    };

    // q is expressed relative to this robot's base. The primary chain is
    // origin -> L1 -> L2, while the parallel chain is origin -> A -> B.
    // A follows q2 and B follows q1, with B having the same length as L1.
    VM2 origin = VM2::Zero();
    VM2 endL1 = VM2(L1 * std::cos(q[0]), L1 * std::sin(q[0]));
    VM2 endL2 = endL1 + VM2(L2 * std::cos(q[1]), L2 * std::sin(q[1]));
    VM2 endA = VM2(collisionLinkALength * std::cos(q[1]),
                   collisionLinkALength * std::sin(q[1]));
    VM2 endB = endA + VM2(L1 * std::cos(q[0]), L1 * std::sin(q[0]));
    VM2 frameCenterLocal(collisionFrameCenterX, collisionFrameCenterY);

    double radius = 0.5 * collisionLinkThickness;
    return MTRCollisionHitboxes{
        {toWorkspace(origin), toWorkspace(endL1), radius, collisionBoundary},
        {toWorkspace(endL1), toWorkspace(endL2), radius, collisionBoundary},
        {toWorkspace(origin), toWorkspace(endA), radius, collisionBoundary},
        {toWorkspace(endA), toWorkspace(endB), radius, collisionBoundary},
        {toWorkspace(origin), collisionJointRadius, collisionBoundary},
        {toWorkspace(endL1), collisionJointRadius, collisionBoundary},
        {toWorkspace(endA), collisionJointRadius, collisionBoundary},
        {toWorkspace(endL2), collisionEndEffectorRadius, collisionBoundary},
        {toWorkspace(frameCenterLocal),
         VM2(0.5 * collisionFrameWidth, 0.5 * collisionFrameHeight),
         collisionBaseYaw,
         collisionFrameBoundary}
    };
}

VM2 RobotMTR::collisionPointVelocity(std::size_t linkIndex, double fraction) const {
    if (joints.size() < 2 || linkIndex > 4)
        return VM2::Zero();

    const double q1 = joints[0]->getPosition();
    const double q2 = joints[1]->getPosition();
    const double dq1 = joints[0]->getVelocity();
    const double dq2 = joints[1]->getVelocity();
    const VM2 tangent1(-std::sin(q1), std::cos(q1));
    const VM2 tangent2(-std::sin(q2), std::cos(q2));

    VM2 localVelocity = VM2::Zero();
    switch (linkIndex) {
    case 0: localVelocity = fraction * L1 * dq1 * tangent1; break;
    case 1: localVelocity = L1 * dq1 * tangent1 + fraction * L2 * dq2 * tangent2; break;
    case 2: localVelocity = fraction * collisionLinkALength * dq2 * tangent2; break;
    case 3: localVelocity = fraction * L1 * dq1 * tangent1 +
                             collisionLinkALength * dq2 * tangent2; break;
    case 4: localVelocity = L1 * dq1 * tangent1 + L2 * dq2 * tangent2; break;
    }

    const double cosine = std::cos(collisionBaseYaw);
    const double sine = std::sin(collisionBaseYaw);
    return VM2(cosine * localVelocity.x() - sine * localVelocity.y(),
               sine * localVelocity.x() + cosine * localVelocity.y());
}

VM2 RobotMTR::collisionPointJointTorque(std::size_t linkIndex, double fraction,
                                       const VM2 &force) const {
    if (joints.size() < 2 || linkIndex > 4)
        return VM2::Zero();

    const double q1 = joints[0]->getPosition();
    const double q2 = joints[1]->getPosition();
    const VM2 tangent1(-std::sin(q1), std::cos(q1));
    const VM2 tangent2(-std::sin(q2), std::cos(q2));
    VM2 jacobianColumn1 = VM2::Zero();
    VM2 jacobianColumn2 = VM2::Zero();

    switch (linkIndex) {
    case 0:
        jacobianColumn1 = fraction * L1 * tangent1;
        break;
    case 1:
        jacobianColumn1 = L1 * tangent1;
        jacobianColumn2 = fraction * L2 * tangent2;
        break;
    case 2:
        jacobianColumn2 = fraction * collisionLinkALength * tangent2;
        break;
    case 3:
        jacobianColumn1 = fraction * L1 * tangent1;
        jacobianColumn2 = collisionLinkALength * tangent2;
        break;
    case 4:
        jacobianColumn1 = L1 * tangent1;
        jacobianColumn2 = L2 * tangent2;
        break;
    }

    const double cosine = std::cos(collisionBaseYaw);
    const double sine = std::sin(collisionBaseYaw);
    auto rotate = [cosine, sine](const VM2 &vector) {
        return VM2(cosine * vector.x() - sine * vector.y(),
                   sine * vector.x() + cosine * vector.y());
    };
    return VM2(rotate(jacobianColumn1).dot(force),
               rotate(jacobianColumn2).dot(force));
}


// ═══════════════════════════════════════════════════════════════════════════════
// Robot update (called every control cycle)
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::updateRobot() {
    spdlog::trace("RobotMTR::updateRobot()");
    Robot::updateRobot();   // fills jointPositions_, jointVelocities_, jointTorques_; also calls imu->updateInput()

    // IMU disabled — not in use for now. Uncomment to re-enable.
    // updateArmSide();        // no-op if imu is null (listener) — see RobotMTR.h

    // ESTOP-PROBE disabled — not in use. Uncomment to re-enable.
    // // TEMP: log statusword changes to identify the E-stop signature.
    // static std::vector<int> lastStatus;
    // lastStatus.resize(joints.size(), -1);
    // for (size_t i = 0; i < joints.size(); ++i) {
    //     // Mask bit 7 (warning): it can toggle every few ms and flood the log.
    //     int sw = static_cast<JointMT *>(joints[i])->getDriveStatus() & ~0x0080;
    //     if (sw != lastStatus[i]) {
    //         spdlog::warn("[ESTOP-PROBE] {} joint {} statusword 0x{:04x} -> 0x{:04x}", robotName, i, lastStatus[i] & 0xFFFF, sw);
    //         lastStatus[i] = sw;
    //     }
    // }

    bool two = joints.size() > 1;
    VM2 q  (joints[0]->getPosition(), two ? joints[1]->getPosition() : 0.0);
    VM2 dq (joints[0]->getVelocity(), two ? joints[1]->getVelocity() : 0.0);
    VM2 tau(joints[0]->getTorque(),   two ? joints[1]->getTorque()   : 0.0);

    // End-effector position via forward kinematics
    endEffPositions  = directKinematic(q);

    // End-effector velocity: dX = J · dq
    Matrix2d _J      = J();
    endEffVelocities = _J * dq;

    // End-effector force from motor torques: F = (Jᵀ)⁻¹ · τ
    double det = _J.determinant();   // = L1·L2·sin(θ₂ - θ₁) — zero only at singularity (arm fully extended/folded)
    if (abs(det) > 1e-6) {
        Matrix2d _Jtinv = (_J.transpose()).inverse();
        endEffForces      = _Jtinv * tau;
        interactionForces = endEffForces;   // gravity is zero; τ_interaction = τ_motor
    }

    if (safetyCheck() != SUCCESS) {
        disable();
    }

    // Polling liveness check disabled — drive power-off is detected from the drives' EMCY
    // frames instead (MTRobotMachine::openDrivePowerMonitor). Each SDO poll blocked the
    // control loop for ~20 ms per drive. Uncomment to re-enable.
    // Throttled drive-liveness check — sets safetyTriggered_ on failure, which safetyStop
    // (MTRobotMachine.cpp) turns into an EmergencyStop transition.
    // if (++livenessCheckCounter_ >= livenessCheckPeriod_) {
    //     livenessCheckCounter_ = 0;
    //     for (int nodeID : driveNodeIds_) {
    //         if (!checkDriveAlive(nodeID)) {
    //             spdlog::error("RobotMTR: drive node {} not responding — treating as lost.", nodeID);
    //             safetyTriggered_ = true;
    //             disable();
    //             break;
    //         }
    //     }
    // }

    last_update_time = chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Drive liveness
// ═══════════════════════════════════════════════════════════════════════════════

// SDO read of register 0x6076 (static, read-only). A successful read replies "[seq] <data>"
// (only writes reply "OK"), while a timeout or abort replies "[seq] ERROR: ...", so the drive
// is alive if the reply has no "ERROR". Kept MTR-local rather than changing
// Drive::SDORead(), which is shared with other robot platforms and checks for "OK".
bool RobotMTR::checkDriveAlive(int nodeID) {
#ifdef NOROBOT
    (void)nodeID;
    return true;
#else
    std::stringstream cmd;
    cmd << "[1] " << nodeID << " read 0x6076 0 i16 ";
    char returnMessage[STRING_BUFFER_SIZE];
    cancomm_socketFree(const_cast<char *>(cmd.str().c_str()), returnMessage);
    return std::string(returnMessage).find("ERROR") == std::string::npos;
#endif
}


// Bring-up aid: reads the registers that decide whether a drive obeys CANopen torque commands.
void RobotMTR::printDriveDiagnostics() {
#ifndef NOROBOT
    static const std::vector<std::pair<std::string, std::string>> regs = {
        {"0x2300 0 i16",  "desired state (Copley; CANopen control = 30)"},
        {"0x6060 0 i8",   "mode of operation (torque = 4)"},
        {"0x6061 0 i8",   "mode display (torque = 4)"},
        {"0x6041 0 u16",  "status word (enabled = 0x..37)"},
        {"0x2183 0 u32",  "latched fault (Copley)"},
        {"0x1002 0 u32",  "event status (Copley)"},
        {"0x6076 0 u32",  "rated torque (scale of 0x6071)"},
        {"0x6072 0 u16",  "max torque [per mille]"},
        {"0x6071 0 i16",  "target torque [per mille]"},
        {"0x6077 0 i16",  "actual torque [per mille]"},
    };
    for (int node : driveNodeIds_) {
        spdlog::info("MTR drive diagnostics, node {}:", node);
        for (const auto &r : regs) {
            std::string cmd = "[1] " + std::to_string(node) + " read " + r.first;
            char returnMessage[STRING_BUFFER_SIZE];
            cancomm_socketFree(const_cast<char *>(cmd.c_str()), returnMessage);
            std::string reply(returnMessage);
            while (!reply.empty() && (reply.back() == '\n' || reply.back() == '\r')) reply.pop_back();
            spdlog::info("  {:<14} {:<45} -> {}", r.first, r.second, reply);
        }
    }
#endif
}

// ═══════════════════════════════════════════════════════════════════════════════
// Device orientation (Left/Right)
// ═══════════════════════════════════════════════════════════════════════════════
//
// Left vs Right is a physical 180° flip of the housing (chip facing up vs down) —
// not a yaw rotation, which gravity sensing can't detect at all. It's the same flip
// that gives collisionBaseYaw its 0°/180° values for the collision frame, though
// that's a separate, YAML-configured fact about the shared workspace frame, not
// something read from the IMU. armSide_ is currently set from arm_side in
// MTR_params.yaml (see loadParametersFromYAML()) while the IMU stays disabled.

// IMU disabled — not in use for now. Uncomment to re-enable.
// void RobotMTR::updateArmSide() {
    // if (!imu) return;   // listener — no IMU, side is inferred by the caller instead

    // // TODO: confirm which IMU axis reads "up" when the device is mounted upright — this is a
    // // fixed mechanical fact (how the MPU-6050 board sits inside the housing), not something
    // // derivable at runtime, so it can't be left auto-detecting. To find it: hold the device in
    // // its intended ArmSide::LEFT orientation, print imu->getAcc(), and see which axis reads
    // // close to +9.81 m/s^2. Then pick the matching line below (only one should be uncommented).
    // double upAxisRaw = imu->getAcc()[2];   // TODO (placeholder): Z axis, default orientation
    // // double upAxisRaw = imu->getAcc()[0];   // TODO option: X axis, default orientation
    // // double upAxisRaw = imu->getAcc()[1];   // TODO option: Y axis, default orientation
    // // double upAxisRaw = -imu->getAcc()[2];  // TODO option: Z axis, inverted mounting (chip facing down)
    // // double upAxisRaw = -imu->getAcc()[0];  // TODO option: X axis, inverted mounting
    // // double upAxisRaw = -imu->getAcc()[1];  // TODO option: Y axis, inverted mounting

    // // Low-pass filter to reject noise/vibration before making a decision.
    // const double alpha = 0.01;   // TODO: tune — smaller = more smoothing, slower to respond
    // armSideAxisFiltered_ += alpha * (upAxisRaw - armSideAxisFiltered_);

    // // Dead-zone (Schmitt-trigger) decision: upright (~+1g on the up axis) -> ArmSide::LEFT,
    // // flipped 180° (~-1g) -> ArmSide::RIGHT. The dead zone is the band of tilt angles around
    // // the 90° boundary where we deliberately do NOT commit to either side, so mounting slop or
    // // vibration near the boundary can't cause a spurious decision. The housing is rigid (can
    // // only physically sit at ~0° or ~180°), so this margin only needs to cover mounting
    // // tolerance, not a genuinely ambiguous resting angle — kept tight for a fast, confident
    // // decision rather than generously wide.
    // // reading = g*cos(tilt), so |reading| < deadZoneThreshold <=> tilt within (90 ± marginDeg)°.
    // const double g                 = 9.80665;
    // const double marginDeg         = 30.0;    // TODO: tune against actual mounting tolerance
    // const double deadZoneThreshold = g * std::cos(marginDeg * M_PI / 180.0);

    // if (armSideAxisFiltered_ > deadZoneThreshold) {
        // armSide_ = ArmSide::LEFT;
    // } else if (armSideAxisFiltered_ < -deadZoneThreshold) {
        // armSide_ = ArmSide::RIGHT;
    // }
    // // else: inside the dead zone — ambiguous reading, keep the previous (latched) classification
    // // rather than forcing a decision from a noisy/boundary sample.
// }


// ═══════════════════════════════════════════════════════════════════════════════
// Safety
// ═══════════════════════════════════════════════════════════════════════════════

setMovementReturnCode_t RobotMTR::safetyCheck() {
    if (calibrated) {
        if (calibGraceCycles_ > 0) { calibGraceCycles_--; return SUCCESS; }
        for (unsigned int i = 0; i < joints.size(); i++) {
            double q = joints[i]->getPosition();
            if (q < qLimits[2*i] || q > qLimits[2*i+1]) {
                spdlog::error("MTR: Joint {} position limit exceeded ({:.1f} deg)!",
                              i, q * 180.0 / M_PI);
                return OUTSIDE_LIMITS;
            }
        }
        if (getEndEffVelocity().norm() > maxEndEffVel) {
            spdlog::error("MTR: End-effector velocity limit exceeded ({:.2f} m/s)!",
                          getEndEffVelocity().norm());
            return OUTSIDE_LIMITS;
        }
        for (unsigned int i = 0; i < joints.size(); i++) {
            double tau = joints[i]->getTorque();   // joint torque [N·m] — reductionRatios applied
            if (tau > tauSafetyMax || tau < -tauSafetyMax) {
                spdlog::error("MTR: Joint {} measured torque e-stop  tau={:.3f} limit=±{:.3f} N·m",
                              i, tau, tauSafetyMax);
                return OUTSIDE_LIMITS;
            }
        }
    } else {
        for (unsigned int i = 0; i < joints.size(); i++) {
            if (((JointMT *)joints[i])->safetyCheck() != SUCCESS) {
                spdlog::error("MTR: Joint {} safety triggered!", i);
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Joint-space setters
// ═══════════════════════════════════════════════════════════════════════════════

setMovementReturnCode_t RobotMTR::setJointTorque(VM2 tau,
                                                  bool applySingularityDamping) {
    if (applySingularityDamping && joints.size() > 1) {
        VM2 q(joints[0]->getPosition(), joints[1]->getPosition());
        VM2 dq(joints[0]->getVelocity(), joints[1]->getVelocity());
        tau += singularityDampingTorque(q, dq);
    }
    return applyTorque({tau[0], tau[1]});
}

setMovementReturnCode_t RobotMTR::setJointPosition(VM2 q) {
    return applyPosition({q[0], q[1]});
}

setMovementReturnCode_t RobotMTR::setJointVelocity(VM2 dq) {
    return applyVelocity({dq[0], dq[1]});
}

VM2 RobotMTR::singularityDampingTorque(VM2 q, VM2 dq) const {
    if (singularityDampingGain <= 0.0)
        return VM2::Zero();

    // The link alignment is determined by the relative angle q2 - q1.
    // Zero is fully extended; +/- pi is fully folded.
    double delta = q[1] - q[0];

    // Wrap the relative angle so equivalent configurations use one common range.
    double wrappedDelta = std::atan2(std::sin(delta), std::cos(delta));

    // Select the nearer singularity branch: zero for extended, +/- pi for folded.
    bool approachingExtended = std::abs(wrappedDelta) <= M_PI / 2.0;

    // Measure the shortest angular distance to the selected singularity.
    double distance = approachingExtended
                    ? std::abs(wrappedDelta)
                    : M_PI - std::abs(wrappedDelta);

    // Extended and folded configurations can use different activation bands.
    double dampingAngle = approachingExtended
                        ? singularityDampingAngleExtended
                        : singularityDampingAngleFolded;

    // Outside the band there is no damping; the exact singularity is left to
    // the mechanical stop rather than creating a direction ambiguity here.
    if (dampingAngle <= 0.0 || distance >= dampingAngle || distance < 1e-9)
        return VM2::Zero();

    // This gradient points toward increasing distance from the selected singularity.
    double direction = wrappedDelta < 0.0 ? -1.0 : 1.0;
    VM2 gradient = approachingExtended
                 ? VM2(-direction, direction)
                 : VM2(direction, -direction);

    // A negative rate means the measured joint motion is approaching the singularity.
    double distanceRate = gradient.dot(dq);
    if (distanceRate >= 0.0)
        return VM2::Zero();

    // Ramp damping smoothly from zero at the band edge to full gain at the stop.
    double activation = 1.0 - distance / dampingAngle;

    // Viscous damping opposes approach velocity; it does not create a spring force.
    return -singularityDampingGain * activation * distanceRate * gradient;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Task-space force setter
// ═══════════════════════════════════════════════════════════════════════════════

// τ = Jᵀ · F  +  τ_friction
// Gravity is zero (horizontal plane) — no τ_gravity term.
//
// Safety pipeline (in order):
//   1. Saturate XY Cartesian force magnitude — preserves force direction.
//   2. Map to joint torques via Jᵀ — never requires Jacobian inversion.
//   3. Add per-joint friction compensation in joint space.
//   4. Clamp each joint torque to ±tauMax.
setMovementReturnCode_t
RobotMTR::setEndEffForceWithCompensation(VM2 F, bool friction_comp) {
    return setEndEffForceWithCompensation(F, friction_comp, VM2::Zero());
}

setMovementReturnCode_t
RobotMTR::setEndEffForceWithCompensation(VM2 F, bool friction_comp,
                                         const VM2 &collisionPriorityTorque) {
    if (!calibrated) return NOT_CALIBRATED;

    // ── 1. Cartesian force saturation ─────────────────────────────────────────
    double fNorm = F.norm();
    if (fNorm > maxEndEffForce) {
        F *= maxEndEffForce / fNorm;
        spdlog::warn("MTR: Cartesian force saturated from {:.1f} N to {:.1f} N.",
                     fNorm, maxEndEffForce);
    }

    // ── 2. Cartesian → joint torques via Jᵀ ──────────────────────────────────
    VM2 tau = J().transpose() * F;

    // ── 3. Friction compensation ──────────────────────────────────────────────
    if (friction_comp) {
        const double threshold = 0.03;   // dead-band [rad/s]
        for (unsigned int i = 0; i < joints.size(); i++) {
            double dq = ((JointMT *)joints[i])->getVelocity();
            if (std::abs(dq) > threshold)
                tau(i) += frictionCoul[i] * sign(dq) + frictionVis[i] * dq;
        }
    }

    if (joints.size() > 1) {
        VM2 q(joints[0]->getPosition(), joints[1]->getPosition());
        VM2 dq(joints[0]->getVelocity(), joints[1]->getVelocity());
        tau += singularityDampingTorque(q, dq);
    }

    // Reserve torque authority for collision repulsion and keep the total
    // command in the same direction as that repulsion on each joint.
    for (unsigned int i = 0; i < joints.size() && i < 2; i++) {
        const double collisionTorque = std::clamp(collisionPriorityTorque(i),
                                                   -tauMax, tauMax);
        if (collisionTorque > 0.0)
            tau(i) = collisionTorque +
                     std::clamp(tau(i), -collisionTorque, tauMax - collisionTorque);
        else if (collisionTorque < 0.0)
            tau(i) = collisionTorque +
                     std::clamp(tau(i), -tauMax - collisionTorque, -collisionTorque);
        else
            tau(i) = std::clamp(tau(i), -tauMax, tauMax);
    }

    return applyTorque({tau[0], tau[1]});
}


// ═══════════════════════════════════════════════════════════════════════════════
// Private drive helpers
// ═══════════════════════════════════════════════════════════════════════════════

setMovementReturnCode_t RobotMTR::applyTorque(vector<double> torques) {
    lastCommandedTorque = VM2(torques[0], torques.size() > 1 ? torques[1] : 0.0);
    int i = 0;
    setMovementReturnCode_t ret = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t code = ((JointMT *)p)->setTorque(torques[i]);
        if (code == INCORRECT_MODE) {
            spdlog::error("MTR joint {}: not in torque control.", p->getId());
            ret = INCORRECT_MODE;
        } else if (code != SUCCESS) {
            spdlog::error("MTR joint {} torque error: {}",
                          p->getId(), setMovementReturnCodeString[code]);
            ret = UNKNOWN_ERROR;
        }
        i++;
    }
    return ret;
}

setMovementReturnCode_t RobotMTR::applyPosition(vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t ret = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t code = ((JointMT *)p)->setPosition(positions[i]);
        if (code == INCORRECT_MODE) {
            spdlog::error("MTR joint {}: not in position control.", p->getId());
            ret = INCORRECT_MODE;
        } else if (code != SUCCESS) {
            spdlog::error("MTR joint {} position error: {}",
                          p->getId(), setMovementReturnCodeString[code]);
            ret = UNKNOWN_ERROR;
        }
        i++;
    }
    return ret;
}

setMovementReturnCode_t RobotMTR::applyVelocity(vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t ret = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t code = ((JointMT *)p)->setVelocity(velocities[i]);
        if (code == INCORRECT_MODE) {
            spdlog::error("MTR joint {}: not in velocity control.", p->getId());
            ret = INCORRECT_MODE;
        } else if (code != SUCCESS) {
            spdlog::error("MTR joint {} velocity error: {}",
                          p->getId(), setMovementReturnCodeString[code]);
            ret = UNKNOWN_ERROR;
        }
        i++;
    }
    return ret;
}


// ═══════════════════════════════════════════════════════════════════════════════
// State readers
// ═══════════════════════════════════════════════════════════════════════════════

const VX &RobotMTR::getEndEffPosition()   { return endEffPositions;   }
const VX &RobotMTR::getEndEffVelocity()   { return endEffVelocities;  }
const VX &RobotMTR::getEndEffForce()      { return endEffForces;      }
const VX &RobotMTR::getInteractionForce() { return interactionForces; }


// ═══════════════════════════════════════════════════════════════════════════════
// Diagnostics
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::printStatus() {
    cout << fixed << setprecision(3) << showpos;
    cout << "X=["  << getEndEffPosition().transpose() << " ]  ";
    cout << "dX=[" << getEndEffVelocity().transpose() << " ]  ";
    cout << "F=["  << getEndEffForce().transpose()    << " ]" << endl;
    cout << noshowpos;
}

void RobotMTR::printJointStatus() {
    cout << fixed << setprecision(1) << showpos;
    cout << "q=["   << getPosition().transpose() * 180.0 / M_PI << " deg]  ";
    cout << "dq=["  << getVelocity().transpose() * 180.0 / M_PI << " deg/s]  ";
    cout << "tau=[" << getTorque().transpose()                   << " N·m]  {";
    for (auto joint : joints)
        cout << "0x" << hex << ((JointMT *)joint)->getDriveStatus() << "; ";
    cout << "}" << endl;
    cout << noshowpos;
}
