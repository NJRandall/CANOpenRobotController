#include "RobotMTR.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

using namespace Eigen;
using namespace std;


// ═══════════════════════════════════════════════════════════════════════════════
// Construction / destruction
// ═══════════════════════════════════════════════════════════════════════════════

RobotMTR::RobotMTR(const string &robot_name, const string &yaml_config_file)
    : Robot(robot_name, yaml_config_file) {

    // Load YAML overrides before joints are constructed so limits are correct.
    initialiseFromYAML(yaml_config_file);
    // SHOULDER
     addJoint(new JointMT(0,
                          qLimits[0], qLimits[1],          // θ₁ min / max
                          (short int)qSigns[0],
                          -dqMax, dqMax,
                          -tauMax, tauMax,
                          iPeakDrives[0], motorCstt[0],
                          new CopleyDrive(1), "q1"));
    // ELBOW
    addJoint(new JointMT(1,
                         qLimits[2], qLimits[3],          // θ₂ min / max
                         (short int)qSigns[1],
                         -dqMax, dqMax,
                         -tauMax, tauMax,
                         iPeakDrives[1], motorCstt[1],
                         new CopleyDrive(3), "q2"));

    addInput(keyboard = new Keyboard());

    // KY-040 rotary encoder: S1(CLK)=P8_11, S2(DT)=P8_12, Key(SW)=P8_15
    addInput(encoder = new RotaryEncoder(8, 11, 8, 12, 8, 15));

    // 16×2 I2C LCD: PCF8574 at 0x27 on /dev/i2c-2
    lcd = new LCD1602(0x27, 2);
    lcd->init();

    last_update_time = chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}

RobotMTR::~RobotMTR() {
    for (auto p : joints) delete p;
    joints.clear();
    delete keyboard;
    delete encoder;
    delete lcd;
    inputs.clear();
}


// ═══════════════════════════════════════════════════════════════════════════════
// YAML parameter loading
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::fillParam(YAML::Node node, vector<double> &vec) {
    if (node) {
        for (unsigned int i = 0; i < vec.size(); i++)
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

    // Geometry — loaded from YAML so that tuning never requires a recompile
    if (p["L1"])             L1             = p["L1"].as<double>();
    if (p["L2"])             L2             = p["L2"].as<double>();
    if (p["parallel_ratio"]) parallel_ratio = p["parallel_ratio"].as<double>();

    // Drive envelope (hard-constrained for safety)
    if (p["dqMax"])       dqMax       = min(max(0., p["dqMax"].as<double>()), 3600.) * M_PI / 180.;
    if (p["tauMax"])      tauMax      = min(max(0., p["tauMax"].as<double>()), 80.);
    if (p["tauSafetyMax"]) tauSafetyMax = max(tauMax, p["tauSafetyMax"].as<double>());

    // Safety envelope
    if (p["maxEndEffForce"]) maxEndEffForce = max(0., p["maxEndEffForce"].as<double>());

    fillParam(p["qSigns"],       qSigns);
    fillParam(p["iPeakDrives"],  iPeakDrives);
    fillParam(p["motorCstt"],    motorCstt);
    fillParam(p["frictionVis"],  frictionVis);
    fillParam(p["frictionCoul"], frictionCoul);

    if (p["qLimits"]) {
        for (unsigned int i = 0; i < qLimits.size(); i++)
            qLimits[i] = p["qLimits"][i].as<double>() * M_PI / 180.;
    }

    if (p["qCalibration"]) {
        qCalibration[0] = p["qCalibration"][0].as<double>() * M_PI / 180.;
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
// getPosition() returns joint-space radians (reductionRatio=15 applied in JointMT).
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


// ═══════════════════════════════════════════════════════════════════════════════
// Robot update (called every control cycle)
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::updateRobot() {
    spdlog::trace("RobotMTR::updateRobot()");
    Robot::updateRobot();   // fills jointPositions_, jointVelocities_, jointTorques_

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

    last_update_time = chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}


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
            double tau = joints[i]->getTorque();   // joint torque [N·m] — reductionRatio=15 applied
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

setMovementReturnCode_t RobotMTR::setJointTorque(VM2 tau) {
    return applyTorque({tau[0], tau[1]});
}

setMovementReturnCode_t RobotMTR::setJointPosition(VM2 q) {
    return applyPosition({q[0], q[1]});
}

setMovementReturnCode_t RobotMTR::setJointVelocity(VM2 dq) {
    return applyVelocity({dq[0], dq[1]});
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

    // ── 4. Joint torque saturation ────────────────────────────────────────────
    for (unsigned int i = 0; i < joints.size() && i < 2; i++) {
        if (std::abs(tau(i)) > tauMax) {
            spdlog::warn("MTR: Joint {} torque saturated from {:.2f} to ±{:.2f} N·m.",
                         i, tau(i), tauMax);
            tau(i) = std::max(-tauMax, std::min(tauMax, tau(i)));
        }
    }

    return setJointTorque(tau);
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
    if (!calibrated) return NOT_CALIBRATED;
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
