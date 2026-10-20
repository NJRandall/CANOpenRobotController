/**
 * \file RobotMTR.h
 * \brief RobotMTR — 2-DOF parallel-linkage tabletop mirror-therapy robot.
 *
 * Kinematic model
 * ───────────────
 * Standard 2D planar robot in the horizontal plane (gravity ⊥ workspace).
 * Parameters loaded from config/MTR_params.yaml:
 *   L1            — proximal link length [m]  (shoulder → elbow pivot)
 *   L2            — distal  link length [m]  (elbow pivot → handle)
 *   parallel_ratio — gear ratio of the parallelogram linkage on joint 2
 *                    (motor encoder counts / actual elbow angle change)
 *
 * Joint convention
 * ────────────────
 *   q[0] = θ₁   proximal joint angle [rad], measured from the +X axis
 *   q[1] = θ₂   distal  joint angle  [rad], measured from link-1 direction
 *
 * Forward kinematics (System Modelling/torquecalc (1).m):
 *   x = L1·cos(θ₁) + L2·cos(θ₁ + θ₂·r)
 *   y = L1·sin(θ₁) + L2·sin(θ₁ + θ₂·r)   where r = parallel_ratio
 *
 * Jacobian — 2×2
 * ────────────────────────────────────────────────────────────────────
 *   J = │−L1·s₁ − L2·s₁₂   −L2·s₁₂│   ← ẋ
 *       │ L1·c₁ + L2·c₁₂    L2·c₁₂│   ← ẏ
 *
 *   det(J) = L1·L2·sin(θ₂·r)  — always non-zero in the operating range
 *   θ₂ ∈ [−160°, −30°] keeps the arm away from fully extended (0°) and
 *   fully folded (±180°), so the Jacobian is always invertible.
 *
 * Gravity torques
 * ────────────────
 *   Zero — arm moves in a horizontal plane; gravity is normal to the workspace.
 *   Placeholder functions exist for inertia/Coriolis once modelling is complete.
 *
 * Drive hardware
 * ──────────────
 *   2 × Copley ACK-055-06 drives on CAN bus (node IDs 1 and 3), each driving a
 *   Maxon EC60 BLDC motor through a 1:15 gearbox.
 *   VERIFIED: 0x6076 (Motor Rated Torque) = 319 mN·m = 2.795 A × 0.114 N·m/A.
 *   JointMT applies the per-joint reductionRatios from MTR_params.yaml (15 by default)
 *   so all joint-level values are in output-shaft
 *   units: positions [rad], velocities [rad/s], torques [N·m at joint].
 *
 * Workspace joint limits (System Modelling/ROM (1).m, accelCalc (1).m)
 * ────────────────────────────────────────────────────────────────────
 *   θ₁ ∈ [−30°,  100°]
 *   θ₂ ∈ [−160°, −30°]   (always negative — elbow bends toward the patient)
 *
 * Calibration (STEP 2 / STEP 3 — see MTRobotStates.h)
 * ─────────────────────────────────────────────────────
 *   CalibState drives all joints with a positive torque until motion stops.
 *   qCalibration holds the KNOWN angles at those hard stops.
 *   Verify qCalibration via SDO read of 0x6064 at each stop before enabling
 *   CalibState as the initial state.
 */

#ifndef ROBOTMTR_H
#define ROBOTMTR_H

#include "JointMT.h"        // Copley-drive-backed joint
#include "Keyboard.h"       // Keyboard input (development / fallback)
#include "RotaryEncoder.h"  // KY-040 rotary encoder with push button
#include <memory>
#include "LCD1602.h"        // 16×2 LCD via PCF8574 I2C backpack
// IMU disabled — not in use for now. Uncomment to re-enable.
// #include "I2CMPU6050IMU.h"  // MPU-6050 IMU — device orientation (Left/Right) detection
#include "USBMouse.h"       // Optional local USB mouse — live reference for MOUSE_MIMIC/MOUSE_MIRROR
#include "Robot.h"          // CORC Robot base class
#include "CopleyDrive.h"    // Copley drive interface

typedef Eigen::Vector2d VM2;   //!< 2-vector (planar XY workspace)
typedef Eigen::VectorXd VX;    //!< Dynamic-size vector (for FLNLHelper / logging)

/**
 * Which way this physical unit is mounted, decided from its IMU reading.
 * UNKNOWN until the dead zone around the flip boundary is cleared at least once
 * (see RobotMTR::updateArmSide()).
 */
enum class ArmSide { UNKNOWN, LEFT, RIGHT };

/** The other side: LEFT<->RIGHT, UNKNOWN stays UNKNOWN. Used to infer the listener's side
 *  from the master's measured side — see RobotMTR::setArmSide(). */
inline ArmSide oppositeArmSide(ArmSide s) {
    if (s == ArmSide::LEFT)  return ArmSide::RIGHT;
    if (s == ArmSide::RIGHT) return ArmSide::LEFT;
    return ArmSide::UNKNOWN;
}

struct MTRLinkHitbox {
    VM2 start;       //!< Centreline start point in the shared workspace [m]
    VM2 end;         //!< Centreline end point in the shared workspace [m]
    double radius;   //!< Physical link radius [m]
    double boundary; //!< Additional collision clearance outside the radius [m]
};

struct MTRJointHitbox {
    VM2 center;      //!< Joint centre in the shared workspace [m]
    double radius;   //!< Physical joint radius [m]
    double boundary; //!< Additional collision clearance outside the radius [m]
};

struct MTRFrameHitbox {
    VM2 center;       //!< Rectangle centre in the shared workspace [m]
    VM2 halfExtents;  //!< Half-width and half-height in the frame-local axes [m]
    double yaw;       //!< Rectangle orientation in the shared workspace [rad]
    double boundary;  //!< Additional clearance around the rectangle [m]
};

struct MTRCollisionHitboxes {
    MTRLinkHitbox L1; //!< Primary link from the origin using q1
    MTRLinkHitbox L2; //!< Primary link from the end of L1 using q2
    MTRLinkHitbox A;  //!< Parallel link from the origin using q2
    MTRLinkHitbox B;  //!< Parallel link from the end of A using q1
    MTRJointHitbox origin;
    MTRJointHitbox elbowL1L2;
    MTRJointHitbox elbowAB;
    MTRJointHitbox endEffector;
    MTRFrameHitbox frame;
};

class RobotMTR : public Robot {
   public:
    // With no explicit node list, drive_node_ids are loaded from YAML.
    RobotMTR(const std::string &robot_name      = "RobotMTR",
             const std::string &yaml_config_file = "");

    // Overload allowing explicit drive node ids (single-drive test wrapper uses this).
    // isMaster: true (default) constructs the Keyboard/RotaryEncoder/LCD1602 UI hardware —
    // only one physical unit should be master. A non-master instance leaves keyboard/encoder/lcd
    // as nullptr: it only drives its motors over CAN, acting as a CAN-only listener.
    RobotMTR(const std::string &robot_name,
             const std::string &yaml_config_file,
             const std::vector<int> &drive_node_ids,
             bool isMaster = true);
    ~RobotMTR();

    bool isMaster() const { return isMaster_; }
    const std::string &getName() const { return robotName; }   //!< YAML unit name, e.g. "v1_unit"

    // Stop this unit reading the shared keyboard/encoder/mouse, so they are updated once per
    // cycle by the unit that owns the UI (active robot 0). The shared LCD is kept for display.
    void releaseUserInputs();

    Keyboard      *keyboard = nullptr;  //!< Keyboard input (fallback / development) — master only
    RotaryEncoder *encoder  = nullptr;  //!< KY-040 rotary encoder (S1/CLK=P8_11, S2/DT=P8_12, Key/SW=P8_15) — master only
    LCD1602       *lcd      = nullptr;  //!< 16×2 I2C LCD (SDA=P9_20, SCL=P9_19, addr=0x27) — master only; one object shared by all units on this board
    USBMouse      *mouse    = nullptr;  //!< Optional local USB mouse (/dev/input/mouse0) — master only

    // ── Device orientation (Left/Right) ───────────────────────────────────────
    // Only the master reads an IMU: master and listener are two separate BeagleBones, and I2C
    // is a local bus (not networked), so the master's process can't read a listener-side IMU
    // directly anyway. In this project's fixed bimanual mount the two arms are necessarily
    // mounted 180° apart, so the listener's side is simply the logical opposite of whatever
    // the master's IMU reads — no second reading is needed for the configurations this
    // project currently supports.
    // FUTURE EXTENSION: if a configuration ever needs the listener's orientation measured
    // rather than inferred (e.g. a non-fixed/independent mount), the listener would need its
    // own local IMU read on its own board and the result sent to the master over the existing
    // master<->listener channel — a second I2CMPU6050IMU here would not be reachable from the
    // master's process. See setArmSide() below for how that result would be applied.
    ArmSide armSide() const { return armSide_; }

    /** Manually set this robot's side — used to apply the listener's inferred (not measured)
     *  orientation once the master's own IMU reading has been decided. */
    void setArmSide(ArmSide s) { armSide_ = s; }

    // ── Bimanual control role ──────────────────────────────────────────────────
    // Which arm drives the live reference in MultiControllerState. Unrelated to isMaster()
    // (BB role) or armSide() (mounting). Chosen via the master's encoder in CalibState.
    bool isLeader() const { return isLeader_; }
    void setLeader(bool v) { isLeader_ = v; }

    // ── Drive mode initialisation ─────────────────────────────────────────────
    bool initTorqueControl()   override;
    bool initPositionControl() override;
    bool initVelocityControl() override;
    int  getJointDriveStatus(unsigned int i) const { return i < joints.size() ? joints[i]->getDriveStatus() : -1; }
    void printDriveDiagnostics();   //!< SDO-read and print key DS402/Copley registers of every drive

    // ── CORC pure-virtual interface ───────────────────────────────────────────
    bool initialiseJoints()  override;
    bool initialiseNetwork() override;
    bool initialiseInputs()  override;
    void updateRobot()       override;
    std::size_t probeDriveNodes() const;

    // ── Safety ────────────────────────────────────────────────────────────────
    setMovementReturnCode_t safetyCheck();
    bool hasSafetyTriggered() const { return safetyTriggered_; }
    void clearSafetyTrigger()       { safetyTriggered_ = false; }  // called on EmergencyStop reset
    // EmergencyStop reset: clears drive faults and confirms every drive reaches Ready to
    // Switch On with bus voltage present. Leaves the drives disabled; false if any is not ready.
    bool recoverDrives();

    // ── Calibration ───────────────────────────────────────────────────────────
    void applyCalibration();
    bool isCalibrated() const { return calibrated; }
    void decalibrate()        { calibrated = false; }
    const VM2 &calibrationPosition() const { return qCalibration; }
    double getTauMax()  const { return tauMax; }
    bool automaticCalibrationEnabled() const { return automaticCalibrationEnabled_; }
    const std::vector<int> &configuredDriveNodeIds() const { return driveNodeIds; }
    const VM2 &calibrationDirection() const { return calibrationDirection_; }
    const VM2 &calibrationTorque() const { return calibrationTorque_; }
    bool collisionUsesHitboxes() const { return collisionUseHitboxes_; }
    double collisionPredictionHorizon() const { return collisionPredictionHorizon_; }
    double collisionInfluenceDistance() const { return collisionInfluenceDistance_; }
    double collisionSpringGain() const { return collisionSpringGain_; }
    double collisionDecayLength() const { return collisionDecayLength_; }
    double collisionDampingGain() const { return collisionDampingGain_; }
    // Mirror-mode midline (x = 0) collision line for the leader
    double midlineInfluenceDistance() const { return midlineInfluenceDistance_; }
    double midlineSpringGain() const { return midlineSpringGain_; }
    double midlineDecayLength() const { return midlineDecayLength_; }
    double midlineDampingGain() const { return midlineDampingGain_; }
    double controllerStiffness() const { return controllerStiffness_; }   //!< Spring K [N/m]
    double controllerDamping() const { return controllerDamping_; }       //!< Damping D [N·s/m]
    double alignSpeed() const { return alignSpeed_; }   //!< Max target speed when moving into a reference pose [m/s]
    bool bimanualAsymmetricSpring() const { return bimanualAsymmetricSpring_; }   //!< Reflect follower spring onto leader
    double bimanualReflectionRatio() const { return bimanualReflectionRatio_; }     //!< Fraction of follower force reflected
    //! Live tuning (GUI slider), starting from the YAML values. Ratio 0 switches the asymmetric spring off; any positive ratio switches it on.
    void setBimanualReflection(double ratio) {
        bimanualReflectionRatio_ = std::max(0.0, ratio);
        bimanualAsymmetricSpring_ = ratio > 0.0;
    }
    //! Share (0-1) of the YAML friction model applied by friction compensation (GUI slider).
    double frictionScale() const { return frictionScale_; }
    void setFrictionScale(double scale) { frictionScale_ = std::min(1.0, std::max(0.0, scale)); }
    double parkStiffness() const { return parkStiffness_; }   //!< Parked-arm joint hold K [N·m/rad]
    double parkDamping() const { return parkDamping_; }       //!< Parked-arm joint hold D [N·m·s/rad]
    double parkMaxTorque() const { return parkMaxTorque_; }   //!< Parked-arm per-joint torque cap [N·m]
    double parkSpeed() const { return parkSpeed_; }           //!< Parked-arm joint target speed [rad/s]
    bool collisionLogOnly() const { return collisionLogOnly_; }

    // ── Kinematics & dynamics ─────────────────────────────────────────────────
    /** 2×2 planar Jacobian in this robot's local (shoulder) frame. */
    Eigen::Matrix2d J();

    /** Forward kinematics → end-effector position (x, y) in the shared task frame. */
    VM2 directKinematic(VM2 q);

    // ── Shared task frame ─────────────────────────────────────────────────────
    // Every task-space quantity this class exposes (end-effector position, velocity, force,
    // setEndEffForce* input) is in one frame shared by both units: origin at the centre
    // between the two shoulder joints, common x/y axes. task_origin (YAML) is that origin's
    // position seen from this robot's shoulder in its local axes; task_mirror_x (mirror-image
    // arm) reverses the local x axis and task_yaw then rotates it into the shared axes. The
    // defaults (origin at the shoulder, no mirror, no rotation) reproduce the old local frame.
    VM2 toTaskFrame(const VM2 &localPoint) const  { return taskRotation_ * (localPoint - taskOrigin_); }
    VM2 fromTaskFrame(const VM2 &taskPoint) const { return taskRotation_.transpose() * taskPoint + taskOrigin_; }
    VM2 toTaskVector(const VM2 &localVector) const  { return taskRotation_ * localVector; }
    VM2 fromTaskVector(const VM2 &taskVector) const { return taskRotation_.transpose() * taskVector; }

    /** Returns VM2::Zero() — horizontal plane, gravity is perpendicular. */
    VM2 calculateGravityTorques();

    /** Build the four link hitboxes from the current calibrated joint positions. */
    MTRCollisionHitboxes getCollisionHitboxes() const;

    /** Build the four link hitboxes for a supplied joint configuration. */
    MTRCollisionHitboxes getCollisionHitboxes(VM2 q) const;
    VM2 collisionPointVelocity(std::size_t linkIndex, double fraction) const;
    VM2 collisionPointJointTorque(std::size_t linkIndex, double fraction,
                                  const VM2 &force) const;

    // ── Joint-space setters ───────────────────────────────────────────────────
    setMovementReturnCode_t setJointTorque(VM2 tau,
                                            bool applySingularityDamping = true,
                                            bool applyLimitDamping = true);
    setMovementReturnCode_t setJointPosition(VM2 q);
    setMovementReturnCode_t setJointVelocity(VM2 dq);

    // ── Task-space setter ─────────────────────────────────────────────────────
    /** τ = Jᵀ·F + τ_friction  (no gravity term — horizontal plane). */
    setMovementReturnCode_t setEndEffForceWithCompensation(VM2 F,
                                                           bool friction_comp = true);
    setMovementReturnCode_t setEndEffForceWithCompensation(
        VM2 F, bool friction_comp, const VM2 &collisionPriorityTorque);

    // ── State readers (VX for FLNLHelper / CSV logging) ──────────────────────
    const VX &getEndEffPosition();
    const VX &getEndEffVelocity();
    const VX &getEndEffForce();
    const VX &getInteractionForce();

    void printStatus(const char *role = nullptr);   //!< Prefixed with [unit role]
    void printSingularityStatus();                  //!< Elbow angle and singularity damping torque
    void printJointStatus();

   private:
    // ─────────────────────────────────────────────────────────────────────────
    // Parameters — defaults from system modelling; overridden by YAML if provided
    // ─────────────────────────────────────────────────────────────────────────

    // TO DO: update once YAML file confirmed.
    // Geometry (System Modelling/torquecalc (1).m) — MUST match YAML
    double L1             = 0.43;   //!< Proximal link [m]  (MUST VERIFY)
    double L2             = 0.37;   //!< Distal   link [m]  (MUST VERIFY)
    double parallel_ratio = 1.0;    //!< Joint-2 parallelogram transmission ratio
    VM2 taskOrigin_ = VM2::Zero();                                 //!< Shared origin in local frame [m]
    Eigen::Matrix2d taskRotation_ = Eigen::Matrix2d::Identity();  //!< Local → shared axes (may mirror)

    // Robot pose and link collision geometry in the shared workspace.
    double collisionLinkThickness = 0.08;        //!< Link diameter [m]
    double collisionBoundary = 0.02;             //!< Extra clearance [m]
    double collisionLinkALength = 0.0;           //!< Parallel link A length [m]
    double collisionJointRadius = 0.05;          //!< Physical joint radius [m]
    double collisionEndEffectorRadius = 0.12;    //!< End-effector hitbox radius [m] (MUST VERIFY)
    double collisionFrameCenterX = 0.0;           //!< Frame centre x in base frame [m]
    double collisionFrameCenterY = 0.0;           //!< Frame centre y in base frame [m]
    double collisionFrameWidth = 0.60;            //!< Frame width [m]
    double collisionFrameHeight = 0.40;           //!< Frame height [m]
    double collisionFrameBoundary = 0.02;         //!< Frame clearance [m]

    // Drive envelope — loaded from MTR_params.yaml (these are YAML defaults; YAML overrides at runtime)
    double dqMax        = 200.0 * M_PI / 180.0;  //!< Max JOINT speed  [rad/s] (200 deg/s joint = 3000 deg/s motor)
    double tauMax       =   3.0;                  //!< Max JOINT torque [N·m]   (3.0 N·m joint = 0.2 N·m motor)
    double tauSafetyMax =   6.0;                  //!< Measured-torque e-stop [N·m joint]; must be > tauMax
    double qDampingWidth = 10.0 * M_PI / 180.0;   //!< Soft-limit damping band [rad]
    double qDampingGain  = 0.5;                   //!< Soft-limit viscous gain [N·m·s/rad]
    double singularityDampingAngleExtended = 15.0 * M_PI / 180.0; //!< Extended singularity band [rad]
    double singularityDampingAngleFolded   = 15.0 * M_PI / 180.0; //!< Folded singularity band [rad]
    double singularityDampingGain          = 0.5;                 //!< Singularity viscous gain [N·m·s/rad]
    // Elbow angles (q2 - q1) of the mechanical stops that keep the elbow off its singularities.
    // Each damping band ramps from zero at (stop ± band) to full gain at the stop. The defaults
    // are the singularities themselves (0 and 180 deg).
    double singularityExtendedStop         = 0.0;                 //!< Extended elbow stop [rad]
    double singularityFoldedStop           = M_PI;                //!< Folded elbow stop [rad]

    // Per-joint drive parameters (index 0 = proximal/shoulder, index 1 = distal/elbow)
    std::vector<int> driveNodeIds = {1, 3};          //!< CAN node for each joint
    std::vector<double> iPeakDrives  = {2.795, 2.795};  //!< Maxon EC60 rated current [A]  (VERIFIED)
    std::vector<double> motorCstt    = {0.114, 0.114};  //!< Maxon EC60 torque constant Kt [N·m/A]
    std::vector<double> reductionRatios = {15.0, 15.0}; //!< Gearbox ratio motor:joint (1:15)
    std::vector<double> encoderCounts   = {4096, 4096}; //!< Encoder counts per motor revolution
    std::vector<double> qSigns       = {-1.0,   -1.0};   //!< Sign correction (VERIFIED from spin tests)

    // Friction model — zero until system identification; see MTR_params.yaml
    std::vector<double> frictionVis  = {0.0, 0.0};   //!< Viscous  [N·m·s/rad]
    std::vector<double> frictionCoul = {0.0, 0.0};   //!< Coulomb  [N·m]
    double frictionScale_ = 1.0;                      //!< Applied share of the model above

    // Joint limits [rad] layout: {θ1_min, θ1_max, θ2_min, θ2_max}
    // TODO: LIMITS REMOVED NEED TO BE REINSTATED ONCE HARDWARE VERIFIED — see MTR_params.yaml
    // Joints are declared as larger link: Joint 0 = θ1_min, θ1_max and smaller link: Joint 1 = θ2_min, θ2_max
    // Single joint limits are specified as {θ_min, θ_max} for each joint

    //!!!!!!!!! Angles between inaccurate joint 0 at 90 real is 80 in software (TO DO: VERIFY) !!!!!!!!!!!!!!!!!!

    std::vector<double> qLimits = {
        -12.0 * M_PI / 180.0,    180.0 * M_PI / 180.0,   // θ₁ ∈ [−10+(2 safety)°, 180°]
        15.0 * M_PI / 180.0,    245.0 * M_PI / 180.0    // θ₂ ∈ [15°, 245°]
    };

    // TODO !!!!MUST VERIFY against physical robot — measure true joint angles at each hard stop.
    // Joint 0 stops at θ₁_max; joint 1 stops at θ₂_min (see MTR_params.yaml for procedure).
    VM2 qCalibration = {
         -10.0 * M_PI / 180.0,   // θ₁_max hard stop [rad] — overridden by YAML
         140.0 * M_PI / 180.0    // θ₂_min hard stop [rad] — overridden by YAML
    };

        bool automaticCalibrationEnabled_ = false;
        bool collisionUseHitboxes_ = true;
        bool collisionLogOnly_ = false;
        double collisionPredictionHorizon_ = 0.15;
        double collisionInfluenceDistance_ = 0.15;
        double collisionSpringGain_ = 0.25;
        double collisionDecayLength_ = 0.05;
        double collisionDampingGain_ = 2.0;
        double midlineInfluenceDistance_ = 0.08;   // midline_influence_distance [m]
        double midlineSpringGain_ = 1.0;           // midline_spring_gain [N]
        double midlineDecayLength_ = 0.03;         // midline_decay_length [m]
        double midlineDampingGain_ = 30.0;         // midline_damping_gain [N*s/m]
        double controllerStiffness_ = 250.0;   // controller_stiffness (YAML) [N/m]
        double controllerDamping_ = 20.0;      // controller_damping (YAML) [N·s/m]
        double alignSpeed_ = 0.05;             // align_speed (YAML) [m/s]
        bool bimanualAsymmetricSpring_ = false;  // bimanual_asymmetric_spring (YAML)
        double bimanualReflectionRatio_ = 0.1;   // bimanual_reflection_ratio (YAML) [-]
        double parkStiffness_ = 20.0;          // park_stiffness (YAML) [N·m/rad]
        double parkDamping_ = 1.0;             // park_damping (YAML) [N·m·s/rad]
        double parkMaxTorque_ = 4.0;           // park_max_torque (YAML) [N·m]
        double parkSpeed_ = 10.0 * M_PI / 180.0;  // park_speed (YAML, deg/s) [rad/s]
        VM2 calibrationDirection_ = VM2(-1.0, -1.0);
        VM2 calibrationTorque_ = VM2(2.6, 2.0);

    bool calibrated      = false;
    int  calibGraceCycles_ = 0;   // skip position limits for N cycles after applyCalibration()
    bool safetyTriggered_ = false; // true while robot is disabled due to a safety event
    std::shared_ptr<LCD1602> lcdOwner_;   // keeps the shared LCD alive while this unit exists
    std::shared_ptr<Keyboard> keyboardOwner_;     // shared UI inputs, one object per board
    std::shared_ptr<RotaryEncoder> encoderOwner_;
    std::shared_ptr<USBMouse> mouseOwner_;
    bool isMaster_        = true; // true: constructs keyboard/encoder/lcd/imu; false: CAN-only listener
    bool isLeader_        = false; // bimanual control role — see isLeader() above

    // ── Device orientation (Left/Right) ───────────────────────────────────────
    // IMU disabled — not in use for now. Uncomment to re-enable.
    // I2CMPU6050IMU *imu = nullptr;   // master only — see armSide() comment above
    ArmSide armSide_             = ArmSide::UNKNOWN;
    // double  armSideAxisFiltered_ = 0.0;   // low-pass filtered "up axis" reading [m/s^2]

    // /**
     // * \brief Read the IMU, low-pass filter it, and update armSide_ via a dead-zone
     // * (Schmitt-trigger) decision. No-op if imu is null (listener / no IMU).
     // * Called every updateRobot() cycle — this is a filtered read + comparison, not a
     // * safety check, so there's no reason to throttle it.
     // */
    // void updateArmSide();

    // ── Drive liveness ─────────────────────────────────────────────────────────
    // Periodic check that each drive node still answers on the CAN bus. A non-responding
    // node sets safetyTriggered_, which safetyStop (MTRobotMachine.cpp) turns into an
    // EmergencyStop transition. Kept MTR-local: calls cancomm_socketFree() directly with the
    // same SDO-read-and-check-for-"OK" logic Drive::sendSDOMessages() already uses, rather
    // than going through Drive/CopleyDrive (shared with other robot platforms).
    std::vector<int> driveNodeIds_;            // copy of the constructor's drive_node_ids
    int livenessCheckCounter_ = 0;
    double lastSaturationWarn_ = 0.0;   // time of the last force-saturation warning [s]
    int saturationCount_ = 0;           // saturated cycles since that warning
    static constexpr int livenessCheckPeriod_ = 500;  // TODO: tune — cycles between checks;
                                                       // SDO round-trips are slow/blocking,
                                                       // unlike the PDO-based control traffic
    static bool checkDriveAlive(int nodeID);

    // ── Safety envelope ───────────────────────────────────────────────────────
    double maxEndEffVel   = 2.0;    //!< End-effector speed e-stop [m/s]; YAML maxEndEffVel
    int    maxEndEffVelCycles = 5;  //!< Consecutive over-limit cycles before the speed e-stop trips; YAML maxEndEffVelCycles
    int    endEffVelOverCount_ = 0; //!< Current run of consecutive over-limit cycles
    double maxEndEffForce = 20.0;   //!< Max end-effector force (XY) [N]

    // ── Cached end-effector state (VX size 2 for FLNLHelper compatibility) ───
    VX endEffPositions   = VX::Zero(2);
    VX endEffVelocities  = VX::Zero(2);
    VX endEffForces      = VX::Zero(2);
    VX interactionForces = VX::Zero(2);

    double last_update_time    = 0.0;
    VM2    lastCommandedTorque = VM2::Zero();

    // ── YAML parameter loading ────────────────────────────────────────────────
    bool loadParametersFromYAML(YAML::Node params) override;
    void fillParam(YAML::Node node, std::vector<double> &vec);

    // ── Private drive helpers ─────────────────────────────────────────────────
    setMovementReturnCode_t applyTorque(std::vector<double> torques, bool applyLimitDamping = true);
    setMovementReturnCode_t applyPosition(std::vector<double> positions);
    setMovementReturnCode_t applyVelocity(std::vector<double> velocities);
    VM2 singularityDampingTorque(VM2 q, VM2 dq) const;
};

#endif  // ROBOTMTR_H