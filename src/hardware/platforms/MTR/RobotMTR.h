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
 *   JointMT applies reductionRatio=15 so all joint-level values are in output-shaft
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
#include "LCD1602.h"        // 16×2 LCD via PCF8574 I2C backpack
#include "I2CMPU6050IMU.h"  // MPU-6050 IMU — device orientation (Left/Right) detection
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


class RobotMTR : public Robot {
   public:
    // Original two-arg constructor retained for backward compatibility (master: has UI hardware)
    RobotMTR(const std::string &robot_name      = "RobotMTR",
             const std::string &yaml_config_file = "");

    // Overload allowing explicit drive node ids (single-drive test wrapper uses this).
    // isMaster: true (default) constructs the Keyboard/RotaryEncoder/LCD1602 UI hardware —
    // only one physical unit should be master. A non-master instance leaves keyboard/encoder/lcd
    // as nullptr: it only drives its motors over CAN, matching the doc's "listening BB" role.
    RobotMTR(const std::string &robot_name,
             const std::string &yaml_config_file,
             const std::vector<int> &drive_node_ids,
             bool isMaster = true);
    ~RobotMTR();

    bool isMaster() const { return isMaster_; }

    Keyboard      *keyboard = nullptr;  //!< Keyboard input (fallback / development) — master only
    RotaryEncoder *encoder  = nullptr;  //!< KY-040 rotary encoder (S1/CLK=P8_11, S2/DT=P8_12, Key/SW=P8_15) — master only
    LCD1602       *lcd      = nullptr;  //!< 16×2 I2C LCD (SDA=P9_20, SCL=P9_19, addr=0x27) — master only

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

    // ── Drive mode initialisation ─────────────────────────────────────────────
    bool initTorqueControl()   override;
    bool initPositionControl() override;
    bool initVelocityControl() override;

    // ── CORC pure-virtual interface ───────────────────────────────────────────
    bool initialiseJoints()  override;
    bool initialiseNetwork() override;
    bool initialiseInputs()  override;
    void updateRobot()       override;

    // ── Safety ────────────────────────────────────────────────────────────────
    setMovementReturnCode_t safetyCheck();
    bool hasSafetyTriggered() const { return safetyTriggered_; }

    // ── Calibration ───────────────────────────────────────────────────────────
    void applyCalibration();
    bool isCalibrated() const { return calibrated; }
    void decalibrate()        { calibrated = false; }
    double getTauMax()  const { return tauMax; }

    // ── Kinematics & dynamics ─────────────────────────────────────────────────
    /** 2×2 planar Jacobian. */
    Eigen::Matrix2d J();

    /** 2R planar forward kinematics → end-effector position (x, y). */
    VM2 directKinematic(VM2 q);

    /** Returns VM2::Zero() — horizontal plane, gravity is perpendicular. */
    VM2 calculateGravityTorques();

    // ── Joint-space setters ───────────────────────────────────────────────────
    setMovementReturnCode_t setJointTorque(VM2 tau);
    setMovementReturnCode_t setJointPosition(VM2 q);
    setMovementReturnCode_t setJointVelocity(VM2 dq);

    // ── Task-space setter ─────────────────────────────────────────────────────
    /** τ = Jᵀ·F + τ_friction  (no gravity term — horizontal plane). */
    setMovementReturnCode_t setEndEffForceWithCompensation(VM2 F,
                                                           bool friction_comp = true);

    // ── State readers (VX for FLNLHelper / CSV logging) ──────────────────────
    const VX &getEndEffPosition();
    const VX &getEndEffVelocity();
    const VX &getEndEffForce();
    const VX &getInteractionForce();

    void printStatus();
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

    // Drive envelope — loaded from MTR_params.yaml (these are YAML defaults; YAML overrides at runtime)
    double dqMax        = 200.0 * M_PI / 180.0;  //!< Max JOINT speed  [rad/s] (200 deg/s joint = 3000 deg/s motor)
    double tauMax       =   3.0;                  //!< Max JOINT torque [N·m]   (3.0 N·m joint = 0.2 N·m motor)
    double tauSafetyMax =   6.0;                  //!< Measured-torque e-stop [N·m joint]; must be > tauMax

    // Per-joint drive parameters (index 0 = proximal/shoulder, index 1 = distal/elbow)
    std::vector<double> iPeakDrives  = {2.795, 2.795};  //!< Maxon EC60 rated current [A]  (VERIFIED)
    std::vector<double> motorCstt    = {0.114, 0.114};  //!< Maxon EC60 torque constant Kt [N·m/A]
    std::vector<double> qSigns       = {-1.0,   1.0};   //!< Sign correction (VERIFIED from spin tests)

    // Friction model — zero until system identification; see MTR_params.yaml
    std::vector<double> frictionVis  = {0.0, 0.0};   //!< Viscous  [N·m·s/rad]
    std::vector<double> frictionCoul = {0.0, 0.0};   //!< Coulomb  [N·m]

    // Joint limits [rad] layout: {θ1_min, θ1_max, θ2_min, θ2_max}
    // TODO: LIMITS REMOVED NEED TO BE REINSTATED ONCE HARDWARE VERIFIED — see MTR_params.yaml
    std::vector<double> qLimits = {
        -360.0 * M_PI / 180.0,    360.0 * M_PI / 180.0,   // θ₁ ∈ [−30°, 100°]
        -360.0 * M_PI / 180.0,    160.0 * M_PI / 180.0    // θ₂ ∈ [−160°, −30°]
    };

    // MUST VERIFY against physical robot — measure true joint angles at each hard stop.
    // Joint 0 stops at θ₁_max; joint 1 stops at θ₂_min (see MTR_params.yaml for procedure).
    VM2 qCalibration = {
         0.0 * M_PI / 180.0,   // θ₁_max hard stop [rad] — overridden by YAML
         0.0 * M_PI / 180.0    // θ₂_min hard stop [rad] — overridden by YAML
    };

    bool calibrated      = false;
    int  calibGraceCycles_ = 0;   // skip position limits for N cycles after applyCalibration()
    bool safetyTriggered_ = false; // true while robot is disabled due to a safety event
    bool isMaster_        = true; // true: constructs keyboard/encoder/lcd/imu; false: CAN-only listener

    // ── Device orientation (Left/Right) ───────────────────────────────────────
    I2CMPU6050IMU *imu = nullptr;   // master only — see armSide() comment above
    ArmSide armSide_             = ArmSide::UNKNOWN;
    double  armSideAxisFiltered_ = 0.0;   // low-pass filtered "up axis" reading [m/s^2]

    /**
     * \brief Read the IMU, low-pass filter it, and update armSide_ via a dead-zone
     * (Schmitt-trigger) decision. No-op if imu is null (listener / no IMU).
     * Called every updateRobot() cycle — this is a filtered read + comparison, not a
     * safety check, so there's no reason to throttle it.
     */
    void updateArmSide();

    // ── Safety envelope ───────────────────────────────────────────────────────
    double maxEndEffVel   = 2.0;    //!< Max end-effector speed      [m/s]
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
    setMovementReturnCode_t applyTorque(std::vector<double> torques);
    setMovementReturnCode_t applyPosition(std::vector<double> positions);
    setMovementReturnCode_t applyVelocity(std::vector<double> velocities);
};

#endif  // ROBOTMTR_H
