/**
 * \file JointMT.h
 * \author Vincent Crocher
 * \brief An M3 actuated joint
 * \version 0.2
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef JOINTM3_H_INCLUDED
#define JOINTM3_H_INCLUDED

#include <iostream>
#include <cmath>

#include "Joint.h"
#include "CopleyDrive.h"


typedef struct JointDrivePairs {
    int drivePosA;
    int drivePosB;
    double jointPosA;
    double jointPosB;
} JointDrivePairs;

/**
 * \brief MTR actuated joints, using Copley drives.
 *
 */
class JointMT : public Joint {
   private:
    const short int sign;
    const double qMin, qMax, dqMin, dqMax, tauMin, tauMax;

    // Maxon EC60 encoder: 8192 counts per motor revolution (verified on hardware).
    int encoderCounts = 8192;
    // 1:15 gearbox between motor shaft and joint output.
    // All conversion functions divide/multiply by reductionRatio so that getPosition(),
    // getVelocity(), getTorque() return joint-space values and setTorque() accepts
    // joint torques and internally scales to motor torque.
    double reductionRatio = 15.0;

    // Torque scaling: 1000 drive units (DS402 0x6071) = Ipeak × Kt = rated motor torque.

    // VERIFIED: 0x6076 = 319 mN·m = 2.795 A × 0.114 N·m/A. Joint rated = 319 mN·m × 15 = 4785 mN·m.
    double Ipeak = 2.795;                //!< Maxon EC60 rated current [A]  (0.319 N·m / 0.114 N·m/A)
    double motorTorqueConstant = 0.114;  //!< Maxon EC60 torque constant [N·m/A]

    double driveUnitToJointPosition(int driveValue) { return sign * driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return sign * driveValue * (2. * M_PI) / 60. / 512. / (double)encoderCounts * 1875 / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * 60. * 512. * (double)encoderCounts / 1875 * reductionRatio; };
    
    // Torque conversion verified: 1000 units → Ipeak × Kt × reductionRatio = 4.785 N·m joint rated.
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / 1000.0 * motorTorqueConstant * Ipeak * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return (int)(sign * jointValue / (motorTorqueConstant * Ipeak) * 1000.0 / reductionRatio); };

   public:
    JointMT(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, double ipeak = 2.795, double motor_kt = 0.114, Drive *drive = NULL, const std::string& name="");
    ~JointMT();
    /**
     * \brief Check if current velocity and torque are within limits.
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();

    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);

    bool initNetwork();
};

#endif
