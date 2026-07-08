#include "JointMT.h"


JointMT::JointMT(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max, double ipeak, double motor_kt, Drive *drive, const std::string& name) :   Joint(jointID, q_min, q_max, drive, name),
                                                                                                                                                                sign(sign_),
                                                                                                                                                                qMin(q_min), qMax(q_max),
                                                                                                                                                                dqMin(dq_min), dqMax(dq_max),
                                                                                                                                                                tauMin(tau_min), tauMax(tau_max),
                                                                                                                                                                Ipeak(ipeak),
                                                                                                                                                                motorTorqueConstant(motor_kt)
                                                                                                                                                                {
                                                                                                                                                                    spdlog::debug("MY JOINT ID: {} ({})", this->id, name);
                                                                                                                                                                }

JointMT::~JointMT() {
   
    delete drive;
}

setMovementReturnCode_t JointMT::safetyCheck() {
    if (velocity > dqMax || velocity < dqMin) {
        static int velErrCount = 0;
        if (++velErrCount % 400 == 1)
            spdlog::error("JointMT {}: velocity limit  vel={:.1f} limit=±{:.1f} deg/s",
                          id, velocity * 180.0 / M_PI, dqMax * 180.0 / M_PI);
        return OUTSIDE_LIMITS;
    }
    // Measured torque is not checked here: setTorque() limits commanded torque,
    // and the post-calibration e-stop (tauSafetyMax) lives in RobotMTR::safetyCheck()
    // so that wake-and-wiggle phasing current never triggers it.
    return SUCCESS;
}

setMovementReturnCode_t JointMT::setPosition(double qd) {
    if (calibrated) {
        if (qd >= qMin && qd <= qMax && std::isfinite(qd)) {
            return Joint::setPosition(qd);
        } else {
            return OUTSIDE_LIMITS;
        }
    } else {
        return NOT_CALIBRATED;
    }
}

setMovementReturnCode_t JointMT::setVelocity(double dqd) {
    //Position protection first only if calibrated
    if (calibrated) {
        if (position <= qMin && dqd < 0) {
            dqd = 0;
        }
        if (position >= qMax && dqd > 0) {
            dqd = 0;
        }
    }
    //Caped velocity
    if (dqd >= dqMin && dqd <= dqMax && std::isfinite(dqd)) {
        return Joint::setVelocity(dqd);
    } else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointMT::setTorque(double taud) {
    //Position protection first only if calibrated
    if (calibrated) {
        if (position <= qMin && taud < 0) {
            taud = 0;
        }
        if (position >= qMax && taud > 0) {
            taud = 0;
        }
    }
    //Caped torque
    if (taud >= tauMin && taud <= tauMax && std::isfinite(taud)) {
        return Joint::setTorque(taud);
    } else {
        return OUTSIDE_LIMITS;
    }
}

bool JointMT::initNetwork() {
    spdlog::debug("JointMT::initNetwork()");
    drive->start();
    if (drive->initPDOs()) {
        return true;
    } else {
        return false;
    }
}
