/**
 *
 * \file I2CMPU6050IMU.h
 * \brief Class allowing communication with an MPU-6050 IMU (GY-521 style breakout) via I2C
 * on a BeagleBone (and maybe other).
 *
 * Unlike the BNO055 (see I2CBNO55IMU), the MPU-6050 has no onboard sensor-fusion processor —
 * it only exposes raw accelerometer and gyroscope registers. For this project's one use of the
 * IMU (deciding which way the device is mounted — see RobotMTR::sampleArmSide()), that's
 * sufficient: a stationary accelerometer reading *is* the gravity vector, so no fusion is
 * needed to tell which way is "up".
 *
 * Register map and defaults from the MPU-6050 datasheet (Register Map and Descriptions, Rev 4.2):
 *   https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * Default I2C address 0x68 (AD0 pin low); 0x69 if AD0 is tied to 3.3V.
 * Logic level is 3.3V — directly BeagleBone-compatible, no level shifter needed.
 *
 * Use `i2cdetect -y -r 1` to list addresses of devices connected on bus 1 (/dev/i2c-1) of the BB.
 */

#ifndef I2CMPU6050IMU_H
#define I2CMPU6050IMU_H
#include <Eigen/Dense>

#include "InputDevice.h"
#include "I2CDevice.h"


/** MPU-6050 registers (subset needed for raw accelerometer reading) **/
typedef enum {
    MPU6050_PWR_MGMT_1_ADDR   = 0x6B,  //!< Power management 1 — chip powers on in sleep mode (bit 6 set); write 0x00 to wake
    MPU6050_ACCEL_CONFIG_ADDR = 0x1C,  //!< Accelerometer full-scale select — default (0x00) is +/-2g, left untouched here
    MPU6050_ACCEL_XOUT_H_ADDR = 0x3B,  //!< First of 3 consecutive 16-bit signed accel registers: X, Y, Z
    MPU6050_WHO_AM_I_ADDR     = 0x75   //!< Should read back 0x68 regardless of the address pin setting
} mpu6050_reg_t;


/**
 * \ingroup IO
 * \brief Class inheriting InputDevice to handle an I2C-based MPU-6050 IMU.
 *
 * Only raw accelerometer is read (updateInput() -> getAcc()) — gyroscope is not used by
 * this project and is left unread.
 */
class I2CMPU6050IMU : public InputDevice, public I2CDevice {
   private:
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);  //!< x,y,z acceleration [m/s^2] — gravity vector when stationary

   public:
    I2CMPU6050IMU(unsigned char _DeviceAddress, int _BusId = 1);

    void setDeviceAddress(unsigned char _DeviceAddress) override { DeviceAddress = _DeviceAddress; };
    void setBusId(int _BusId) override { BusId = _BusId; };

    /**
     * \brief Read the raw accelerometer vector (= gravity vector while the device is stationary).
     */
    void updateInput() override;

    /**
     * \brief Get the last-read raw acceleration vector [m/s^2], x,y,z.
     * While the device is stationary this is the gravity vector (magnitude ~9.81 m/s^2).
     */
    Eigen::VectorXd &getAcc() { return acc; };
};
#endif
