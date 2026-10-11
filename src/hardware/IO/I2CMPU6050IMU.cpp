#include "I2CMPU6050IMU.h"


I2CMPU6050IMU::I2CMPU6050IMU(unsigned char _DeviceAddress, int _BusId) {
    setDeviceAddress(_DeviceAddress);
    setBusId(_BusId);
#ifndef NOROBOT
    initDevice();
    // MPU-6050 powers on in sleep mode (PWR_MGMT_1 bit 6 set) — must wake it before it will
    // update its data registers.
    writeReg(MPU6050_PWR_MGMT_1_ADDR, 0x00);
#endif
    spdlog::debug("Created I2C MPU6050 IMU on {}.", _DeviceAddress);
}

void I2CMPU6050IMU::updateInput() {
    // ACCEL_XOUT_H..ACCEL_ZOUT_H: 3 consecutive 16-bit signed registers (X,Y,Z).
    int16_t val[3];
    readBlockValues16RdwrS(MPU6050_ACCEL_XOUT_H_ADDR, 3, val);

    // Default full-scale range is +/-2g (ACCEL_CONFIG left at power-on default 0x00)
    // -> sensitivity 16384 LSB/g per the datasheet.
    const double lsbPerG = 16384.0;
    const double g       = 9.80665;
    acc[0] = (val[0] / lsbPerG) * g;
    acc[1] = (val[1] / lsbPerG) * g;
    acc[2] = (val[2] / lsbPerG) * g;
}
