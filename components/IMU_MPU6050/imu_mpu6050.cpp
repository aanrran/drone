#include "imu_mpu6050.h"
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "I2Cbus.hpp"

IMU_MPU6050::IMU_MPU6050(gpio_num_t sda_pin, gpio_num_t scl_pin)
    : sda_pin(sda_pin), scl_pin(scl_pin) {
}

void IMU_MPU6050::mpu6050_init() {
    // I2C_t& i2c0 = I2C0;
    i2c0.begin(sda_pin, scl_pin, 10000);  // initialize the I2C bus
             
    setBus(i2c0);  // set communication bus, for SPI -> pass 'hspi'
    setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address or handle, for SPI -> pass 'mpu_spi_handle'
    testConnection(); // test connection with the chip, return is a error code
    initialize();  // this will initialize the chip and set default configurations

    setSampleRate(250);  // in (Hz)
    setAccelFullScale(mpud::ACCEL_FS_4G);
    setGyroFullScale(mpud::GYRO_FS_500DPS);
    setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
    // setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);  // enable INT pin
}

void IMU_MPU6050::mpu6050_printReadings() {
    acceleration(&accelRaw);  // fetch raw data from the registers
    rotation(&gyroRaw);       // fetch raw data from the registers
    printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
    printf("gyro: %+d %+d %+d\n", gyroRaw.x, gyroRaw.y, gyroRaw.z);

    accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);  // raw data to gravity
    gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);  // raw data to º/s
    printf("accel: %+.2f %+.2f %+.2f\n", accelG.x, accelG.y, accelG.z);
    printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);
}
