#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "MPU.hpp"        // main file, provides the class itself


class IMU_MPU6050 : public MPU_t {
public:
    IMU_MPU6050(gpio_num_t sda_pin, gpio_num_t scl_pin);
    void mpu6050_init();
    void mpu6050_printReadings();

private:
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    mpud::raw_axes_t accelRaw;     // holds x, y, z axes as int16
    mpud::raw_axes_t gyroRaw;      // holds x, y, z axes as int16
    mpud::float_axes_t accelG;
    mpud::float_axes_t gyroDPS;
};

#endif // IMU_MPU6050_H
