#ifndef IMU_ICM20948_H
#define IMU_ICM20948_H

#include "MPU.hpp"        // main file, provides the class itself


class IMU_ICM20948 : public MPU_t {
public:
    IMU_ICM20948(gpio_num_t sda_pin, gpio_num_t scl_pin);
    void icm20948_init();
    void icm20948_printReadings();

private:
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    mpud::raw_axes_t accelRaw;     // holds x, y, z axes as int16
    mpud::raw_axes_t gyroRaw;      // holds x, y, z axes as int16
    mpud::raw_axes_t magRaw;       // holds x, y, z axes as int16
    mpud::float_axes_t accelG;
    mpud::float_axes_t gyroDPS;
};

#endif // IMU_ICM20948_H
