#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "MPU.hpp"  // Main file, provides the class itself
#include <math.h>
#include "esp_dsp.h"

// Define RESTRICT_PITCH as default
#define RESTRICT_PITCH


/**
 * @brief IMU_MPU6050 class for handling MPU6050 sensor.
 */
class IMU_MPU6050 : public MPU_t {
public:
    /**
     * @brief Constructor to initialize with SDA and SCL pins.
     * @param sda_pin SDA pin for I2C.
     * @param scl_pin SCL pin for I2C.
     */
    IMU_MPU6050(gpio_num_t sda_pin, gpio_num_t scl_pin);

    /**
     * @brief Function to initialize the MPU6050 sensor.
     */
    void mpu6050_init();

    /**
     * @brief Function to print raw accelerometer and gyroscope readings.
     */
    void mpu6050_printReadings();

    /**
     * @brief Function to calculate roll, pitch, and yaw angles.
     */
    void mpu6050_updateAngles(float dt);
    /**
     * @brief Function to print roll, pitch, and yaw angles.
     */
    void mpu6050_printAngles();

private:
    gpio_num_t sda_pin;  ///< SDA pin for I2C
    gpio_num_t scl_pin;  ///< SCL pin for I2C

    mpud::raw_axes_t accelRaw;  ///< Raw data structure for accelerometer
    mpud::raw_axes_t gyroRaw;   ///< Raw data structure for gyroscope

    mpud::float_axes_t accelG;  ///< Converted data structure for accelerometer in g
    mpud::float_axes_t gyroDPS; ///< Converted data structure for gyroscope in degrees per second

    float roll;   ///< Roll angle
    float pitch;  ///< Pitch angle
    float yaw;    ///< Yaw angle

    float imu_roll;
    float imu_pitch;
    float imu_yaw;

    float prev_time;  ///< Previous time for delta time calculation
};

#endif // IMU_MPU6050_H
