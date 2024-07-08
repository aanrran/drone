#include "imu_mpu6050.h"
#include "mpu/math.hpp"   // Math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "I2Cbus.hpp"
#include "esp_timer.h"

#define RAD_TO_DEG 57.2957795131  // Conversion factor from radians to degrees

/**
 * @brief Constructor to initialize with SDA and SCL pins.
 * @param sda_pin SDA pin for I2C.
 * @param scl_pin SCL pin for I2C.
 */
IMU_MPU6050::IMU_MPU6050(gpio_num_t sda_pin, gpio_num_t scl_pin)
    : sda_pin(sda_pin), scl_pin(scl_pin), roll(0), pitch(0), yaw(0), prev_time(0) {
}

/**
 * @brief Function to initialize the MPU6050 sensor.
 */
void IMU_MPU6050::mpu6050_init() {
    // Initialize the I2C bus
    i2c0.begin(sda_pin, scl_pin, 100000);

    // Set communication bus and address
    setBus(i2c0);
    setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

    // Test connection and initialize the chip with default configurations
    testConnection();
    initialize();

    // Configure sample rate, accelerometer and gyroscope full scale, and low pass filter
    setSampleRate(250);  // in Hz
    setAccelFullScale(mpud::ACCEL_FS_4G);
    setGyroFullScale(mpud::GYRO_FS_500DPS);
    setDigitalLowPassFilter(mpud::DLPF_42HZ);  // Smoother data

    // Fetch raw accelerometer data to calculate initial roll and pitch
    acceleration(&accelRaw);

    // Convert raw accelerometer data to g's
    accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);

    // Calculate initial roll and pitch from accelerometer data
#ifdef RESTRICT_PITCH
    roll = atan2(accelG.y, accelG.z) * RAD_TO_DEG;
    pitch = atan(-accelG.x / sqrt(accelG.y * accelG.y + accelG.z * accelG.z)) * RAD_TO_DEG;
#else
    roll = atan(accelG.y / sqrt(accelG.x * accelG.x + accelG.z * accelG.z)) * RAD_TO_DEG;
    pitch = atan2(-accelG.x, accelG.z) * RAD_TO_DEG;
#endif

    yaw = 0.0;

}

/**
 * @brief Function to print raw accelerometer and gyroscope readings.
 */
void IMU_MPU6050::mpu6050_printReadings() {
    // Fetch raw data from the registers
    acceleration(&accelRaw);
    rotation(&gyroRaw);

    // Print raw data
    printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
    printf("gyro: %+d %+d %+d\n", gyroRaw.x, gyroRaw.y, gyroRaw.z);

    // Convert raw data to meaningful units
    accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
    gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);

    // Print converted data
    printf("accel: %+.2f %+.2f %+.2f\n", accelG.x, accelG.y, accelG.z);
    printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);
}

/**
 * @brief Function to calculate and print roll, pitch, and yaw angles.
 */
void IMU_MPU6050::mpu6050_printAngles() {
    // Fetch raw data from the registers
    acceleration(&accelRaw);
    rotation(&gyroRaw);

    // Convert raw accelerometer data to g's
    accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);

    // Calculate delta time (dt)
    const float current_time = xthal_get_ccount();
    float dt = 0;
    if (current_time > prev_time) {
        dt = (current_time - prev_time) / 240000000.0; // Convert to seconds
    }
    prev_time = current_time;

    // Calculate gyro angles using delta time
    float gyroRoll  = roll + mpud::math::gyroDegPerSec(gyroRaw.x, mpud::GYRO_FS_500DPS) * dt;
    float gyroPitch = pitch + mpud::math::gyroDegPerSec(gyroRaw.y, mpud::GYRO_FS_500DPS) * dt;
    float gyroYaw   = yaw + mpud::math::gyroDegPerSec(gyroRaw.z, mpud::GYRO_FS_500DPS) * dt;

    // Calculate accelerometer angles
#ifdef RESTRICT_PITCH
    float accelRoll  = atan2(accelG.y, accelG.z) * RAD_TO_DEG;
    float accelPitch = atan(-accelG.x / sqrt(accelG.y * accelG.y + accelG.z * accelG.z)) * RAD_TO_DEG;
#else
    float accelRoll  = atan(accelG.y / sqrt(accelG.x * accelG.x + accelG.z * accelG.z)) * RAD_TO_DEG;
    float accelPitch = atan2(-accelG.x, accelG.z) * RAD_TO_DEG;
#endif

    // Fuse the angles using complementary filter
    roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
    pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
    yaw   = gyroYaw;

    // Correct yaw to stay within -180 to 180 degrees
    if (yaw > 180.f) yaw -= 360.f;
    else if (yaw < -180.f) yaw += 360.f;

    // Print the calculated angles
    printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f\n", pitch, roll, yaw);
}
