# IMU MPU6050 Library

This library provides an interface for the MPU6050 IMU sensor using ESP32. It allows you to easily initialize the sensor, configure it, and read data from it.

## Prerequisites

The MPU driver depends on the following protocol libraries to communicate with the chip:
- **esp-dsp**: For math calculation
- **I2Cbus**: For I2C communication
- **MPUdriver**: provides MUP6050 basic functions 

### Install esp-dsp
**cd** to the project directory and run the following code: 

```bash
idf.py add-dependency "espressif/esp-dsp"
```
For more information, please see https://github.com/espressif/esp-dsp
### Install I2Cbus
go to the components directory and run this code:
```bash
git clone https://github.com/natanaeljr/esp32-I2Cbus.git I2Cbus
```

### Install MPUdriver
go to the components directory and run this code:
```bash
git clone https://github.com/natanaeljr/esp32-MPU-driver.git MPUdriver
```
for more information please see https://github.com/natanaeljr/esp32-MPU-driver?tab=readme-ov-file

### Configurate the Driver

run *idf.py menuconfig* , then select MPU9250 from menu *Component config -> MPU Driver*:

![Menuconfig](C:\Users\aanrr\drone\components\IMU_ICM20948\menuconfig_mpu-driver.png)

There are a few bugs needs to fix in file *MPU.cpp* from line 1871 to line 1894. Copy the below code in place of the original code:

```c++
        // slave 0 reads from magnetometer data register
        const auxi2c_slv_config_t kSlaveReadDataConfig = {
            .slave           = MAG_SLAVE_READ_DATA,
            .addr            = COMPASS_I2CADDRESS,
            .rw              = AUXI2C_READ,
            .reg_addr        = regs::mag::STATUS1,
            .reg_dis         = 0,
            .sample_delay_en = 1,
            .swap_en         = 0,
            .end_of_word     = (auxi2c_eow_t)0,
            .rxlength        = 8  //
        };
        if (MPU_ERR_CHECK(setAuxI2CSlaveConfig(kSlaveReadDataConfig))) return err;

        // slave 1 changes mode to single measurement
        const auxi2c_slv_config_t kSlaveChgModeConfig = {
            .slave           = MAG_SLAVE_CHG_MODE,
            .addr            = COMPASS_I2CADDRESS,
            .rw              = AUXI2C_WRITE,
            .reg_addr        = regs::mag::CONTROL1,
            .reg_dis         = 0,
            .sample_delay_en = 1,
            .txdata          = kControl1Value  //
        };
```



## Example

```C++
#include "imu_mpu6050.h"

extern "C" void app_main() {
    // Define the SDA and SCL pins
    gpio_num_t sda_pin = GPIO_NUM_21; // Change to your specific pin number
    gpio_num_t scl_pin = GPIO_NUM_22; // Change to your specific pin number

    // Create an instance of the IMU_MPU6050 class
    IMU_MPU6050 imu(sda_pin, scl_pin);

    // Initialize the MPU6050 sensor
    imu.mpu6050_init();

    // Continuously read and print sensor data
    while (true) {
        imu.mpu6050_printReadings();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second (1000 ms)
    }
}
```