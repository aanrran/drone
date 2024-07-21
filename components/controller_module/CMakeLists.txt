idf_component_register(SRCS "controller_module.cpp"
                            "../IMU_MPU6050/IMU_MPU6050.cpp"
                    REQUIRES arduino
                    # REQUIRES I2Cbus
                    REQUIRES MPUdriver
                    # REQUIRES driver
                    INCLUDE_DIRS ../joysticks_console
                    INCLUDE_DIRS ../dc_motor_driver
                    INCLUDE_DIRS ../IMU_MPU6050
                    INCLUDE_DIRS "."
                    )
