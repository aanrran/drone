#include <Arduino.h>
#include "controller_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "joysticks_console.h"
#include "dc_motor_driver.h"
#include "imu_mpu6050.h"

// Timer configuration
#define TIMER_INTERVAL_US 10000  // Timer interval in microseconds

static gptimer_handle_t timer = NULL;  // Timer handle

// Define the states
typedef enum {
    STATE_START,
    STATE_OPERATE,
    STATE_RESTART
} controller_state_t;

// Create and initialize the state
static controller_state_t current_state = STATE_START;

// Create an instance of the IMU_MPU6050 class
static IMU_MPU6050 imu6050(GPIO_NUM_48, GPIO_NUM_45);

struct PID {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain
    float integral;    // Integral term
    float prev_error;  // Previous error for derivative term
    float integral_max; // Maximum integral value to prevent windup
};

// Initialize PID parameters for roll, pitch, and yaw
static PID pid_roll = {1.0, 0.1, 0.05, 0, 0, 50};   // PID parameters for roll
static PID pid_pitch = {1.2, 0.1, 0.06, 0, 0, 50};  // PID parameters for pitch
static PID pid_yaw = {0.8, 0.1, 0.04, 0, 0, 50};    // PID parameters for yaw

/**
 * @brief check if joysticks are pushed
 * This function reads the static joystick data and check if they are not all zero.
 */
bool joystick_pushed() {
    // Read joystick data
    int8_t x1 = joystickData[0];
    int8_t y1 = joystickData[1];
    int8_t x2 = joystickData[2];
    int8_t y2 = joystickData[3];
    static int8_t counter = 0;
        // Check if the joystick data is not all zero. if not zero, print the reading
    if (x1 >= 1 || y1 >= 1 || x2 >= 1 || y2 >= 1||
        x1 <= -1 || y1 <= -1 || x2 <= -1 || y2 <= -1) {
            counter ++;
            if(counter > 2) { // check if the joysticks has moved for a while
                counter = 0;
                return  true;
            } else return false;
    }
    return false;
}

/**
 * @brief setter function to triger the restart of the state machine
 * This function reset the state machine status to restart
 */
void state_machine_restart() {
    current_state = STATE_RESTART;
}

/**
 * @brief Sets the PID parameters for roll, pitch, and yaw
 * 
 * @param Kp_roll Proportional gain for roll
 * @param Ki_roll Integral gain for roll
 * @param Kd_roll Derivative gain for roll
 * @param Kp_pitch Proportional gain for pitch
 * @param Ki_pitch Integral gain for pitch
 * @param Kd_pitch Derivative gain for pitch
 * @param Kp_yaw Proportional gain for yaw
 * @param Ki_yaw Integral gain for yaw
 * @param Kd_yaw Derivative gain for yaw
 * @param integral_max Maximum integral value to prevent windup
 */
void setPIDParameters(float Kp_roll, float Ki_roll, float Kd_roll,
                      float Kp_pitch, float Ki_pitch, float Kd_pitch,
                      float Kp_yaw, float Ki_yaw, float Kd_yaw, float integral_max) {
    pid_roll.Kp = Kp_roll;
    pid_roll.Ki = Ki_roll;
    pid_roll.Kd = Kd_roll;
    pid_roll.integral_max = integral_max;

    pid_pitch.Kp = Kp_pitch;
    pid_pitch.Ki = Ki_pitch;
    pid_pitch.Kd = Kd_pitch;
    pid_pitch.integral_max = integral_max;

    pid_yaw.Kp = Kp_yaw;
    pid_yaw.Ki = Ki_yaw;
    pid_yaw.Kd = Kd_yaw;
    pid_yaw.integral_max = integral_max;
    printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch, Kp_yaw, Ki_yaw, Kd_yaw, integral_max);
}

/**
 * @brief Computes the PID output
 * 
 * @param pid The PID structure containing parameters and state
 * @param error The current error
 * @param dt Delta time
 * @return The PID output, clamped between -100 and 100
 */
float computePID(PID &pid, float error, float dt) {
    // Update integral term with clamping to prevent windup
    pid.integral += error * dt;
    if (pid.integral > pid.integral_max) {
        pid.integral = pid.integral_max;
    } else if (pid.integral < -pid.integral_max) {
        pid.integral = -pid.integral_max;
    }
    
    // Compute derivative term
    float derivative = (error - pid.prev_error) / dt;
    
    // Update previous error
    pid.prev_error = error;
    
    // Compute PID output
    float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    
    // Clamp output to -100 to 100
    return fmax(fmin(output, 100), -100);
}

/**
 * @brief PID control function
 * The function is the main controller loop and updates motor duty cycle.
 */
void PID_control() {
    if(current_state != STATE_OPERATE) { // check to make sure drone is in operation.
        return;
    }
    // Read joystick data
    int8_t x1 = joystickData[0]; // + move right, - move left
    int8_t y1 = joystickData[1]; // from -0.5 to + 1.0 move up, at -0.5 motor zero speed, -1 to -0.5 keep previous y1 reading  
    int8_t x2 = joystickData[2]; // + turn right, - turn left
    int8_t y2 = joystickData[3]; // + move front, - move back

    // Scale joystick data to command values (pitch_cmd, roll_cmd, yaw_cmd)
    float roll_cmd = (float) x1 * 0.5f; // Scale to -10 to 10 degrees
    float pitch_cmd = (float) y2 * 0.5f; // Scale to -10 to 10 degrees
    float yaw_cmd = (float) x2 * 0.5f; // Scale to -10 to 10 degrees

    // Calculate delta time (dt)
    const float current_time = xthal_get_ccount(); // Get current time
    float dt = 0;
    static float prev_time = 0;  // Previous time for delta time calculation

    if (current_time > prev_time) {
        dt = (current_time - prev_time) / 240000000.0; // Convert to seconds
    } else {
        dt = 1.0 / 240000000.0; // Default dt value in case of error
    }
    prev_time = current_time; // Update previous time

    imu6050.mpu6050_updateAngles(dt); // Update and print roll, pitch, and yaw angles

    // Calculate errors
    float roll_err = roll_cmd - imu6050.roll;   // Roll error
    float pitch_err = pitch_cmd - imu6050.pitch; // Pitch error
    float yaw_err = yaw_cmd - imu6050.yaw;      // Yaw error

    // Compute PID outputs
    float roll_output = computePID(pid_roll, roll_err, dt);   // Roll PID output
    float pitch_output = computePID(pid_pitch, pitch_err, dt); // Pitch PID output
    float yaw_output = computePID(pid_yaw, yaw_err, dt);      // Yaw PID output

    // Base motor speed logic
    float base_speed = 0;

    if (y1 >= 0) {
        base_speed = (float) y1 * (100.0f - 30.0f) /20.0f+ 30.0f; // Scale from 0 to 1.0 as 30 to 100
    } else {
        base_speed = ((float) y1 + 20.0f) * 30.0f/20.0f; // Scale from -1.0 to 0 as 0 to 30
    }

    // Calculate motor duty cycles based on base speed and PID outputs
    float duty_cycle1 = base_speed - pitch_output + roll_output + yaw_output; // Motor 1
    float duty_cycle2 = base_speed - pitch_output - roll_output - yaw_output; // Motor 2
    float duty_cycle3 = base_speed + pitch_output - roll_output + yaw_output; // Motor 3
    float duty_cycle4 = base_speed + pitch_output + roll_output - yaw_output; // Motor 4

    // Set motor PWM duty cycles
    set_motor_pwm_duty(duty_cycle1, duty_cycle2, duty_cycle3, duty_cycle4);
}

/**
 * @brief Timer interrupt service routine
 * This function is called by the timer interrupt and triggers the PID control function.
 */
static bool IRAM_ATTR onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    // Notify the PID control task to run
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE); // Return whether we should yield at the end of ISR
}

/**
 * @brief PID control task function
 * This function runs on core 0 and handles the PID control.
 */
void PID_control_task(void *pvParameters) {
    Serial.println("PID control task started on core 2");
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for the notification from ISR
        PID_control();  // Call the PID control function
    }
}

/**
 * @brief Controller task function
 * This function runs on core 0 and continuously reads joystick data from the queue.
 * If new data is received, it updates the static joystick data array.
 * If no new data is received, it resets the joystick data to zero.
 */
void controller_task(void *pvParameters) {
    Serial.println("Controller task started on core 2");
    while (true) {
        joysticks_read();

        switch (current_state) {
            case STATE_START:
                // printf("State: START\n");
                // Perform actions for the START state  
                current_state = STATE_START;  // repeat if below events not happening
                set_motor_pwm_duty(0,0,0,0);

                if(joystick_pushed()) {
                    reboot_motor(); // start the motor and prepare for operation
                    current_state = STATE_OPERATE;
                }                      
                break;

            case STATE_OPERATE:
                // printf("State: OPERATE\n");
                current_state = STATE_OPERATE; // repeat if below events not happening
                // Perform actions for the OPERATE state

                break;
            
            case STATE_RESTART:
                // printf("State: RESTART\n");
                soft_stop_motor();
                current_state = STATE_START;
                break;

            default:
                printf("Unknown state!\n");
                break;
        }
        // Note, this wait period need to be less than the python joystick update rates. 
        // Otherwise, joysticks_read() doesn't get enough inputs in queue, and then, it will wait for an interally preset timeout period.
        vTaskDelay(pdMS_TO_TICKS(10)); 

    }
}

/**
 * @brief Function to start the controller task and configure the timer
 * This function creates a queue for joystick data, starts the controller task on core 2,
 * and configures a high-priority timer to trigger the PID control function.
 */
void startControllerTask() {

    // Initialize the joystick filters
    joysticks_init();    

    // Initialize the MPU6050 sensor
    imu6050.mpu6050_init();

    // Create the controller task pinned to core 0
    xTaskCreatePinnedToCore(controller_task, "ControllerTask", 4096, NULL, 1, NULL, 0);

    // Create the PID control task pinned to core 0
    TaskHandle_t PIDControlTaskHandle;
    xTaskCreatePinnedToCore(PID_control_task, "PIDControlTask", 4096, NULL, 23, &PIDControlTaskHandle, 0);

    // Configure the timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz, 1 tick = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    // Configure the alarm
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_INTERVAL_US,  // Set the alarm count for the interval
        .flags = {
            .auto_reload_on_alarm = true,  // Enable auto-reload
        }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

    // Register the timer callback and pass the PID control task handle
    gptimer_event_callbacks_t cbs = {
        .on_alarm = onTimer,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, (void*)PIDControlTaskHandle));

    // Enable and start the timer
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}
