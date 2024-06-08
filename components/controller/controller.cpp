#include "controller.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"

#define MOTOR1_PWM_PIN  14
#define MOTOR2_PWM_PIN  12
#define MOTOR3_PWM_PIN  13
#define MOTOR4_PWM_PIN  15

QueueHandle_t command_queue;

void setup_pwm() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1_PWM_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR2_PWM_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, MOTOR3_PWM_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, MOTOR4_PWM_PIN);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 5000; // Frequency in Hz
    pwm_config.cmpr_a = 0;       // Duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       // Duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

void set_motor_speeds(int speed1, int speed2, int speed3, int speed4) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed2);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, speed3);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, speed4);
}

void control_task(void *pvParameters) {
    command_queue = xQueueCreate(10, sizeof(Command));
    Command cmd;
    while (true) {
        if (xQueueReceive(command_queue, &cmd, portMAX_DELAY)) {
            switch (cmd) {
                case MOVE_UP:
                    set_motor_speeds(70, 70, 70, 70);
                    break;
                // Handle other commands similarly...
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust as needed
    }
}
