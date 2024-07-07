#include "dc_motor_driver.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <stdio.h>

// Define GPIO pins for motors
#define GPIO_PWM0A_OUT 21   // Motor 1
#define GPIO_PWM0B_OUT 41   // Motor 2
#define GPIO_PWM1A_OUT 45   // Motor 3
#define GPIO_PWM1B_OUT 47   // Motor 4

void mcpwm_example_gpio_initialize(void) {
    printf("Initializing MCPWM GPIO...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
}

void dc_motor_init() {
    // 1. MCPWM GPIO initialization
    mcpwm_example_gpio_initialize();

    // 2. Initialize MCPWM configuration
    printf("Configuring Initial Parameters of MCPWM...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;    // Set frequency
    pwm_config.cmpr_a = 0;                   // Initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                   // Initial duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Initialize PWM for each motor
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

void set_motor_pwm_duty(float duty1, float duty2, float duty3, float duty4) {
    // Constrain duty cycle values to be within 0 to 100
    if (duty1 < 0) duty1 = 0;
    if (duty1 > 100) duty1 = 100;
    if (duty2 < 0) duty2 = 0;
    if (duty2 > 100) duty2 = 100;
    if (duty3 < 0) duty3 = 0;
    if (duty3 > 100) duty3 = 100;
    if (duty4 < 0) duty4 = 0;
    if (duty4 > 100) duty4 = 100;

    // Set duty cycle for each motor
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty2);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty3);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty4);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}
