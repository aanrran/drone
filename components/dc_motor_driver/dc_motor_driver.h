#ifndef DC_MOTOR_DRIVER_H
#define DC_MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_FREQUENCY 200  // PWM frequency in Hz

/**
 * @brief Initialize the DC motor driver using MCPWM.
 * 
 * This function sets up the MCPWM unit, configures the GPIO pins, and
 * sets the initial parameters for PWM operation, including frequency and duty cycle.
 */
void dc_motor_init();

/**
 * @brief Set the PWM duty cycle for the motors.
 * 
 * This function sets the duty cycle for each of the four motors. The duty cycle values
 * should be between 0 and 100, representing the percentage of the PWM period.
 * 
 * @param duty1 Duty cycle for motor 1 (0-100).
 * @param duty2 Duty cycle for motor 2 (0-100).
 * @param duty3 Duty cycle for motor 3 (0-100).
 * @param duty4 Duty cycle for motor 4 (0-100).
 */
void set_motor_pwm_duty(int duty1, int duty2, int duty3, int duty4);

#ifdef __cplusplus
}
#endif

#endif // DC_MOTOR_DRIVER_H
