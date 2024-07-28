#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

extern int8_t battery_level;  // int8_t to hold battery level

/**
 * @brief Initialize the ADC for battery monitoring.
 * 
 * This function configures the ADC width, attenuation, and characterizes the ADC.
 * 
 * Choosing a 12-bit resolution for the ADC because:
 * - The required resolution to detect a 0.1V change in battery voltage is approximately 18mV.
 *   Formula: 0.1V * (2.2kΩ / (10kΩ + 2.2kΩ)) ≈ 0.018V = 18mV.
 * - A 12-bit ADC with ADC_ATTEN_DB_11 (0 to 3.1V) provides a resolution of approximately 0.7578mV.
 *   Formula: 3100mV / 2^12 ≈ 0.7578mV.
 * - This provides more than sufficient resolution for our needs and ensures accurate readings.
 */
void init_adc();

/**
 * @brief Get the battery level as a uint8_t value.
 * 
 * This function reads the ADC value from the specified channel, converts it to
 * the corresponding battery voltage, and maps the voltage to a battery level.
 * 
 * The 3S LiPo battery voltage is calculated using the voltage divider formula:
 * Vlipo = Vpin2 * ((10kΩ + 2.2kΩ) / 2.2kΩ), where Vpin2 is the voltage at the ADC pin.
 * 
 * update uint8_t Battery level from 0 to 6, where 0 represents an empty battery
 * and 6 represents a full battery.
 */
void update_battery_level();

/**
 * @brief Print the ADC voltage.
 * 
 * This function reads the ADC value from the specified channel and prints
 * the voltage at pin 2 (Vpin2).
 */
void print_ADC_volt();

#endif // BATTERY_MONITOR_H
