#include "battery_monitor.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

/**
 * 
 * @brief Adjusted VREF Calculation:
 * - Because ADC_ATTEN_DB_12 has measurement range of 0~3100mV, the initial DEFAULT_VREF was 3100mV.
 * - After applying 3.3V on the ADC pin, The measured saturate voltage was 3.173V using print_ADC_volt() function.
 * - Therefore, the true VREF is 3173mV
 */
#define DEFAULT_VREF    3173    // Adjusted VREF based on calibration
#define ADC_CHANNEL     ADC1_CHANNEL_1 // GPIO2 is ADC1_CH1
#define ADC_WIDTH       ADC_WIDTH_BIT_12 // 12-bit ADC width for appropriate accuracy and efficiency
#define ADC_ATTEN       ADC_ATTEN_DB_12  // Adjust attenuation for input range

/**
 * @brief Predefined voltage divider ratio
 * 
 * The 3S LiPo battery voltage is calculated using the voltage divider formula:
 * Vlipo = Vpin2 * ((10kΩ + 2.2kΩ) / 2.2kΩ)
 * This ratio ((10kΩ + 2.2kΩ) / 2.2kΩ) is a constant and equals approximately 5.545.
 */
#define VOLTAGE_DIVIDER_RATIO ((10.0 + 2.2) / 2.2)
#define Vfull 12.0
#define Vempty 9.0

static const char *TAG = "BATTERY_MONITOR";
static esp_adc_cal_characteristics_t *adc_chars;

int8_t battery_level = 0;

/**
 * @brief Initialize the ADC for battery monitoring.
 * 
 * This function configures the ADC width, attenuation, and characterizes the ADC.
 * 
 * Choosing a 12-bit resolution for the ADC because:
 * - The required resolution to detect a 0.1V change in battery voltage is approximately 18mV.
 *   Formula: 0.1V * (2.2kΩ / (10kΩ + 2.2kΩ)) ≈ 0.018V = 18mV.
 * - A 12-bit ADC with ADC_ATTEN_DB_12 (0 to 3.1V) provides a resolution of approximately 0.7578mV.
 *   Formula: 3100mV / 2^12 ≈ 0.7578mV.
 * - This provides more than sufficient resolution for our needs and ensures accurate readings.
 */
void init_adc() {
    // Configure ADC width
    adc1_config_width(ADC_WIDTH);
    
    // Configure ADC channel attenuation
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    // Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars);
}

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
void update_battery_level() {
    // Read ADC value
    int raw = adc1_get_raw(ADC_CHANNEL);
    
    // Convert raw ADC value to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
    
    // Calculate the voltage at pin 2 (Vpin2) in volts
    float Vpin2 = voltage / 1000.0; // Convert mV to V
    
    // Calculate the actual battery voltage (Vlipo) using the voltage divider formula
    float Vlipo = Vpin2 * VOLTAGE_DIVIDER_RATIO;
    
    // Map battery voltage to battery level
    if (Vlipo >= Vfull) {
        // Maximum battery level
        battery_level = (uint8_t)((Vfull - 9.0) / 0.5);
    } else if (Vlipo <= Vempty) {
        // Minimum battery level
        battery_level = 0;
    } else {
        // Calculate battery level in 0.5V segments
        battery_level = (uint8_t)((Vlipo - 9.0) / 0.5);
    }
}

/**
 * @brief Print the ADC voltage.
 * 
 * This function reads the ADC value from the specified channel and prints
 * the voltage at pin 2 (Vpin2).
 */
void print_ADC_volt() {
    // Read ADC value
    int raw = adc1_get_raw(ADC_CHANNEL);
    
    // Convert raw ADC value to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
    
    // Calculate the voltage at pin 2 (Vpin2) in volts
    float Vpin2 = voltage / 1000.0; // Convert mV to V
    printf("ADC voltage: %.3fV\n", Vpin2);
}
