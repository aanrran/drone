#include <Arduino.h>
#include "joysticks_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_dsp.h"
#include <stdio.h>
#include <algorithm> // For std::min and std::max in C++

// Define the FIR filter order and coefficients
#define FIR_ORDER 3
std::array<float, FIR_ORDER + 1> fir_coefficients = {0.5, 0.5, 0.5, 0.5};

// Joysticks delay lines for each data stream
std::array<int8_t, FIR_ORDER + 1> delayLineX1 = {0};
std::array<int8_t, FIR_ORDER + 1> delayLineY1 = {0};
std::array<int8_t, FIR_ORDER + 1> delayLineX2 = {0};
std::array<int8_t, FIR_ORDER + 1> delayLineY2 = {0};

// Define a queue to communicate joystick data between tasks
QueueHandle_t joystickQueue;

int8_t joystickData[4] = {0, 0, 0, 0}; // Define and initialize the array

/**
 * @brief Initialize joystick filters.
 * 
 * This function initializes the filter coefficients and delay lines 
 * for the joystick filters using a first-order low-pass filter.
 */
void joysticks_init() {
    // Create a queue to hold joystick data
    joystickQueue = xQueueCreate(5, sizeof(int8_t[4]));
}

/**
 * @brief Clamp a value to the range [-1, 1].
 * 
 * @param value The value to clamp.
 * @return The clamped value.
 */
int8_t clamp(int8_t value) {
    return std::max(static_cast<int8_t>(-20), std::min(static_cast<int8_t>(20), value));
}

/**
 * @brief Apply FIR filter to a single data point.
 * 
 * @param input The current raw data point.
 * @param delayLine The delay line array.
 * @return The filtered data point.
 */
int8_t applyFIRFilter(int8_t input, std::array<int8_t, FIR_ORDER + 1>& delayLine) {
    // Shift delay line values
    for (int8_t i = FIR_ORDER; i > 0; i--) {
        delayLine[i] = delayLine[i - 1];
    }
    delayLine[0] = input;

    // Apply FIR filter
    int8_t output = 0.0;
    for (int8_t i = 0; i <= FIR_ORDER; i++) {
        output += static_cast<int8_t>(fir_coefficients[i] * delayLine[i]);
    }

    return output;
}

/**
 * @brief Filter joystick data using an FIR filter.
 * 
 * This function applies an FIR filter to the raw joystick data 
 * and stores the filtered data in the provided output array.
 * 
 * @param joystickRawData An array of 4 raw joystick data points.
 * @param filteredJoystickData An array to store the 4 filtered joystick data points.
 */
void filterJoystickData(int8_t joystickRawData[4], int8_t filteredJoystickData[4]) {
    // Apply FIR filter to each data point
    filteredJoystickData[0] = clamp(applyFIRFilter(joystickRawData[0], delayLineX1));
    filteredJoystickData[1] = clamp(applyFIRFilter(joystickRawData[1], delayLineY1));
    filteredJoystickData[2] = clamp(applyFIRFilter(joystickRawData[2], delayLineX2));
    filteredJoystickData[3] = clamp(applyFIRFilter(joystickRawData[3], delayLineY2));
}
/**
 * @brief read joystick and filter the data. Then, update the data to int
 * 
 * The function filter read and the joystick data. Then, convert to int
 */
void joysticks_read() {
        
    int8_t receivedData[4];  // Array to hold received joystick data
    int8_t filteredJoystickData[4];
    // Wait for joystick data from the queue with a timeout of 10 milliseconds
    if (xQueueReceive(joystickQueue, &receivedData, pdMS_TO_TICKS(100)) == pdPASS) {
        // Filter the joystick data
        filterJoystickData(receivedData, filteredJoystickData);
    } else {
        // Reset joystickData to zero if no new data received
        for (int i = 0; i < 4; i++) {
            receivedData[i] = 0;
        }
        // update the filter with zero readings
        filterJoystickData(receivedData, filteredJoystickData);

    }
    // Update the static joystickData array with new data
    for (int i = 0; i < 4; i++) {
        // joystickData[i] = static_cast<int>(filteredJoystickData[i] * 100);  // Convert to integer representation
        joystickData[i] = filteredJoystickData[i];
    }
    // Check if the joystick data is not all zero. if not zero, print the reading
    if (joystickData[0] >= 1 || joystickData[1] >= 1 || joystickData[2] >= 1 || joystickData[3] >= 1||
        joystickData[0] <= -1 || joystickData[1] <= -1 || joystickData[2] <= -1 || joystickData[3] <= -1
    ) {
        // printf("Joysticks fil Reading: x1: %d, y1: %d, x2: %d, y2: %d\n", joystickData[0], joystickData[1], joystickData[2], joystickData[3]);
    }
    printf("Joysticks Reading: x1: %d, y1: %d, x2: %d, y2: %d\n", joystickData[0], joystickData[1], joystickData[2], joystickData[3]);

}

/**
 * @brief Function to process joystick data
 * This function sends joystick data to the queue for the controller task to process.
 * @param x1 First joystick X-axis
 * @param y1 First joystick Y-axis
 * @param x2 Second joystick X-axis
 * @param y2 Second joystick Y-axis
 */
void processJoystickData(int8_t x1, int8_t y1, int8_t x2, int8_t y2) {
    // Send joystick data to the queue
    int8_t data[4] = {
        static_cast<int8_t>(x1 - 20),
        static_cast<int8_t>(y1 - 20),
        static_cast<int8_t>(x2 - 20),
        static_cast<int8_t>(y2 - 20)
    };
    xQueueSend(joystickQueue, &data, portMAX_DELAY);
}
