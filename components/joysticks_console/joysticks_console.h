#ifndef JOYSTICKS_CONSOLE_H
#define JOYSTICKS_CONSOLE_H

extern int8_t joystickData[4];  // array to hold joystick data

// Initialize joystick filters.
void joysticks_init();

// read joystick and filter the data. Then, update the data to int
void joysticks_read();

// Function to process joystick data and send it to a queue storage
void processJoystickData(int8_t x1, int8_t y1, int8_t x2, int8_t y2);

#endif // JOYSTICKS_CONSOLE_H