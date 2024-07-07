#ifndef JOYSTICKS_CONSOLE_H
#define JOYSTICKS_CONSOLE_H

extern float joystickData[4];  // array to hold joystick data

// Initialize joystick filters.
void joysticks_init();

// read joystick and filter the data. Then, update the data to int
void joysticks_read();

// Function to process joystick data and send it to a queue storage
void processJoystickData(int x1, int y1, int x2, int y2);

#endif // JOYSTICKS_CONSOLE_H