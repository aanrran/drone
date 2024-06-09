#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H

// Function to start the controller task and configure the timer
void startControllerTask();

// Function to process joystick data and send it to the controller task
void processJoystickData(float x1, float y1, float x2, float y2);

#endif // CONTROLLER_MODULE_H
