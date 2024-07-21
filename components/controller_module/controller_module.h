#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H


void setPIDParameters(float Kp_roll, float Ki_roll, float Kd_roll,
                      float Kp_pitch, float Ki_pitch, float Kd_pitch,
                      float Kp_yaw, float Ki_yaw, float Kd_yaw, float integral_max);

// Function to start the controller task and configure the timer
void startControllerTask();

#endif // CONTROLLER_MODULE_H
