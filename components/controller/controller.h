#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    MOVE_UP,
    MOVE_DOWN,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_FRONT,
    MOVE_BACK,
    TURN_RIGHT,
    TURN_LEFT
} Command;

extern QueueHandle_t command_queue;

void setup_pwm();
void control_task(void *pvParameters);
void set_motor_speeds(int speed1, int speed2, int speed3, int speed4);

#endif // CONTROLLER_H
