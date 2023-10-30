#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Define the GPIO pins for motor control
#define MOTOR_ENA 0  // Enable motor A
#define MOTOR_IN1 2  // Input 1 for motor A
#define MOTOR_IN2 3  // Input 2 for motor A

#define MOTOR_ENB 1  // Enable motor B
#define MOTOR_IN3 5  // Input 1 for motor B
#define MOTOR_IN4 4  // Input 2 for motor B

// Function prototypes
void motor_setup();
void move_motor_a_forward(uint speed);
void move_motor_a_backward(uint speed);
void move_motor_b_forward(uint speed);
void move_motor_b_backward(uint speed);
void stop();

#endif  // MOTOR_H