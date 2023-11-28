#include <stdio.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "magnometer.h"
#include "hardware/gpio.h"

#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7
#define I2C i2c1 // Either i2c0 or i2c1, depending on pins 
#define BAUDRATE 400000 // Set baudrate at 400kHz

// Define the GPIO pins for motor control
#define MOTOR_ENA 0  // Enable motor A
#define MOTOR_IN1 2  // Input 1 for motor A
#define MOTOR_IN2 3  // Input 2 for motor A

#define MOTOR_ENB 1  // Enable motor B
#define MOTOR_IN3 5  // Input 1 for motor B
#define MOTOR_IN4 4  // Input 2 for motor B


void motor_setup() {
    // Configure GPIO pins for motor control as outputs
    gpio_init(MOTOR_ENA); // Initialize the motor A enable pin
    gpio_init(MOTOR_IN1); // Initialize motor A input 1
    gpio_init(MOTOR_IN2); // Initialize motor A input 2

    gpio_init(MOTOR_ENB); // Initialize the motor B enable pin
    gpio_init(MOTOR_IN3); // Initialize motor B input 1
    gpio_init(MOTOR_IN4); // Initialize motor B input 2

    gpio_set_dir(MOTOR_ENA, GPIO_OUT); // Set motor A enable pin as an output
    gpio_set_dir(MOTOR_IN1, GPIO_OUT); // Set motor A input 1 as an output
    gpio_set_dir(MOTOR_IN2, GPIO_OUT); // Set motor A input 2 as an output

    gpio_set_dir(MOTOR_ENB, GPIO_OUT); // Set motor B enable pin as an output
    gpio_set_dir(MOTOR_IN3, GPIO_OUT); // Set motor B input 1 as an output
    gpio_set_dir(MOTOR_IN4, GPIO_OUT); // Set motor B input 2 as an output

    // Initialize motor control (set ENA and ENB HIGH for full speed)
    gpio_put(MOTOR_ENA, 1); // Enable motor A
    gpio_put(MOTOR_IN1, 0); // Set motor A input 1 to low
    gpio_put(MOTOR_IN2, 0); // Set motor A input 2 to low

    gpio_put(MOTOR_ENB, 1); // Enable motor B
    gpio_put(MOTOR_IN3, 0); // Set motor B input 1 to low
    gpio_put(MOTOR_IN4, 0); // Set motor B input 2 to low
}

void motorA(signed short speed)
{
    if(speed > 1000) speed = 1000;
    if(speed < -1000) speed = -1000;

    if(speed > 0){
        // Set motor A direction for forward motion
        gpio_put(MOTOR_IN1, 1);
        gpio_put(MOTOR_IN2, 0);
    }
    else if(speed < 0){
        // Set motor A direction for backward motion
        gpio_put(MOTOR_IN1, 0);
        gpio_put(MOTOR_IN2, 1);
    }
    else{
        // Set motor A to stop
        gpio_put(MOTOR_IN1, 0);
        gpio_put(MOTOR_IN2, 0);
    }

    pwm_set_wrap(0, 1000); // Set the PWM period for motor speed control
    pwm_set_chan_level(0, PWM_CHAN_A, abs(speed)); // Set the motor A speed
}

void motorB(signed short speed)
{
    if(speed > 1000) speed = 1000;
    if(speed < -1000) speed = -1000;

    if(speed > 0){
        // Set motor B direction for forward motion
        gpio_put(MOTOR_IN3, 1);
        gpio_put(MOTOR_IN4, 0);
    }
    else if(speed < 0){
        // Set motor B direction for backward motion
        gpio_put(MOTOR_IN3, 0);
        gpio_put(MOTOR_IN4, 1);
    }
    else{
        // Set motor B to stop
        gpio_put(MOTOR_IN3, 0);
        gpio_put(MOTOR_IN4, 0);
    }

    pwm_set_wrap(1, 1000); // Set the PWM period for motor speed control
    pwm_set_chan_level(1, PWM_CHAN_A, abs(speed)); // Set the motor B speed
}

void leftTurn(){
    printf("TURING LEFT\n");
    
    // move forward
    motorA(200);
    motorB(200);
    sleep_ms(600);

    // stop
    motorA(0);
    motorB(0);
    sleep_ms(500);

    double start_angle = magnometer_angle();
    double desired_angle = start_angle - 90;
    
    // turn left
    motorA(-100);
    motorB(100);
    
    // check if it turn 90 degrees
    sleep_ms(50);
    if(desired_angle < 0){
        desired_angle += 360;
        while(magnometer_angle() < start_angle || magnometer_angle() > desired_angle);
    }
    else{
        while(magnometer_angle() < start_angle && magnometer_angle() > desired_angle);
    }

    // stop
    motorA(0);
    motorB(0);
    sleep_ms(500);

    // move forward
    motorA(200);
    motorB(200);
    sleep_ms(500);

    // stop
    motorA(0);
    motorB(0);
    printf("DONE TURING LEFT\n");
}

void rightTurn(){
    printf("TURING RIGHT\n");

    // move forward
    motorA(200);
    motorB(200);
    sleep_ms(600);

    // stop
    motorA(0);
    motorB(0);
    sleep_ms(500);

    double start_angle = magnometer_angle();
    double desired_angle = start_angle + 90;

    // turn right
    motorA(100);
    motorB(-100);

    // check if it turn 90 degrees
    sleep_ms(50);
    if(desired_angle > 360){
        desired_angle -= 360;
        while(magnometer_angle() > start_angle || magnometer_angle() < desired_angle);
    }
    else{
        while(magnometer_angle() > start_angle && magnometer_angle() < desired_angle);
    }

    // stop
    motorA(0);
    motorB(0);
    sleep_ms(500);

    // move forward
    motorA(200);
    motorB(200);
    sleep_ms(500);

    // stop
    motorA(0);
    motorB(0);
    printf("DONE TURING RIGHT\n");
}

int main() {
    stdio_init_all();
    sleep_ms(5000);

    gpio_init(8);
    gpio_set_dir(8, GPIO_OUT);
    gpio_put(8, 1);
    
    motor_setup();
    init_i2c_pin(I2C, BAUDRATE, I2C_SDA_PIN, I2C_SCL_PIN);
    init_magnometer();

    //uncomment and comment to test turning left or right (vice-versa)
    //leftTurn();
    rightTurn();

    return 0;
}Z