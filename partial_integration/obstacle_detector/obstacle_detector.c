// ultrasonic_freertos.c
#include <stdio.h>                                  // Standard I/O for debugging and output.
#include "pico/cyw43_arch.h"                        // CYW43 architecture for WiFi/Bluetooth functionality.
#include "pico/stdlib.h"                            // Standard library for Pico functionality.
#include "pico/time.h"                              // Time functions for timing and delays.
#include "hardware/gpio.h"                          // GPIO for general-purpose input/output functionality.
#include "hardware/adc.h"                           // ADC for analog-to-digital conversion (not used in this snippet).
#include "hardware/pwm.h"                           // PWM for pulse width modulation, used in motor control.
#include "hardware/timer.h"                         // Timer hardware for precise timing.
#include "FreeRTOS.h"                               // FreeRTOS for real-time operating system capabilities.
#include "task.h"                                   // Task management for FreeRTOS.
#include "queue.h"                                  // Queue management for inter-task communication.
#include "semphr.h"                                 // Semaphore management for task synchronization.

// Semaphore for synchronizing ultrasonic sensor readings.
SemaphoreHandle_t ultrasonicSemaphore;

// Function prototypes for better code organization.
void ultrasonicTask(void *pvParameters);
void setupUltrasonicPins(uint trigPin, uint echoPin);
uint64_t getPulse(uint trigPin, uint echoPin);
float getCm(uint trigPin, uint echoPin);  // Function to calculate distance in cm.

// Motor control GPIO pin definitions for maintainability and clarity.
#define MOTOR_ENA 0  // Enable pin for motor A.
#define MOTOR_IN1 2  // Input 1 for motor A direction control.
#define MOTOR_IN2 3  // Input 2 for motor A direction control.

#define MOTOR_ENB 1  // Enable pin for motor B.
#define MOTOR_IN3 5  // Input 1 for motor B direction control.
#define MOTOR_IN4 4  // Input 2 for motor B direction control.

// Constants for ultrasonic sensor operation.
#define TRIG_PIN 14   // GPIO pin for ultrasonic sensor trigger.
#define ECHO_PIN 15   // GPIO pin for ultrasonic sensor echo.
int timeout = 26100; // Timeout for pulse measurement to avoid infinite loop.


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

void move_motor_a_forward(uint speed) {
    // Set motor A direction for forward motion
    gpio_put(MOTOR_IN1, 1); // Set motor A input 1 to high
    gpio_put(MOTOR_IN2, 0); // Set motor A input 2 to low

    // Adjust the speed of motor A using PWM (e.g., speed 0-255)
    pwm_set_wrap(0, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(0, PWM_CHAN_A, speed);  // Set the motor A speed
}

void move_motor_a_backward(uint speed) {
    // Set motor A direction for backward motion
    gpio_put(MOTOR_IN1, 0); // Set motor A input 1 to low
    gpio_put(MOTOR_IN2, 1); // Set motor A input 2 to high

    // Adjust the speed of motor A using PWM (e.g., speed 0-255)
    pwm_set_wrap(0, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(0, PWM_CHAN_A, speed);  // Set the motor A speed
}

void move_motor_b_forward(uint speed) {
    // Set motor B direction for forward motion
    gpio_put(MOTOR_IN3, 1); // Set motor B input 1 to high
    gpio_put(MOTOR_IN4, 0); // Set motor B input 2 to low

    // Adjust the speed of motor B using PWM (e.g., speed 0-255)
    pwm_set_wrap(1, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(1, PWM_CHAN_A, speed);  // Set the motor B speed
}

void move_motor_b_backward(uint speed) {
    // Set motor B direction for backward motion
    gpio_put(MOTOR_IN3, 0); // Set motor B input 1 to low
    gpio_put(MOTOR_IN4, 1); // Set motor B input 2 to high
    // Adjust the speed of motor B using PWM (e.g., speed 0-255)
    pwm_set_wrap(1, 255);  // Set the PWM period for motor speed control
    pwm_set_chan_level(1, PWM_CHAN_A, speed);  // Set the motor B speed
}

void stop() {
    // Stop both motors
    gpio_put(MOTOR_IN1, 0); // Set motor A input 1 to low
    gpio_put(MOTOR_IN2, 0); // Set motor A input 2 to low
    pwm_set_chan_level(0, PWM_CHAN_A, 0);  // Set PWM duty cycle for motor A to 0

    gpio_put(MOTOR_IN3, 0); // Set motor B input 1 to low
    gpio_put(MOTOR_IN4, 0); // Set motor B input 2 to low
    pwm_set_chan_level(1, PWM_CHAN_A, 0);  // Set PWM duty cycle for motor B to 0
}

void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

uint64_t getPulse(uint trigPin, uint echoPin) {
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    while (gpio_get(echoPin) == 0) tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1) {
        width++;
        sleep_us(1);
        if (width > timeout) return 0;
    }
    absolute_time_t endTime = get_absolute_time();
    
    return absolute_time_diff_us(startTime, endTime);}

float getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return (float)pulseLength / 29.0 / 2.0;
}

void ultrasonicTask(void *pvParameters) {
    while (1) {
        float distance = getCm(TRIG_PIN, ECHO_PIN);
        printf("Distance: %.2f cm\n", distance);
        if(distance >30)
        {
            // Move motor A forward at a specified speed
            move_motor_a_forward(255);  // Adjust speed as needed
            // Move motor B forward at a specified speed
            move_motor_b_forward(255);  // Adjust speed as needed
        }
        else{
                stop();
                vTaskDelay(pdMS_TO_TICKS(1000));  // FreeRTOS delay function
                    // Move motor A backward at a specified speed
                move_motor_a_backward(200);  // Adjust speed as needed
                // Move motor B backward at a specified speed
                move_motor_b_backward(200);  // Adjust speed as needed
                vTaskDelay(pdMS_TO_TICKS(1000));  // FreeRTOS delay function

        }
        vTaskDelay(pdMS_TO_TICKS(500));  // FreeRTOS delay function
    }
}

int main() {
    stdio_init_all();
    motor_setup();
    gpio_init(13);
    gpio_set_dir(13, GPIO_OUT);
    gpio_put(13, 1);

    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

    ultrasonicSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(ultrasonicSemaphore);

    xTaskCreate(ultrasonicTask, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (1) {
        // Handle any error conditions here.
    }

    return 0;
}