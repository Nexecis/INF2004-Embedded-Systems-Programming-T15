#include <stdio.h>                                  // Standard I/O for debugging and output.
#include "pico/cyw43_arch.h"                        // CYW43 architecture for WiFi/Bluetooth functionality.
#include "pico/stdlib.h"                            // Standard library for Pico functionality.
#include "FreeRTOS.h"                               // FreeRTOS for real-time operating system capabilities.
#include "task.h"                                   // Task management for FreeRTOS.
#include "queue.h"                                  // Queue management for inter-task communication.
#include "hardware/gpio.h"                          // GPIO for general-purpose input/output functionality.
#include "hardware/adc.h"                           // ADC for analog-to-digital conversion (not used in this snippet).
#include "hardware/pwm.h"                           // PWM for pulse width modulation, used in motor control.

#define QUEUE_SIZE 10                               // Queue size for speed measurement queues.

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0                      // Default core for running FreeRTOS, configurable.
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL) // Priority level for the test task.

// Queue handles for inter-task communication regarding motor speed.
static QueueHandle_t xSpeedAQueue;
static QueueHandle_t xSpeedBQueue;

// Motor control GPIO pins definitions for clarity and maintainability.
#define MOTOR_ENA 0  // Enable pin for motor A.
#define MOTOR_IN1 2  // Input 1 for motor A direction control.
#define MOTOR_IN2 3  // Input 2 for motor A direction control.

#define MOTOR_ENB 1  // Enable pin for motor B.
#define MOTOR_IN3 5  // Input 1 for motor B direction control.
#define MOTOR_IN4 4  // Input 2 for motor B direction control.

// Physical constants and debounce delays for encoder processing.
#define PI 3.14159
#define WHEEL_DIAMETER 6.5
#define DEBOUNCE_DELAY_A_MS 3
#define DEBOUNCE_DELAY_B_MS 3

// Last event time for debouncing in interrupt handlers.
static uint32_t last_event_1_time = 0;
static uint32_t last_event_2_time = 0;

// Encoder data structure to encapsulate encoder-related information.
typedef struct {
    uint64_t start_time;          // Start time for speed calculation.
    float quarter_rotation;       // Distance for a quarter rotation of the wheel.
    float total_distance;         // Total distance covered by the wheel.
    int pinhole;                  // Pinhole count for encoder state.
    uint64_t milliseconds_per_cycle; // Time for one encoder cycle.
    float speed;                  // Calculated speed of the wheel.
} Encoder;

// Encoder instances for each wheel.
Encoder encoder1 = {0, 0.0, 0.0, 0, 0.0, 0, 0.0};
Encoder encoder2 = {0, 0.0, 0.0, 0, 0.0, 0, 0.0};

// PID controller structure for motor speed control.
typedef struct {
    float Kp; // Proportional gain.
    float Ki; // Integral gain.
    float Kd; // Derivative gain.

    float integral;             // Integral term for integral gain.
    float previous_error;       // Previous error for derivative calculation.
} PIDController;

PIDController pid;

// Initialize PID controller with specified gains.
void pid_init(float Kp, float Ki, float Kd) {
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;

    pid.integral = 0.0f;
    pid.previous_error = 0.0f;
}

// Update PID controller with new error value and time delta.
float pid_update(float error, float dt) {
    pid.integral += error * dt;
    float derivative = (error - pid.previous_error) / dt;
    pid.previous_error = error;

    // Calculate control output.
    return (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
}


// Callback function to handle GPIO interrupt events
void check_speed_callback(uint gpio, uint32_t events) {

    if (gpio == 26) {
        // Get the current time in milliseconds since boot
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Check if enough time has passed to debounce the signal
        if (current_time - last_event_1_time >= DEBOUNCE_DELAY_A_MS) {

            if (encoder1.pinhole == 0) {
                // Record the start time for the cycle
                encoder1.start_time = time_us_64();
                encoder1.pinhole++;
            } else if (encoder1.pinhole > 0 && encoder1.pinhole < 4) {
                // Increment the pinhole count
                encoder1.pinhole++;
            } else {
                // Calculate the total distance, milliseconds per cycle and speed
                encoder1.total_distance = encoder1.total_distance + encoder1.quarter_rotation;
                encoder1.milliseconds_per_cycle = (time_us_64() - encoder1.start_time) / 1000;
                encoder1.speed = ((encoder1.quarter_rotation / 100) * (1000 / encoder1.milliseconds_per_cycle));

                // Print the results
                printf("en1: %.5f m/s\n\n", encoder1.speed);

                // Reset the pinhole count
                encoder1.pinhole = 0;
            }

            // Update the last event time for debouncing
            last_event_1_time = current_time;
        }
        
        xQueueSend(xSpeedAQueue, &encoder1.speed, portMAX_DELAY);
    } else if (gpio == 27) {
        // Get the current time in milliseconds since boot
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Check if enough time has passed to debounce the signal
        if (current_time - last_event_2_time >= DEBOUNCE_DELAY_B_MS) {

            if (encoder2.pinhole == 0) {
                // Record the start time for the cycle
                encoder2.start_time = time_us_64();
                encoder2.pinhole++;
            } else if (encoder2.pinhole > 0 && encoder2.pinhole < 4) {
                // Increment the pinhole count
                encoder2.pinhole++;
            } else {
                // Calculate the total distance, milliseconds per cycle and speed
                encoder2.total_distance = encoder2.total_distance + encoder2.quarter_rotation;
                encoder2.milliseconds_per_cycle = (time_us_64() - encoder2.start_time) / 1000;
                encoder2.speed = ((encoder2.quarter_rotation / 100) * (1000 / encoder2.milliseconds_per_cycle));

                // Print the results
                printf("en2: %.5f m/s\n\n", encoder2.speed);

                // Reset the pinhole count
                encoder2.pinhole = 0;
            }


            // Update the last event time for debouncing
            last_event_2_time = current_time;
        }

        xQueueSend(xSpeedBQueue, &encoder2.speed, portMAX_DELAY);
    }
}

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

void check_speed_a_task(__unused void *params) {

    // Calculate the properties of the encoder movement
    encoder1.quarter_rotation = (PI * WHEEL_DIAMETER) / 4;

    // Set up GPIO interrupt on pin 2 with a rising edge trigger and callback function
    gpio_set_irq_enabled_with_callback(26, GPIO_IRQ_EDGE_RISE, true, &check_speed_callback);
    vTaskDelay(1000);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield control to scheduler
    }

}

void check_speed_b_task(__unused void *params) {

    // Calculate the properties of the encoder movement
    encoder2.quarter_rotation = (PI * WHEEL_DIAMETER) / 4;

    // Set up GPIO interrupt on pin 2 with a rising edge trigger and callback function
    gpio_set_irq_enabled_with_callback(27, GPIO_IRQ_EDGE_RISE, true, &check_speed_callback);
    vTaskDelay(1000);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield control to scheduler
    }

}

void move_forward_task(__unused void *params) {
    int base_speed = 100; // Base speed for both motors
    float speed_correction;
    float speedA, speedB;
    float error, dt = 0.1; // dt is the time difference in seconds

    while (true) {
        // Assume xQueueReceive gets the current speeds from the encoders
        xQueueReceive(xSpeedAQueue, &speedA, pdMS_TO_TICKS(1000));
        xQueueReceive(xSpeedBQueue, &speedB, pdMS_TO_TICKS(1000));

        error = speedA - speedB; // Calculate the difference in speeds
        speed_correction = pid_update(error, dt);

        move_motor_a_forward(base_speed + speed_correction);
        move_motor_b_forward(base_speed - speed_correction);

        vTaskDelay(pdMS_TO_TICKS(dt * 1000)); // Delay for dt seconds
    }
}

void vLaunch(void) {
    // Create queues.
    xSpeedAQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    xSpeedBQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));

    // Create tasks.
    xTaskCreate(move_forward_task, "ForwardThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(check_speed_a_task, "SpeedAThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(check_speed_b_task, "SpeedBThread", configMINIMAL_STACK_SIZE, NULL, 9, NULL);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    vTaskCoreAffinitySet(task, 1);                // Set core affinity for the main task.
#endif

    vTaskStartScheduler();                         // Start the FreeRTOS scheduler.
}

int main(void) {
    stdio_init_all();                              // Initialize standard I/O.
    motor_setup();
    pid_init(0.1f, 0.01f, 0.01f);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    gpio_init(28);
    gpio_set_dir(28, GPIO_OUT);
    gpio_put(28, 1);

    const char *rtos_name;
#if (portSUPPORT_SMP == 1)
    rtos_name = "FreeRTOS SMP";                   // Define the name for SMP-enabled FreeRTOS.
#else
    rtos_name = "FreeRTOS";                       // Define the name for regular FreeRTOS.
#endif

#if (portSUPPORT_SMP == 1) && (configNUM_CORES == 2)
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();                                     // Start FreeRTOS on both cores.
#elif (RUN_FREERTOS_ON_CORE == 1)
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch());             // Start FreeRTOS on core 1.
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();                                     // Start FreeRTOS on core 0.
#endif
    return 0;                                      // Return from the main function.
}