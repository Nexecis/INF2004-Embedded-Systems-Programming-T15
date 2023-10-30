#include "pico/stdlib.h"
#include "hardware/pwm.h"

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

int main() {
    stdio_init_all();
    motor_setup();

    // Move motor A forward at a specified speed
    move_motor_a_forward(200);  // Adjust speed as needed

    // Move motor B forward at a specified speed
    move_motor_b_forward(180);  // Adjust speed as needed

    sleep_ms(500);  // Keep the motors running forward for 0.5 second

    // Stop both motors
    stop();

    sleep_ms(1000);  // Pause for 1 second

    // Turn the robot left at a specified speed
    move_motor_a_backward(180);  // Adjust speed as needed
    move_motor_b_forward(180);  // Adjust speed as needed

    sleep_ms(500);  // Keep the robot turning left for 0.5 second

    // Stop both motors
    stop();

    sleep_ms(1000);  // Pause for 1 second

    // Move motor A forward at a specified speed
    move_motor_a_forward(200);  // Adjust speed as needed

    // Move motor B forward at a specified speed
    move_motor_b_forward(180);  // Adjust speed as needed

    sleep_ms(500);  // Keep the motors running forward for 0.5 second

    // Stop both motors
    stop();

    sleep_ms(1000);  // Pause for 1 second

    // Turn the robot right at a specified speed
    move_motor_a_forward(180);  // Adjust speed as needed
    move_motor_b_backward(180);  // Adjust speed as needed

    sleep_ms(500);  // Keep the robot turning right for 0.5 second

    // Stop both motors
    stop();

   sleep_ms(1000);  // Pause for 1 second

    // Move motor A backward at a specified speed
    move_motor_a_backward(150);  // Adjust speed as needed

    // Move motor B backward at a specified speed
    move_motor_b_backward(150);  // Adjust speed as needed

    sleep_ms(500);  // Keep the motors running backward for 0.5 second

    // Stop both motors
    stop();

    return 0;
}