// ultrasonic.c
#include "pico/stdlib.h"  // Standard library for Raspberry Pi Pico.
#include <stdio.h>        // Standard I/O for printf.
#include "hardware/gpio.h"// GPIO for general-purpose input/output functionality.
#include "hardware/timer.h"// Timer for timing measurements.

int timeout = 26100;      // Timeout in microseconds for ultrasonic echo.

// Setup GPIO pins for the ultrasonic sensor.
void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);      // Initialize GPIO for trigger pin.
    gpio_init(echoPin);      // Initialize GPIO for echo pin.
    gpio_set_dir(trigPin, GPIO_OUT); // Set trigger pin as output.
    gpio_set_dir(echoPin, GPIO_IN);  // Set echo pin as input.
}

// Measure the pulse width of the ultrasonic echo.
uint64_t getPulse(uint trigPin, uint echoPin) {
    gpio_put(trigPin, 1);    // Send a pulse.
    sleep_us(10);            // Pulse width of 10 microseconds.
    gpio_put(trigPin, 0);    // Stop the pulse.

    uint64_t width = 0;      // Variable to measure pulse width.

    // Wait for the echo to start.
    while (gpio_get(echoPin) == 0) tight_loop_contents();
    absolute_time_t startTime = get_absolute_time(); // Record start time of echo.

    // Measure the length of the echo signal.
    while (gpio_get(echoPin) == 1) {
        width++;
        sleep_us(1);        // Increment width for each microsecond.
        if (width > timeout) return 0; // Return 0 if timeout exceeded.
    }
    absolute_time_t endTime = get_absolute_time(); // Record end time of echo.
    
    return absolute_time_diff_us(startTime, endTime); // Calculate pulse width.
}

// Calculate distance in centimeters based on ultrasonic pulse width.
float getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin); // Get pulse width.
    return (float)pulseLength / 29.0 / 2.0; // Convert pulse width to distance.
}

int main() {
    stdio_init_all();        // Initialize standard I/O.
    
    uint trigPin = 2;        // GPIO pin for ultrasonic trigger.
    uint echoPin = 3;        // GPIO pin for ultrasonic echo.

    setupUltrasonicPins(trigPin, echoPin); // Setup GPIO pins for ultrasonic sensor.

    while (1) {
        float distance = getCm(trigPin, echoPin); // Measure distance.
        printf("Distance: %.2f cm\n", distance);  // Print distance.
        sleep_ms(500);                           // Wait for 500 milliseconds.
    }

    return 0;
}
