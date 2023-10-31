#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define PI 3.14159
#define WHEEL_DIAMETER 6.5
#define DEBOUNCE_DELAY_MS 3

// Define a global variable to store the last event time for debouncing
static uint32_t last_event_time = 0;

// Define a structure to represent the encoder data
typedef struct {
    uint64_t start_time;
    float quarter_rotation;
    float arch;
    int pinhole;
    float total_distance;
    uint64_t milliseconds_per_cycle;
    float speed;
} Encoder;

// Initialize an instance of the Encoder structure
Encoder encoder = {0, 0.0, 0.0, 0, 0.0, 0, 0.0};

// Function to handle GPIO interrupt events
void gpio_callback(uint gpio, uint32_t events);

int main() {
    stdio_init_all();

    // Calculate the properties of the encoder movement
    encoder.quarter_rotation = (PI * WHEEL_DIAMETER) / 4;

    sleep_ms(3000);

    printf("Hello ENCODER!\n\n");

    // Set up GPIO interrupt on pin 2 with a rising edge trigger and callback function
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Wait forever
    while (1);
}

// Callback function to handle GPIO interrupt events
void gpio_callback(uint gpio, uint32_t events) {

    // Get the current time in milliseconds since boot
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Check if enough time has passed to debounce the signal
    if (current_time - last_event_time >= DEBOUNCE_DELAY_MS) {

        if (encoder.pinhole == 0) {
            // Record the start time for the cycle
            encoder.start_time = time_us_64();
            encoder.pinhole++;
        } else if (encoder.pinhole > 0 && encoder.pinhole < 4) {
            // Increment the pinhole count
            encoder.pinhole++;
        } else {
            // Calculate the total distance, milliseconds per cycle and speed
            encoder.total_distance = encoder.total_distance + encoder.quarter_rotation;
            encoder.milliseconds_per_cycle = (time_us_64() - encoder.start_time) / 1000;
            encoder.speed = ((encoder.quarter_rotation / 100) * (1000 / encoder.milliseconds_per_cycle));

            // Print the results
            printf("%.2f cm\n", encoder.total_distance);
            printf("%lld milliseconds\n", encoder.milliseconds_per_cycle);
            printf("%.5f m/s\n\n", encoder.speed);

            // Reset the pinhole count
            encoder.pinhole = 0;
        }

        // Update the last event time for debouncing
        last_event_time = current_time;
    }
}
