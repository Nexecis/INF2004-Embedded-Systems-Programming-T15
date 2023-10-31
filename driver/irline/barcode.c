#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <string.h>

#define IR_SENSOR_PIN 15  // Pin for the IR sensor
#define THIN_WIDTH_THRESHOLD 50  // Threshold for detecting thin bars (in microseconds)
#define WIDE_WIDTH_THRESHOLD 100  // Threshold for detecting wide bars (in microseconds)

// Function to initialize the IR sensor
void initIRSensor() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
}

// Function to measure the pulse width
uint32_t measurePulseWidth() {
    uint32_t start_time, end_time;

    // Wait for the falling edge of the pulse (transition from black to white)
    while (gpio_get(IR_SENSOR_PIN) == 1) {}

    start_time = time_us_32();

    // Wait for the rising edge of the pulse (transition from white to black)
    while (gpio_get(IR_SENSOR_PIN) == 0) {}

    end_time = time_us_32();

    return end_time - start_time;
}

int main() {
    stdio_init_all();
    initIRSensor();

    while (1) {
        // Measure the pulse width using the IR sensor
        uint32_t pulse_width = measurePulseWidth();

        // Determine the bar type based on the pulse width
        char bar_type[4];
        if (pulse_width < THIN_WIDTH_THRESHOLD) {
            strcpy(bar_type, "1");  // Thin black bar
        } else if (pulse_width < WIDE_WIDTH_THRESHOLD) {
            strcpy(bar_type, "111");  // Thick black bar
        } else if (pulse_width < 2 * THIN_WIDTH_THRESHOLD) {
            strcpy(bar_type, "0");  // Thin white bar
        } else if (pulse_width < 2 * WIDE_WIDTH_THRESHOLD) {
            strcpy(bar_type, "000");  // Thick white bar
        } else {
            // The pulse width does not match any expected bar width
            continue;
        }
        if (pulse_width > 1) {
                printf("Detected bar type: %s with pulse width: %u us\n", bar_type, pulse_width);
        }
    }

    return 0;
}
