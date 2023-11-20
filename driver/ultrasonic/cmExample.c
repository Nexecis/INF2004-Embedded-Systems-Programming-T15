// ultrasonic.c
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

int timeout = 26100;

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
    
    return absolute_time_diff_us(startTime, endTime);
}

float getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return (float)pulseLength / 29.0 / 2.0;
}

int main() {
    stdio_init_all();
    
    uint trigPin = 2;  // Adjust pin number based on your configuration
    uint echoPin = 3;  // Adjust pin number based on your configuration

    setupUltrasonicPins(trigPin, echoPin);

    while (1) {
        float distance = getCm(trigPin, echoPin);
        printf("Distance: %.2f cm\n", distance);
        sleep_ms(500);
    }

    return 0;
}