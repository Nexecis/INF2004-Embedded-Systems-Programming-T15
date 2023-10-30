/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define PI 3.142
#define WHEEL_RIM 6.4

// Define a structure to represent the stopwatch
typedef struct {
    bool running;
    uint64_t start_time;
    uint64_t end_time;
} Stopwatch;

static char event_str[128];
static Stopwatch sw;

float half_rotation = (PI * WHEEL_RIM) / 2;
float arch = 0;
int pinhole = 0;
float total_distance = 0.0;
uint64_t milliseconds_per_cycle = 0.0;
float speed = 0.0; 

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    // printf("GPIO %d %s\n", pinhole, event_str);
    arch = half_rotation / 10;

    total_distance = total_distance + arch;
    printf("%.2f cm\n", total_distance);

    if (pinhole == 0) {
        sw.start_time = time_us_64();
        pinhole++;
    } else if (pinhole > 0 && pinhole < 9) {
        pinhole++;
    } else {
        milliseconds_per_cycle = (time_us_64() - sw.start_time) / 1000;
        speed = ((100 / half_rotation) * milliseconds_per_cycle) / 1000;
        printf("%lld milliseconds\n", milliseconds_per_cycle);
        printf("%.4f m/s\n", speed);
        pinhole = 0;
    }
}

int main() {
    stdio_init_all();

    printf("Hello GPIO IRQ\n");
    // Edge-triggered interrupts
    //gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Level-triggered interrupts
    //gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);

    // Wait forever
    while (1);
}


static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}