//Get readings from ultrasonic sensor

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

int timeout = 26100;

void setupUltrasonicPins(uint trigPin, uint echoPin)//setting up the gpio pins for the trigger and echo pins of the ultrasonic 
{
    gpio_init(trigPin);//initialise gpio pin
    gpio_init(echoPin);//initialise gpio pin
    gpio_set_dir(trigPin, GPIO_OUT);//setting direction of gpio
    gpio_set_dir(echoPin, GPIO_IN);//setting direction of gpio
}

uint64_t getPulse(uint trigPin, uint echoPin)//this function gets the pulse length to be calculated later in the next function
{
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    while (gpio_get(echoPin) == 0) tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1) 
    {
        width++;
        sleep_us(1);
        if (width > timeout) return 0;
    }
    absolute_time_t endTime = get_absolute_time();
    
    return absolute_time_diff_us(startTime, endTime);
}

uint64_t getCm(uint trigPin, uint echoPin)//takes the pulse length and calculates the distance
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}
