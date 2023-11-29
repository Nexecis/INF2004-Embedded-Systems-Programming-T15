#include <stdio.h>
#include "magnometer.h"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C i2c0 // Either i2c0 or i2c1, depending on pins 
#define BAUDRATE 400000 // Set baudrate at 400kHz

int main() {
    stdio_init_all();

    // Initialize I2C
    init_i2c_pin(I2C, BAUDRATE, I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Initialize magnometer to continuously read orientation
    init_magnometer();

    while(1){   
        double angle_degrees = magnometer_angle();
        printf("Angle in degrees: %lf\n", angle_degrees);
        sleep_ms(1000);
    }
    return 0;
}
