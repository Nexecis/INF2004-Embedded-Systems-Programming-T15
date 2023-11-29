#include <math.h>              // Include math library for trigonometric functions.
#include "pico/stdlib.h"       // Standard library for Raspberry Pi Pico.
#include "hardware/i2c.h"      // I2C library for I2C communication.

#include "magnometer.h"        // Include the magnetometer header file.

#define MAGNOMETER_ADDRESS 0x1E // GY-511 sensor's I2C address.
#define SCALE 0.00091            // Scale factor for the magnetometer readings.

static i2c_inst_t *i2c_instance; // Global variable for I2C instance.

// Function to initialize I2C pins.
uint init_i2c_pin(i2c_inst_t *i2c, uint baudrate, uint sda_gpio, uint scl_gpio){
    i2c_instance = i2c;         // Set the I2C instance.

    gpio_set_function(sda_gpio, GPIO_FUNC_I2C); // Set SDA GPIO to I2C function.
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C); // Set SCL GPIO to I2C function.
    gpio_pull_up(sda_gpio);                     // Enable pull-up resistor on SDA.
    gpio_pull_up(scl_gpio);                     // Enable pull-up resistor on SCL.

    return i2c_init(i2c, baudrate);  // Initialize I2C with the specified baud rate.  
}

// Function to initialize the magnetometer.
int init_magnometer(){
    uint8_t config[] = {0x02, 0x00};  // Configuration array for the magnetometer.
    return i2c_write_blocking(i2c_instance, MAGNOMETER_ADDRESS, config, sizeof(config), false);
}

// Function to calculate the angle from the magnetometer readings.
double magnometer_angle(){
    uint8_t out_x = 0x03;               // Register address for X-axis output.
    uint8_t buffer[6];                  // Buffer to hold the magnetometer data.
    i2c_write_blocking(i2c_instance, MAGNOMETER_ADDRESS, &out_x, 1, true);
    i2c_read_blocking(i2c_instance, MAGNOMETER_ADDRESS, buffer, sizeof(buffer), false);

    int16_t x = ((buffer[0] << 8) | (buffer[1])); // Combine bytes for X-axis.
    // int16_t z = ((buffer[2] << 8) | (buffer[3])); // Z-axis data, unused in this code.
    int16_t y = ((buffer[4] << 8) | (buffer[5])); // Combine bytes for Y-axis.

    double x_scale = x * SCALE;          // Scale X-axis reading.
    double y_scale = y * SCALE;          // Scale Y-axis reading.
    double angle = atan2(y_scale, x_scale);    // Calculate angle using arctangent.
    double angle_degrees = angle * (180.0 / M_PI) + 180; // Convert radians to degrees.

    return angle_degrees;                // Return the angle in degrees.
}
