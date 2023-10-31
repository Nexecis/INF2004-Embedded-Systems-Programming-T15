#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C i2c0 // Either i2c0 or i2c1, depending on pins 
#define MAGNOMETER_ADDRESS 0x1E // GY-511 sensor's I2C address
#define BAUDRATE 400000 // Set baudrate at 400kHz

int main() {
    stdio_init_all();

    // Initialize I2C
    i2c_init(I2C, BAUDRATE);  
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize magnometer to continuously read orientation
    uint8_t config[] = {0x02, 0x00};  
    i2c_write_blocking(I2C, MAGNOMETER_ADDRESS, config, sizeof(config), false);

    while(1){
        uint8_t out_x = 0x03;
        uint8_t buffer[6];
        i2c_write_blocking(I2C, MAGNOMETER_ADDRESS, &out_x, 1, true);
        i2c_read_blocking(I2C, MAGNOMETER_ADDRESS, buffer, sizeof(buffer), false);

        // Parse the data (little-endian format)
        int16_t x = ((buffer[0] << 8) | (buffer[1]));
        int16_t z = ((buffer[2] << 8) | (buffer[3]));
        int16_t y = ((buffer[4] << 8) | (buffer[5]));
                
        double scale = 0.00091;
        double x_scale = x * scale;
        double y_scale = y * scale;

        double angle = atan2(y_scale, x_scale);    
        double angle_degrees = angle * (180.0 / M_PI) + 180;

        printf("Angle in degrees: %lf\n", angle_degrees);

        sleep_ms(1000);
    }

    return 0;
}


// CMakeLists.txt

// # Replace 'picow_blink' with file name (without extension)
// # Define the target name as a variable
// set(target_name picow_blink)

// add_executable(${target_name} ${target_name}.c)

// # pull in common dependencies and additional pwm hardware support
// target_link_libraries(${target_name} 
//         pico_stdlib 
//         pico_cyw43_arch_none
//         hardware_i2c
// )

// # allow the user to use serial monitor
// pico_enable_stdio_usb(${target_name} 1)
// pico_enable_stdio_uart(${target_name} 1)

// # create map/bin/hex file etc.
// pico_add_extra_outputs(${target_name})

// # add url via pico_set_program_url
// example_auto_set_url(${target_name})
