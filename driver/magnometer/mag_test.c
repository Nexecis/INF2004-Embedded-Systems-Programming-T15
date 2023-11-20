#include <stdio.h>
// #include <math.h>
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"

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



/*
# Replace 'picow_blink' with file name (without extension)
# Define the target name as a variable
set(target_name picow_blink)

add_executable(${target_name} ${target_name}.c)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(${target_name} 
        pico_magnometer
)

# allow the user to use serial monitor
pico_enable_stdio_usb(${target_name} 1)
pico_enable_stdio_uart(${target_name} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${target_name})

# add url via pico_set_program_url
example_auto_set_url(${target_name})
*/