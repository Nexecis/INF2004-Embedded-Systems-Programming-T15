#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "magnometer.h"

#define MAGNOMETER_ADDRESS 0x1E // GY-511 sensor's I2C address
#define SCALE 0.00091

static i2c_inst_t *i2c_instance;

uint init_i2c_pin(i2c_inst_t *i2c, uint baudrate, uint sda_gpio, uint scl_gpio){
    i2c_instance = i2c;

    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);

    return i2c_init(i2c, baudrate);  
}

int init_magnometer(){
    uint8_t config[] = {0x02, 0x00};  
    return i2c_write_blocking(i2c_instance, MAGNOMETER_ADDRESS, config, sizeof(config), false);
}

double magnometer_angle(){
    uint8_t out_x = 0x03;
    uint8_t buffer[6];
    i2c_write_blocking(i2c_instance, MAGNOMETER_ADDRESS, &out_x, 1, true);
    i2c_read_blocking(i2c_instance, MAGNOMETER_ADDRESS, buffer, sizeof(buffer), false);

    int16_t x = ((buffer[0] << 8) | (buffer[1]));
    // int16_t z = ((buffer[2] << 8) | (buffer[3]));
    int16_t y = ((buffer[4] << 8) | (buffer[5]));

    double x_scale = x * SCALE;
    double y_scale = y * SCALE;
    double angle = atan2(y_scale, x_scale);    
    double angle_degrees = angle * (180.0 / M_PI) + 180;

    return angle_degrees;
}


