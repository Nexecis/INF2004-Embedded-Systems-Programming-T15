#include "pico/stdlib.h"
#include "hardware/i2c.h"

/*! \brief   Initialise the pins and I2C HW block
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \param baudrate Baudrate in Hz (e.g. 100kHz is 100000)
 * \param sda_gpio GPIO pin number of SDA 
 * \param scl_gpio GPIO pin number of SCL 
 * 
 * \return Actual set baudrate
 */
uint init_i2c_pin(i2c_inst_t *i2c, uint baudrate, uint sda_gpio, uint scl_gpio);

/*! \brief   Initialise magnometer
 * 
 * \return Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged or no device present.
 */
int init_magnometer();


/*! \brief   Gets angle in degrees
 * 
 * \return Angle in degrees based on the X and Y values from the magnometer
 */
double magnometer_angle();


