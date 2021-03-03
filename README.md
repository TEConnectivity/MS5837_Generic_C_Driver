# MS5837 Generic C Driver
Generic C driver for the [MS5837 sensor](http://www.te.com/usa-en/product-CAT-BLPS0037.html)

![ms5837](http://www.te.com/content/dam/te-com/catalog/part/CAT/BLP/S00/CAT-BLPS0017-t1.jpg/jcr:content/renditions/product-details.png)

The MS5837 sensor is a self-contained pressure and temperature sensor that is  fully calibrated during manufacture. The sensor can operate from 1.5V to 3.6V. The sensor module includes a high-linearity pressure sensor and an ultra-low power 24 bit ΔΣ ADC with internal factory-calibrated coefficients.

### Specifications
* Measures pressure from 300mbar to 1200mbar
*	Measures temperature from -40°C to 125°C
*	I2C communication
*	Fully calibrated
*	Fast response time
*	Very low power consumption


### Driver features
* Connection test
* Reset
* Aquisition resolution management
* Temperature and pressure measurement


**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.

### I2C implementation layer
To complete the implementation layer for the i2c component several stubs must be implemented in a user provided i2c.h header. These stubs should be implemented and additional functionality added to each type/function as required on your platform.
```c
#ifndef MS5837_I2C_BACKEND
#define MS5837_I2C_BACKEND

/**
 * @brief The transfer direction of the i2c transaction
 */
enum i2c_transfer_direction {
  I2C_TRANSFER_WRITE = 0,
  I2C_TRANSFER_READ = 1,
};

/**
 * @brief The status of the last transaction
 */
enum status_code {
  STATUS_OK = 0x00,
  STATUS_ERR_OVERFLOW = 0x01,
  STATUS_ERR_TIMEOUT = 0x02,
};

/**
 * @brief The packet of information to send to send over i2c
 */
struct i2c_master_packet {
  // Address to slave device
  uint16_t address;
  // Length of data array
  uint16_t data_length;
  // Data array containing all data to be transferred
  uint8_t *data;
};

/**
 * @brief Initialise the master device
 */
void i2c_master_init(void);

/**
 * @brief Read a packet and wait 
 * 
 * @param packet 
 * @return enum status_code 
 */
enum status_code i2c_master_read_packet_wait(
    struct i2c_master_packet *const packet);

/**
 * @brief Write a packet and wait
 * 
 * @param packet 
 * @return enum status_code 
 */
enum status_code i2c_master_write_packet_wait(
    struct i2c_master_packet *const packet);

/**
 * @brief Write a packet and don't wait
 * 
 * @param packet 
 * @return enum status_code 
 */
enum status_code i2c_master_write_packet_wait_no_stop(
    struct i2c_master_packet *const packet);

/**
 * Delay in milliseconds
 */ 
void delay_ms(uint16_t);
#endif
```