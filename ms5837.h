/**
 * \file ms5837.h
 *
 * \brief MS5837 Temperature sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 *
 * \asf_license_stop
 *
 */

#ifndef MS5837_H_INCLUDED
#define MS5837_H_INCLUDED

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

enum ms5837_resolution_osr {
  ms5837_resolution_osr_256 = 0,
  ms5837_resolution_osr_512,
  ms5837_resolution_osr_1024,
  ms5837_resolution_osr_2048,
  ms5837_resolution_osr_4096,
  ms5837_resolution_osr_8192
};

enum ms5837_status {
  ms5837_status_ok,
  ms5837_status_no_i2c_acknowledge,
  ms5837_status_i2c_transfer_error,
  ms5837_status_crc_error
};

// Functions

/**
 * \brief Configures the SERCOM I2C master to be used with the ms5837 device.
 */
void ms5837_init(void);

/**
 * \brief Check whether MS5837 device is connected
 *
 * \return bool : status of MS5837
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
 */
bool ms5837_is_connected(void);

/**
 * \brief Reset the MS5837 device
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5837_status ms5837_reset(void);

/**
 * \brief Set  ADC resolution.
 *
 * \param[in] ms5837_resolution_osr : Resolution requested
 *
 */
void ms5837_set_resolution(enum ms5837_resolution_osr);

/**
 * \brief Reads the temperature and pressure ADC value and compute the
 * compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5837_status_crc_error : CRC check error on on the PROM coefficients
 */
enum ms5837_status ms5837_read_temperature_and_pressure(float *, float *);

#endif /* MS5837_H_INCLUDED */