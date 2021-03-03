/**
 * \file ms5837.c
 *
 * \brief MS5837 Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to ms5837 datasheet :
 * http://meas-spec.com/downloads/MS5837-30BA.pdf
 *
 */

#include "ms5837.h"

/**
 * The header "i2c.h" has to be implemented for your own platform to
 * conform the following protocol :
 *
 * enum i2c_transfer_direction {
 * 	I2C_TRANSFER_WRITE = 0,
 * 	I2C_TRANSFER_READ  = 1,
 * };
 *
 * enum status_code {
 * 	STATUS_OK           = 0x00,
 * 	STATUS_ERR_OVERFLOW	= 0x01,
 *		STATUS_ERR_TIMEOUT  = 0x02,
 * };
 *
 * struct i2c_master_packet {
 * 	// Address to slave device
 * 	uint16_t address;
 * 	// Length of data array
 * 	uint16_t data_length;
 * 	// Data array containing all data to be transferred
 * 	uint8_t *data;
 * };
 *
 * void i2c_master_init(void);
 * enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const
 *packet); enum status_code i2c_master_write_packet_wait(struct
 *i2c_master_packet *const packet); enum status_code
 *i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);
 */
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constants

// MS5837 device address
#define MS5837_ADDR 0x76  // 0b1110110

// MS5837 device commands
#define MS5837_RESET_COMMAND 0x1E
#define MS5837_START_PRESSURE_ADC_CONVERSION 0x40
#define MS5837_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5837_READ_ADC 0x00

#define MS5837_CONVERSION_OSR_MASK 0x0F

#define MS5837_CONVERSION_TIME_OSR_256 1000
#define MS5837_CONVERSION_TIME_OSR_512 2000
#define MS5837_CONVERSION_TIME_OSR_1024 3000
#define MS5837_CONVERSION_TIME_OSR_2048 5000
#define MS5837_CONVERSION_TIME_OSR_4096 9000
#define MS5837_CONVERSION_TIME_OSR_8192 17000

// MS5837 commands
#define MS5837_PROM_ADDRESS_READ_ADDRESS_0 0xA0
#define MS5837_PROM_ADDRESS_READ_ADDRESS_1 0xA2
#define MS5837_PROM_ADDRESS_READ_ADDRESS_2 0xA4
#define MS5837_PROM_ADDRESS_READ_ADDRESS_3 0xA6
#define MS5837_PROM_ADDRESS_READ_ADDRESS_4 0xA8
#define MS5837_PROM_ADDRESS_READ_ADDRESS_5 0xAA
#define MS5837_PROM_ADDRESS_READ_ADDRESS_6 0xAC
#define MS5837_PROM_ADDRESS_READ_ADDRESS_7 0xAE

// Coefficients indexes for temperature and pressure computation
#define MS5837_CRC_INDEX 0
#define MS5837_PRESSURE_SENSITIVITY_INDEX 1
#define MS5837_PRESSURE_OFFSET_INDEX 2
#define MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX 4
#define MS5837_REFERENCE_TEMPERATURE_INDEX 5
#define MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX 6
#define MS5837_COEFFICIENT_NUMBERS 7

// Static functions
static enum ms5837_status ms5837_write_command(uint8_t);
static enum ms5837_status ms5837_read_eeprom_coeff(uint8_t, uint16_t *);
static enum ms5837_status ms5837_read_eeprom(void);
static enum ms5837_status ms5837_conversion_and_read_adc(uint8_t, uint32_t *);
static bool ms5837_crc_check(uint16_t *n_prom, uint8_t crc);

enum ms5837_resolution_osr ms5837_resolution_osr;
static uint16_t eeprom_coeff[MS5837_COEFFICIENT_NUMBERS + 1];
static uint32_t conversion_time[6] = {
    MS5837_CONVERSION_TIME_OSR_256,  MS5837_CONVERSION_TIME_OSR_512,
    MS5837_CONVERSION_TIME_OSR_1024, MS5837_CONVERSION_TIME_OSR_2048,
    MS5837_CONVERSION_TIME_OSR_4096, MS5837_CONVERSION_TIME_OSR_8192};

// Default value to ensure coefficients are read before converting temperature
bool ms5837_coeff_read = false;

/**
 * \brief Configures the SERCOM I2C master to be used with the MS5837 device.
 */
void ms5837_init(void) {
  ms5837_resolution_osr = ms5837_resolution_osr_8192;

  /* Initialize and enable device with config. */
  i2c_master_init();
}

/**
 * \brief Check whether MS5837 device is connected
 *
 * \return bool : status of MS5837
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
 */
bool ms5837_is_connected(void) {
  enum status_code i2c_status;

  struct i2c_master_packet transfer = {
      .address = MS5837_ADDR,
      .data_length = 0,
      .data = NULL,
  };
  /* Do the transfer */
  i2c_status = i2c_master_write_packet_wait(&transfer);
  if (i2c_status != STATUS_OK) return false;

  return true;
}

/**
 * \brief Reset the MS5837 device
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5837_status ms5837_reset(void) {
  return ms5837_write_command(MS5837_RESET_COMMAND);
}

/**
 * \brief Set  ADC resolution.
 *
 * \param[in] ms5837_resolution_osr : Resolution requested
 *
 */
void ms5837_set_resolution(enum ms5837_resolution_osr res) {
  ms5837_resolution_osr = res;
  return;
}

/**
 * \brief Writes the MS5837 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5837_status ms5837_write_command(uint8_t cmd) {
  enum status_code i2c_status;
  uint8_t data[1];

  data[0] = cmd;

  struct i2c_master_packet transfer = {
      .address = MS5837_ADDR,
      .data_length = 1,
      .data = data,
  };
  /* Do the transfer */
  i2c_status = i2c_master_write_packet_wait(&transfer);
  if (i2c_status == STATUS_ERR_OVERFLOW)
    return ms5837_status_no_i2c_acknowledge;
  if (i2c_status != STATUS_OK) return ms5837_status_i2c_transfer_error;

  return ms5837_status_ok;
}

/**
 * \brief Reads the ms5837 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5837_status_crc_error : CRC check error on the coefficients
 */
enum ms5837_status ms5837_read_eeprom_coeff(uint8_t command, uint16_t *coeff) {
  enum ms5837_status status;
  enum status_code i2c_status;
  uint8_t buffer[2];

  buffer[0] = 0;
  buffer[1] = 0;

  /* Read data */
  struct i2c_master_packet read_transfer = {
      .address = MS5837_ADDR,
      .data_length = 2,
      .data = buffer,
  };

  // Send the conversion command
  status = ms5837_write_command(command);
  if (status != ms5837_status_ok) return status;

  i2c_status = i2c_master_read_packet_wait(&read_transfer);
  if (i2c_status == STATUS_ERR_OVERFLOW)
    return ms5837_status_no_i2c_acknowledge;
  if (i2c_status != STATUS_OK) return ms5837_status_i2c_transfer_error;

  *coeff = (buffer[0] << 8) | buffer[1];

  if (*coeff == 0) return ms5837_status_i2c_transfer_error;

  return ms5837_status_ok;
}

/**
 * \brief Reads the ms5837 EEPROM coefficients to store them for computation.
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5837_status_crc_error : CRC check error on the coefficients
 */
enum ms5837_status ms5837_read_eeprom(void) {
  enum ms5837_status status;
  uint8_t i;

  for (i = 0; i < MS5837_COEFFICIENT_NUMBERS; i++) {
    status = ms5837_read_eeprom_coeff(
        MS5837_PROM_ADDRESS_READ_ADDRESS_0 + i * 2, eeprom_coeff + i);
    if (status != ms5837_status_ok) return status;
  }

  if (!ms5837_crc_check(eeprom_coeff,
                        (eeprom_coeff[MS5837_CRC_INDEX] & 0xF000) >> 12))
    return ms5837_status_crc_error;

  ms5837_coeff_read = true;

  return ms5837_status_ok;
}

/**
 * \brief Triggers conversion and read ADC value
 *
 * \param[in] uint8_t : Command used for conversion (will determine Temperature
 * vs Pressure and osr) \param[out] uint32_t* : ADC value.
 *
 * \return ms5837_status : status of MS5837
 *       - ms5837_status_ok : I2C transfer completed successfully
 *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
 */
static enum ms5837_status ms5837_conversion_and_read_adc(uint8_t cmd,
                                                         uint32_t *adc) {
  enum ms5837_status status;
  enum status_code i2c_status;
  uint8_t buffer[3];

  buffer[0] = 0;
  buffer[1] = 0;
  buffer[2] = 0;

  /* Read data */
  struct i2c_master_packet read_transfer = {
      .address = MS5837_ADDR,
      .data_length = 3,
      .data = buffer,
  };

  status = ms5837_write_command(cmd);
  // delay conversion depending on resolution
  delay_ms(conversion_time[(cmd & MS5837_CONVERSION_OSR_MASK) / 2] / 1000);
  if (status != ms5837_status_ok) return status;

  // Send the read command
  status = ms5837_write_command(MS5837_READ_ADC);
  if (status != ms5837_status_ok) return status;

  i2c_status = i2c_master_read_packet_wait(&read_transfer);
  if (i2c_status == STATUS_ERR_OVERFLOW)
    return ms5837_status_no_i2c_acknowledge;
  if (i2c_status != STATUS_OK) return ms5837_status_i2c_transfer_error;

  *adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  return status;
}

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
 *       - ms5837_status_crc_error : CRC check error on the coefficients
 */
enum ms5837_status ms5837_read_temperature_and_pressure(float *temperature,
                                                        float *pressure) {
  enum ms5837_status status = ms5837_status_ok;
  uint32_t adc_temperature, adc_pressure;
  int32_t dT, TEMP;
  int64_t OFF, SENS, P, T2, OFF2, SENS2;
  uint8_t cmd;

  // If first time adc is requested, get EEPROM coefficients
  if (ms5837_coeff_read == false) status = ms5837_read_eeprom();
  if (status != ms5837_status_ok) return status;

  // First read temperature
  cmd = ms5837_resolution_osr * 2;
  cmd |= MS5837_START_TEMPERATURE_ADC_CONVERSION;
  status = ms5837_conversion_and_read_adc(cmd, &adc_temperature);
  if (status != ms5837_status_ok) return status;

  // Now read pressure
  cmd = ms5837_resolution_osr * 2;
  cmd |= MS5837_START_PRESSURE_ADC_CONVERSION;
  status = ms5837_conversion_and_read_adc(cmd, &adc_pressure);
  if (status != ms5837_status_ok) return status;

  if (adc_temperature == 0 || adc_pressure == 0)
    return ms5837_status_i2c_transfer_error;

  // Difference between actual and reference temperature = D2 - Tref
  dT = (int32_t)adc_temperature -
       ((int32_t)eeprom_coeff[MS5837_REFERENCE_TEMPERATURE_INDEX] << 8);

  // Actual temperature = 2000 + dT * TEMPSENS
  TEMP = 2000 +
         ((int64_t)dT *
              (int64_t)eeprom_coeff[MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX] >>
          23);

  // Second order temperature compensation
  if (TEMP < 2000) {
    T2 = (3 * ((int64_t)dT * (int64_t)dT)) >> 33;
    OFF2 = 3 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 2;
    SENS2 = 5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 8;

    if (TEMP < -1500) {
      OFF2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
      SENS2 += 4 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
    }
  } else {
    T2 = (2 * ((int64_t)dT * (int64_t)dT)) >> 37;
    OFF2 = ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) >> 4;
    SENS2 = 0;
  }

  // OFF = OFF_T1 + TCO * dT
  OFF = ((int64_t)(eeprom_coeff[MS5837_PRESSURE_OFFSET_INDEX]) << 16) +
        (((int64_t)(eeprom_coeff[MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) *
          dT) >>
         7);
  OFF -= OFF2;

  // Sensitivity at actual temperature = SENS_T1 + TCS * dT
  SENS =
      ((int64_t)eeprom_coeff[MS5837_PRESSURE_SENSITIVITY_INDEX] << 15) +
      (((int64_t)eeprom_coeff[MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] *
        dT) >>
       8);
  SENS -= SENS2;

  // Temperature compensated pressure = D1 * SENS - OFF
  P = (((adc_pressure * SENS) >> 21) - OFF) >> 13;

  *temperature = ((float)TEMP - T2) / 100;
  *pressure = (float)P / 100;

  return status;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 * \param[in] uint8_t : crc to compare with
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool ms5837_crc_check(uint16_t *n_prom, uint8_t crc) {
  uint8_t cnt, n_bit;
  uint16_t n_rem, crc_read;

  n_rem = 0x00;
  crc_read = n_prom[0];
  n_prom[MS5837_COEFFICIENT_NUMBERS] = 0;
  n_prom[0] = (0x0FFF & (n_prom[0]));  // Clear the CRC byte

  for (cnt = 0; cnt < (MS5837_COEFFICIENT_NUMBERS + 1) * 2; cnt++) {
    // Get next byte
    if (cnt % 2 == 1)
      n_rem ^= n_prom[cnt >> 1] & 0x00FF;
    else
      n_rem ^= n_prom[cnt >> 1] >> 8;

    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem <<= 1;
    }
  }
  n_rem >>= 12;
  n_prom[0] = crc_read;

  return (n_rem == crc);
}

#ifdef __cplusplus
}
#endif