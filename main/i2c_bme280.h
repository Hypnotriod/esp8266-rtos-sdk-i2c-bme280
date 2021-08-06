/*
BME280 (temperature, pressure & humiditty) and BMP280 (temperature & pressure) sensors I2C driver port
of RyAndrew's driver https://github.com/RyAndrew/esp8266_i2c_bme280, for ESP8266 RTOS SDK by Ilya Pikin.

The MIT License (MIT)

Copyright (C) 2015 Andrew Rymarczyk
Copyright (C) 2021 Ilya Pikin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef __I2C_BME280_H
#define __I2C_BME280_H

#define BME280_I2C_MASTER_SCL_PIN_DEFAULT 5
#define BME280_I2C_MASTER_SDA_PIN_DEFAULT 4

#define BME280_I2C_ADDR_PRIM 0x76 // SDO to GND
#define BME280_I2C_ADDR_SEC 0x77  // SDO to VDDIO

#define BME280_NO_OVERSAMPLING 0x00
#define BME280_OVERSAMPLING_1X 0x01
#define BME280_OVERSAMPLING_2X 0x02
#define BME280_OVERSAMPLING_4X 0x03
#define BME280_OVERSAMPLING_8X 0x04
#define BME280_OVERSAMPLING_16X 0x05

#define BME280_FILTER_COEFF_OFF 0x00
#define BME280_FILTER_COEFF_2 0x01
#define BME280_FILTER_COEFF_4 0x02
#define BME280_FILTER_COEFF_8 0x03
#define BME280_FILTER_COEFF_16 0x04

#define BME280_STANDBY_TIME_0_5_MS 0x00
#define BME280_STANDBY_TIME_62_5_MS 0x01
#define BME280_STANDBY_TIME_125_MS 0x02
#define BME280_STANDBY_TIME_250_MS 0x03
#define BME280_STANDBY_TIME_500_MS 0x04
#define BME280_STANDBY_TIME_1000_MS 0x05
#define BME280_STANDBY_TIME_10_MS 0x06
#define BME280_STANDBY_TIME_20_MS 0x07

#define BME280_CHIP_ID_REG 0xD0
#define BME280_CHIP_ID 0x60
#define BMP280_CHIP_ID 0x58

#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5

#define BME280_MODE_NORMAL 0x03 // reads sensors at set interval
#define BME280_MODE_FORCED 0x01 // reads sensors once when you write this register

// #define BME280_DEBUG // uncomment for debugging messages

#ifdef BME280_DEBUG
#define BME280_DEBUG_MSG(...) printf(__VA_ARGS__)
#else
#define BME280_DEBUG_MSG(...)
#endif

#define bme280_config_default                          \
    {                                                  \
        .gpio_scl = BME280_I2C_MASTER_SCL_PIN_DEFAULT, \
        .gpio_sda = BME280_I2C_MASTER_SDA_PIN_DEFAULT, \
        .address = BME280_I2C_ADDR_PRIM,               \
        .operation_mode = BME280_MODE_FORCED,          \
        .osrs_t = BME280_OVERSAMPLING_2X,              \
        .osrs_p = BME280_OVERSAMPLING_16X,             \
        .osrs_h = BME280_OVERSAMPLING_1X,              \
        .t_sb = BME280_STANDBY_TIME_500_MS,            \
        .filter = BME280_FILTER_COEFF_OFF,             \
        .spi3w_en = 0                                  \
    }

typedef struct
{
    uint8_t gpio_scl;       // I2C SCl pin number
    uint8_t gpio_sda;       // I2C SDA pin number
    uint8_t address;        // Slave address
    uint8_t operation_mode; // Operation mode
    uint8_t osrs_t;         // Temperature oversampling
    uint8_t osrs_p;         // Pressure oversampling
    uint8_t osrs_h;         // Humidity oversampling
    uint8_t t_sb;           // Tstandby
    uint8_t filter;         // Filter
    uint8_t spi3w_en;       // 3-wire SPI Disable
} bme280_config_t;

bool bme280_init(bme280_config_t config);
void bme280_dispose();
bool bme280_trigger_forced_read();
bool bme280_read_sensor_data();

int32_t bme280_get_t_fine();
int32_t bme280_get_temperature();
uint32_t bme280_get_pressure();
uint32_t bme280_get_humidity();
uint32_t bme280_get_temperature_raw();
uint32_t bme280_get_tressure_raw();
uint32_t bme280_get_humidity_raw();
bool bme280_is_temperature_supported();
bool bme280_is_pressure_supported();
bool bme280_is_humidity_supported();

#endif