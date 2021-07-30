/*
The driver for the temperature, pressure, & humidity sensor BME280
Official repository: https://github.com/RyAndrew/esp8266_i2c_bme280
Adapted From: https://github.com/CHERTS/esp8266-i2c_bmp180
This driver depends on the I2C driver https://github.com/zarya/esp8266_i2c_driver/

The MIT License (MIT)

Copyright (C) 2015 Andrew Rymarczyk

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

#define I2C_MASTER_SCL_IO 2  /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 14 /*!< gpio number for I2C master data  */

#define BME280_W 0xEC
#define BME280_R 0xED
#define BME280_CHIP_ID_REG 0xD0
#define BME280_CHIP_ID 0x60

#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5

#define BME280_MODE_NORMAL 0x03 // reads sensors at set interval
#define BME280_MODE_FORCED 0x01 // reads sensors once when you write this register

#define BME280_DEBUG 1 // uncomment for debugging messages

bool bme280_init(uint8_t operation_mode);
bool bme280_verify_chip_id(void);
bool bme280_read_sensor_data();

signed long int bme280_get_t_fine();
signed long int bme280_get_temperature();
unsigned long int bme280_get_pressure();
unsigned long int bme280_get_humidity();
unsigned long int bme280_get_temperature_raw();
unsigned long int bme280_get_tressure_raw();
unsigned long int bme280_get_humidity_raw();

#endif