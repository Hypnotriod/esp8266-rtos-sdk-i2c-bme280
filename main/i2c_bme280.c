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

#include <math.h>
#include "driver/i2c.h"

#include "i2c_bme280.h"

uint16_t calib_dig_T1;
int16_t calib_dig_T2;
int16_t calib_dig_T3;
uint16_t calib_dig_P1;
int16_t calib_dig_P2;
int16_t calib_dig_P3;
int16_t calib_dig_P4;
int16_t calib_dig_P5;
int16_t calib_dig_P6;
int16_t calib_dig_P7;
int16_t calib_dig_P8;
int16_t calib_dig_P9;
int8_t calib_dig_H1;
int16_t calib_dig_H2;
int8_t calib_dig_H3;
int16_t calib_dig_H4;
int16_t calib_dig_H5;
int8_t calib_dig_H6;

uint8_t osrs_t = 1; // Temperature oversampling x 1
uint8_t osrs_p = 1; // Pressure oversampling x 1
uint8_t osrs_h = 1; // Humidity oversampling x 1

uint8_t t_sb = 4;	  // Tstandby, 5=1000ms, 4=500ms
uint8_t filter = 0;	  // Filter off
uint8_t spi3w_en = 0; // 3-wire SPI Disable

uint8_t bme280_operation_mode = BME280_MODE_NORMAL;

uint32_t hum_raw, temp_raw, pres_raw;
int32_t t_fine;
int32_t temp_act;
uint32_t press_act, hum_act;

bool bme280_read_data(uint8_t read_reg, uint8_t *data, size_t data_len)
{
	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BME280_W, true);
	i2c_master_write_byte(cmd, read_reg, true);
	i2c_master_write_byte(cmd, BME280_R, true);
	i2c_master_read(cmd, data, data_len, true);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return (err == ESP_OK);
}

bool bme280_write_data(uint8_t write_reg, uint8_t *data, size_t data_len)
{
	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BME280_W, true);
	i2c_master_write_byte(cmd, write_reg, true);
	i2c_master_write(cmd, data, data_len, true);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return (err == ESP_OK);
}

bool i2c_master_init()
{
	esp_err_t err;
	i2c_config_t conf;
	int i2c_master_port = I2C_NUM_0;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.clk_stretch_tick = 300;
	err = i2c_driver_install(i2c_master_port, conf.mode);
	if (err != ESP_OK)
	{
#ifdef BME280_DEBUG
		printf("i2c_master_init: i2c_driver_install error!\r\n");
#endif
		return false;
	}
	err = i2c_param_config(i2c_master_port, &conf);
	if (err != ESP_OK)
	{
#ifdef BME280_DEBUG
		printf("i2c_master_init: i2c_param_config error!\r\n");
#endif
		return false;
	}
	return true;
}

bool bme280_write_config_registers(void)
{
	uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | bme280_operation_mode;
	uint8_t ctrl_hum_reg = osrs_h;
	uint8_t config_reg = (t_sb << 5) | (filter << 2) | spi3w_en;

	if (!bme280_write_data(BME280_REG_CTRL_HUM, &ctrl_hum_reg, 1) ||
		!bme280_write_data(BME280_REG_CTRL_MEAS, &ctrl_meas_reg, 1) ||
		!bme280_write_data(BME280_REG_CONFIG, &config_reg, 1))
	{
#ifdef BME280_DEBUG
		printf("bme280_write_config_registers: error!\r\n");
#endif
		return false;
	}

	return true;
}

bool bme280_verify_chip_id(void)
{
	uint8_t version;

	if (!bme280_read_data(BME280_CHIP_ID_REG, &version, 1))
	{
#ifdef BME280_DEBUG
		printf("bme280_verify_chip_id: error!\r\n");
#endif
		return false;
	}

	if (version != BME280_CHIP_ID)
	{
#ifdef BME280_DEBUG
		printf("bme280_verify_chip_id: expected chip id 0x%X, found chip id 0x%X\r\n", BME280_CHIP_ID, version);
#endif
		return false;
	}

	return true;
}

int32_t bme280_get_t_fine()
{
	return t_fine;
}

int32_t bme280_get_temperature()
{
	return temp_act;
}

uint32_t bme280_get_pressure()
{
	return press_act;
}

uint32_t bme280_get_humidity()
{
	return hum_act;
}

uint32_t bme280_get_temperature_raw()
{
	return temp_raw;
}

uint32_t bme280_get_tressure_raw()
{
	return pres_raw;
}

uint32_t bme280_get_humidity_raw()
{
	return hum_raw;
}

int32_t bme280_calibration_temp(int32_t adc_T)
{

	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)calib_dig_T1 << 1))) * ((int32_t)calib_dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib_dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_dig_T1))) >> 12) * ((int32_t)calib_dig_T3)) >> 14;

	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bme280_calibration_press(int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t P;
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_dig_P6);
	var2 = var2 + ((var1 * ((int32_t)calib_dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)calib_dig_P4) << 16);
	var1 = (((calib_dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)calib_dig_P1)) >> 15);
	if (var1 == 0)
	{
		return 0;
	}
	P = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (P < 0x80000000)
	{
		P = (P << 1) / ((uint32_t)var1);
	}
	else
	{
		P = (P / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)calib_dig_P9) * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(P >> 2)) * ((int32_t)calib_dig_P8)) >> 13;
	P = (uint32_t)((int32_t)P + ((var1 + var2 + calib_dig_P7) >> 4));
	return P;
}

uint32_t bme280_calibration_hum(int32_t adc_H)
{
	int32_t v_x1;

	v_x1 = (t_fine - ((int32_t)76800));
	v_x1 = (((((adc_H << 14) - (((int32_t)calib_dig_H4) << 20) - (((int32_t)calib_dig_H5) * v_x1)) +
			  ((int32_t)16384)) >>
			 15) *
			(((((((v_x1 * ((int32_t)calib_dig_H6)) >> 10) *
				 (((v_x1 * ((int32_t)calib_dig_H3)) >> 11) + ((int32_t)32768))) >>
				10) +
			   ((int32_t)2097152)) *
				  ((int32_t)calib_dig_H2) +
			  8192) >>
			 14));
	v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)calib_dig_H1)) >> 4));
	v_x1 = (v_x1 < 0 ? 0 : v_x1);
	v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
	return (uint32_t)(v_x1 >> 12);
}

bool bme280_send_i2c_trigger_forced_read()
{
	uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | bme280_operation_mode;

	if (!bme280_write_data(BME280_REG_CTRL_MEAS, &ctrl_meas_reg, 1))
	{
#ifdef BME280_DEBUG
		printf("bme280_send_i2c_trigger_forced_read: error!\r\n");
#endif
		return false;
	}

	os_delay_us(10000); // wait 10ms for worst case max sensor read time

	return true;
}

bool bme280_send_i2c_read_sensor_data()
{
	uint8_t data[8];

#ifdef BME280_DEBUG
	printf("bme280_send_i2c_read_sensor_data: operation mode = %d\r\n", bme280_operation_mode);
#endif

	if (bme280_operation_mode == BME280_MODE_FORCED)
	{
		if (!bme280_send_i2c_trigger_forced_read())
		{
			return false;
		}
	}

	if (!bme280_read_data(0xF7, data, sizeof(data)))
	{
#ifdef BME280_DEBUG
		printf("bme280_send_i2c_read_sensor_data: section 0xF7 error!\r\n");
#endif
		return false;
	}

	// 0xF7 - pressure
	pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
#ifdef BME280_DEBUG
	printf("pres_raw 0: %X, pres_raw 1: %X, pres_raw 2: %X\r\n", data[0], data[1], data[2]);
#endif

	//0xFA - temp
	temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
#ifdef BME280_DEBUG
	printf("temp_raw 3: %X, temp_raw 4: %X, temp_raw 5: %X\r\n", data[3], data[4], data[5]);
#endif

	//0xFD - humidity
	hum_raw = (data[6] << 8) | data[7];
#ifdef BME280_DEBUG
	printf("hum_raw 6: %X, hum_raw 7: %X\r\n", data[6], data[7]);
#endif

	return true;
}

bool bme280_read_calibration_registers(void)
{
	uint8_t data[24];

	// ***************** Read section 0x88:0x9F *****************
	if (!bme280_read_data(0x88, data, 24))
	{
#ifdef BME280_DEBUG
		printf("bme280_read_calibration_registers: section 0x88:0x9F error!\r\n");
#endif
		return false;
	}

	// 0x88 / 0x89
	calib_dig_T1 = data[0] | (data[1] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_T1 = %u\r\n", data[0], data[1], calib_dig_T1);
#endif

	// 0x8A / 0x8B
	calib_dig_T2 = data[2] | (data[3] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_T2 = %d\r\n", data[2], data[3], calib_dig_T2);
#endif

	// 0x8C / 0x8D
	calib_dig_T3 = data[4] | (data[5] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_T3 = %d\r\n", data[4], data[5], calib_dig_T3);
#endif

	// 0x8E / 0x8F
	calib_dig_P1 = data[6] | (data[7] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P1 = %u\r\n", data[6], data[7], calib_dig_P1);
#endif

	// 0x90 / 0x91
	calib_dig_P2 = data[8] | (data[9] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P2 = %d\r\n", data[8], data[9], calib_dig_P2);
#endif

	// 0x92 / 0x93
	calib_dig_P3 = data[10] | (data[11] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P3 = %d\r\n", data[10], data[11], calib_dig_P3);
#endif

	// 0x94 / 0x95
	calib_dig_P4 = data[12] | (data[13] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P4 = %d\r\n", data[12], data[13], calib_dig_P4);
#endif

	// 0x96 / 0x97
	calib_dig_P5 = data[14] | (data[15] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P5 = %d\r\n", data[14], data[15], calib_dig_P5);
#endif

	// 0x98 / 0x99
	calib_dig_P6 = data[16] | (data[17] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P6 = %d\r\n", data[16], data[17], calib_dig_P6);
#endif

	// 0x9A / 0x9B
	calib_dig_P7 = data[18] | (data[19] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P7 = %d\r\n", data[18], data[19], calib_dig_P7);
#endif

	// 0x9C / 0x9D
	calib_dig_P8 = data[20] | (data[21] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P8 = %d\r\n", data[20], data[21], calib_dig_P8);
#endif

	// 0x9E / 0x9F
	calib_dig_P9 = data[22] | (data[23] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_P9 = %d\r\n", data[22], data[23], calib_dig_P9);
#endif

	// ***************** Read section 0xA1 *****************
	if (!bme280_read_data(0xA1, data, 1))
	{
#ifdef BME280_DEBUG
		printf("bme280_read_calibration_registers: section 0xA1 error!\r\n");
#endif
		return false;
	}

	// 0xA1
	calib_dig_H1 = data[0];
#ifdef BME280_DEBUG
	printf("msb: 0x%X = calib_dig_H1 = %d\r\n", data[0], calib_dig_H1);
#endif

	// ***************** Read section 0xE1:0xE6 *****************
	if (!bme280_read_data(0xE1, data, 7))
	{
#ifdef BME280_DEBUG
		printf("bme280_read_calibration_registers: section 0xE1 error!\r\n");
#endif
		return false;
	}

	// 0xE1 / 0xE2
	calib_dig_H2 = data[0] | (data[1] << 8);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_H2 = %d\r\n", data[0], data[1], calib_dig_H2);
#endif

	// 0xE3
	calib_dig_H3 = data[2];
#ifdef BME280_DEBUG
	printf("lsb 0x%X = calib_dig_H3 = %d\r\n", data[2], calib_dig_H3);
#endif

	// 0xE4 / 0xE5[3:0]
	calib_dig_H4 = (data[3] << 4) | (0x0f & data[4]);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_H4 = %d\r\n", data[3], data[4], calib_dig_H4);
#endif

	// 0xE5[7:4] / 0xE6
	calib_dig_H5 = (data[4] >> 4) | (data[5] << 4);
#ifdef BME280_DEBUG
	printf("lsb 0x%X, msb: 0x%X = calib_dig_H5 = %d\r\n", data[5], data[4], calib_dig_H5);
#endif

	// 0xE7
	calib_dig_H6 = data[6];
#ifdef BME280_DEBUG
	printf("lsb 0x%X = calib_dig_H6 = %d\r\n", data[6], calib_dig_H6);
#endif

	return true;
}

bool bme280_init(uint8_t operation_mode)
{
	bme280_operation_mode = operation_mode;

	if (!i2c_master_init() ||
		!bme280_verify_chip_id() ||
		!bme280_write_config_registers() ||
		!bme280_read_calibration_registers())
	{
#ifdef BME280_DEBUG
		printf("bme280_init: failed\r\n");
#endif
		return false;
	}

#ifdef BME280_DEBUG
	printf("bme280_init: success\r\n");
#endif

	return true;
}

bool bme280_read_sensor_data()
{
	if (!bme280_send_i2c_read_sensor_data())
	{
		return false;
	}

	temp_act = bme280_calibration_temp(temp_raw);
	press_act = bme280_calibration_press(pres_raw);
	hum_act = bme280_calibration_hum(hum_raw);

	return true;
}
