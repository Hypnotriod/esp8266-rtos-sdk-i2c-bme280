#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp8266/gpio_register.h"

#include "i2c_bme280.h"

static void bme280_task(void *prv)
{
    int32_t temp;
    uint32_t press;
    uint32_t hum;

    while (bme280_read_sensor_data())
    {
        temp = bme280_get_temperature();
        press = bme280_get_pressure();
        hum = bme280_get_humidity();

        printf("Temp: %d.%d DegC, ", (int)(temp / 100), (int)(temp % 100));
        printf("Pres: %d.%d hPa, ", (int)(press / 100), (int)(press % 100));
        printf("Hum: %d.%d pct \r\n", (int)(hum / 1024), (int)(hum % 1024));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    bme280_config_t bme280_config = bme280_config_default;
    if (bme280_init(bme280_config))
    {
        xTaskCreate(bme280_task, "bme280_task", 1024, NULL, 2, NULL);
    }

    while (1)
    {
    }
}
