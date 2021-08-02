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
    bme280_config_t bme280_config = bme280_config_default;
    int32_t temp;
    uint32_t press;
    uint32_t hum;

    if (bme280_init(bme280_config))
    {
        while (bme280_read_sensor_data())
        {
            temp = bme280_get_temperature();
            printf("Temp: %d.%d DegC", (int)(temp / 100), (int)(temp % 100));
            press = bme280_get_pressure();
            printf(", Pres: %d.%d hPa", (int)(press / 100), (int)(press % 100));
            if (bme280_is_humidity_supported())
            {
                hum = bme280_get_humidity();
                printf(", Hum: %d.%d pct \r\n", (int)(hum / 1024), (int)(hum % 1024));
            }
            else
            {
                printf("\r\n");
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    printf("bme280_task failed!");
    while (1)
    {
    }
}

void app_main()
{
    xTaskCreate(bme280_task, "bme280_task", 4096, NULL, 2, NULL);
}
