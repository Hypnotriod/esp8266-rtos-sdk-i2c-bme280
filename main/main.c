
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
#include "portmacro.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "lwip/ip4_addr.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp8266/gpio_register.h"

#define BME280_DEBUG
#include "i2c_bme280.h"

void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    bme280_init(BME280_MODE_FORCED);

    while (1)
    {
    }
}
