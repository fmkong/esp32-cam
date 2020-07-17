#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp-sht11.h"

static const char *TAG = "APP_SHT11";

static TaskHandle_t xGetResultTaskHandle = NULL;

void app_get_result_task (void * pvParameters)
{
    float humidity = 0, temperature = 0;

    for (;;) {
        sht11_get_value(&humidity, &temperature);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_sht11_startup()
{
    esp_sht11_init(18, 17);
    xTaskCreate(app_get_result_task, "SHT11", 2048, NULL, tskIDLE_PRIORITY, &xGetResultTaskHandle );
}

void app_sht11_shutdown()
{
    vTaskDelete(xGetResultTaskHandle);
}