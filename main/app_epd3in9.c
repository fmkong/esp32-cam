#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "app_settings.h"
#include "app_wifi.h"
#include "app_camera_httpd.h"
#include "lwip/ip4_addr.h"
#include "image.h"

#include "epd3in9.h"
#include "epdpaint.h"
#include "icons.h"
#include "ubuntu10.h"
#include "ubuntu12.h"
#include "ubuntu14.h"
#include "ubuntu16.h"
#include "ubuntu18.h"
#include "ubuntu20.h"
#include "ubuntu22.h"
#include "ubuntu24.h"
#include "ubuntu8.h"
#include <math.h>

#define COLORED 1
#define UNCOLORED 0

typedef struct Forecasts {
    time_t time;
    char summary[50];
    char icon[20];
    double temperatureMax;
    double temperatureMin;
    double humidity;
    int pressure;
} Forecast;

static char summary[50];
static char icon[20];
static double temperature;
static double humidity;
static int pressure;
static double wind_speed;
static double wind_bearing;
static double precip_probability;

static Forecast forecasts[8];

static const char *TAG = "app_epd3in9";
static TaskHandle_t xUpdateEpdWeatherTaskHandle = NULL;

const char* deg_to_compass(int degrees)
{
    int val = floor((degrees / 22.5) + 0.5);
    const char* arr[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };
    return arr[(val % 16)];
}

static void weather_to_display_task(void* pvParameters)
{
    time_t now;
    struct tm timeinfo;

    char tmp_buff[30];

    unsigned char* frame_black = (unsigned char*)malloc(400 * 300 / 8);

    if (frame_black == NULL) {
        ESP_LOGE(TAG, "error");
    }

    paint(frame_black, 400, 300);

    clear(UNCOLORED);
    ESP_LOGI(TAG, "e-Paper cleared");

    // Current weather
    const tImage* image = NULL;

    if (strcmp(icon, "clear-day") == 0) {
        image = &widaysunny;
    } else if (strcmp(icon, "clear-night") == 0) {
        image = &winightclear;
    } else if (strcmp(icon, "rain") == 0) {
        image = &wirain;
    } else if (strcmp(icon, "snow") == 0) {
        image = &wisnow;
    } else if (strcmp(icon, "sleet") == 0) {
        image = &wisleet;
    } else if (strcmp(icon, "wind") == 0) {
        image = &wistrongwind;
    } else if (strcmp(icon, "fog") == 0) {
        image = &wifog;
    } else if (strcmp(icon, "cloudy") == 0) {
        image = &wicloudy;
    } else if (strcmp(icon, "partly-cloudy-day") == 0) {
        image = &widaycloudy;
    } else if (strcmp(icon, "partly-cloudy-night") == 0) {
        image = &winightaltcloudy;
    }

    if (image != NULL) {
        draw_bitmap_mono_in_center(2, 0, 500, 40, image);
    }

    sprintf(tmp_buff, "%0.1f º", temperature);
    draw_string_in_grid_align_center(3, 0, 400, 45, tmp_buff, &Ubuntu24);

    draw_string_in_grid_align_center(2, 1, 400, 65, summary, &Ubuntu12);

    sprintf(tmp_buff, "Humidity: %d%%", (int)(humidity * 100));
    draw_string_in_grid_align_center(2, 1, 400, 85, tmp_buff, &Ubuntu12);

    sprintf(tmp_buff, "Pressure:%d hPa", pressure);
    draw_string_in_grid_align_center(2, 1, 400, 105, tmp_buff, &Ubuntu12);

    sprintf(tmp_buff, "Wind :%d km/h (%s)", (int)round(wind_speed * 3.6), deg_to_compass(wind_bearing));
    draw_string_in_grid_align_center(2, 1, 400, 125, tmp_buff, &Ubuntu12);

    sprintf(tmp_buff, "Chance of Precipitation : %d%%", (int)round(precip_probability * 100));
    draw_string_in_grid_align_center(2, 1, 400, 145, tmp_buff, &Ubuntu12);

    ESP_LOGI(TAG, "e-Paper werther current");
    for (size_t i = 0; i < (sizeof(forecasts) / sizeof(Forecast)); i++) {
        struct tm timeinfo;
        setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0", 1);
        tzset();
        localtime_r(&forecasts[i].time, &timeinfo);
        char day[20];
        char date[20];
        strftime(date, sizeof(date), "%d - %m", &timeinfo);
        strftime(day, sizeof(date), "%A", &timeinfo);

        if (i == 0) {
            sprintf(day, "Today");
        }

        if (i == 1) {
            sprintf(day, "Tomorrow");
        }

        draw_string_in_grid_align_center(7, i, 400, 210, day, &Ubuntu10);

        draw_string_in_grid_align_center(7, i, 400, 225, date, &Ubuntu10);

        sprintf(tmp_buff, "%d - %d º", (int)round(forecasts[i].temperatureMin), (int)round(forecasts[i].temperatureMax));
        draw_string_in_grid_align_center(7, i, 400, 240, tmp_buff, &Ubuntu10);

        const tImage* forecast_image = NULL;

        if (strcmp(forecasts[i].icon, "clear-day") == 0) {
            forecast_image = &daysunny;
        } else if (strcmp(forecasts[i].icon, "clear-night") == 0) {
            forecast_image = &nightclear;
        } else if (strcmp(forecasts[i].icon, "rain") == 0) {
            forecast_image = &rain;
        } else if (strcmp(forecasts[i].icon, "snow") == 0) {
            forecast_image = &snow;
        } else if (strcmp(forecasts[i].icon, "sleet") == 0) {
            forecast_image = &sleet;
        } else if (strcmp(forecasts[i].icon, "wind") == 0) {
            forecast_image = &strongwind;
        } else if (strcmp(forecasts[i].icon, "fog") == 0) {
            forecast_image = &fog;
        } else if (strcmp(forecasts[i].icon, "cloudy") == 0) {
            forecast_image = &cloudy;
        } else if (strcmp(forecasts[i].icon, "partly-cloudy-day") == 0) {
            forecast_image = &daycloudy;
        } else if (strcmp(forecasts[i].icon, "partly-cloudy-night") == 0) {
            forecast_image = &nightaltcloudy;
        }

        if (forecast_image != NULL) {
            draw_bitmap_mono_in_center(7, i, 400, 255, forecast_image);
        }
    }
    ESP_LOGI(TAG, "e-Paper werther daily");

    draw_string_in_grid_align_left(1, 0, 2, 400, 0, CONFIG_PLACE_NAME, &Ubuntu12);

    time(&now);
    char strftime_buf[64];
    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "CTS-8", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "Last updated: %e %b %H:%M", &timeinfo);

    draw_string_in_grid_align_right(1, 0, 2, 400, 0, strftime_buf, &Ubuntu12);

    draw_horizontal_line(0, 14, 400, COLORED);
    draw_horizontal_line(0, 200, 400, COLORED);
    draw_horizontal_line(0, 0, 400, COLORED);
    draw_vertical_line(0, 0, 300, COLORED);
    draw_horizontal_line(0, 299, 400, COLORED);
    draw_vertical_line(399, 0, 300, COLORED);

    for (size_t i = 1; i < 7; i++) {
        draw_vertical_line((400 / 7 * i), 200, 138, COLORED);
    }

    ESP_LOGI(TAG, "e-Paper werther last update");
    // /* Display the frame buffer */
    display_frame(NULL, frame_black);
    ESP_LOGI(TAG, "e-Paper disaplay frame");

    int count = 0;
    while (true) {
        sprintf(tmp_buff, "test cnt : %d", count);
        draw_string_in_grid_align_center(2, 1, 40, 50, tmp_buff, &Ubuntu12);
        display_frame(frame_black, NULL);
        count++;
        ESP_LOGI(TAG, "e-Paper disaplay partial: %d", count);
        // vTaskDelay(5000 / portTICK_RATE_MS);
    }

    // epd3in9_sleep();

    ESP_LOGI(TAG, "e-Paper sleep");
    vTaskDelete(NULL);
}

static void time_to_display_task(void* pvParameters)
{
    while(1) {

    }
}

void app_epd_startup()
{
    if (epd3in9_init() != 0) {
        ESP_LOGE(TAG, "e-Paper init failed");
        vTaskDelay(2000 / portTICK_RATE_MS);
        return;
    }
    // ESP_LOGI(TAG, "e-Paper initialized");
    // clear_frame();
    // xTaskCreate(weather_to_display_task, "weather_display", 10240, NULL, tskIDLE_PRIORITY, &xUpdateEpdWeatherTaskHandle );

}

void app_epd_shutdown()
{
    vTaskDelete(xUpdateEpdWeatherTaskHandle);
}