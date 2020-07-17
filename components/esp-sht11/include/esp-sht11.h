#ifndef ESP_SHT11_H
#define ESP_SHT11_H

void esp_sht11_init(int sda, int scl);
void esp_sht11_stop();
void sht11_get_value(float *humidity, float *temperature);
float sht11_calc_dewpoint(float h, float t);

#endif /* ESP_SHT11_H */
