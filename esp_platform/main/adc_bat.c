#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "adc_bat.h"

#define DEFAULT_VREF    3900            // Default reference voltage in mV adjusted based on ADC_ATTEN_DBX
#define NO_OF_SAMPLES   64              // Multisampling
#define ADC_CHANNEL     ADC_CHANNEL_0  // For GPIO1
#define ADC_MAX_VALUE   4095

/*ADC_ATTEN_DB_0	~0.95 V
ADC_ATTEN_DB_2_5	~1.34 V
ADC_ATTEN_DB_6	~2.00 V
ADC_ATTEN_DB_12	~3.9 V */

static const char *TAG_ADC = "ADC_BAT";

static void init_adc(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_12);  // Updated attenuation value
}

// This function calculates battery voltage without calibration.
uint32_t read_battery_voltage(void)
{
    uint32_t adc_reading = 0;
    init_adc();
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(ADC_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;

    //ESP_LOGI(TAG_ADC, "ADC raw reading: %lu", (unsigned long)adc_reading);

    // Convert raw reading to voltage (mV) using DEFAULT_VREF
    uint32_t voltage = (adc_reading * DEFAULT_VREF) / ADC_MAX_VALUE;
    //ESP_LOGI(TAG_ADC, "Battery voltage (no calibration): %lu mV", (unsigned long)voltage);
    return voltage;
}

void battery_task(void *pvParameter)
{
    while (1) {
        uint32_t voltage = read_battery_voltage();
        //ESP_LOGI("MAIN", "Battery Voltage: %lu mV", (unsigned long)voltage);
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
    // Optionally delete the task if it ever exits (it will not in this loop)
    vTaskDelete(NULL);
}