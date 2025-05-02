/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "gpio_wakeup.h"
#include "bgt60ltr11aip.h"

/* Most development boards have "boot" button attached to GPIO0.
 * You can also change this to another pin.
 */
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32H2 \
    || CONFIG_IDF_TARGET_ESP32C6
#define BOOT_BUTTON_NUM         9
#elif CONFIG_IDF_TARGET_ESP32C5_MP_VERSION
#define BOOT_BUTTON_NUM         28
#elif CONFIG_IDF_TARGET_ESP32C5_BETA3_VERSION
#define BOOT_BUTTON_NUM         7
#elif CONFIG_IDF_TARGET_ESP32P4
#define BOOT_BUTTON_NUM         35
#else
#define BOOT_BUTTON_NUM         0
#endif

/* "Boot" button is active low */
#define GPIO_WAKEUP_LEVEL       0

static const char *TAG = "gpio_wakeup";
extern volatile int radar_td_rise_edge;

void example_wait_gpio_inactive(void)
{
    printf("Waiting for GPIO%d to go high...\n", RADAR_TD_GPIO);
    while (gpio_get_level(RADAR_TD_GPIO) == GPIO_WAKEUP_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t example_register_gpio_wakeup(void)
{
    /* Initialize GPIO */
    gpio_config_t config = {
            .pin_bit_mask = BIT64(RADAR_TD_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = false,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "Initialize GPIO%d failed", RADAR_TD_GPIO);

    radar_td_rise_edge = gpio_get_level(RADAR_TD_GPIO);
   
    /* Enable wake up from GPIO */
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(RADAR_TD_GPIO, GPIO_WAKEUP_LEVEL == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "Configure gpio as wakeup source failed");

    /* Make sure the GPIO is inactive and it won't trigger wakeup immediately */
    //example_wait_gpio_inactive();
    ESP_LOGI(TAG, "gpio wakeup source is ready");

    return ESP_OK;
}

esp_err_t example_register_gpio_wakeup_high(void)
{
    /* Initialize GPIO */
    gpio_config_t config = {
            .pin_bit_mask = BIT64(RADAR_TD_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = false,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "Initialize GPIO%d failed", RADAR_TD_GPIO);

    //radar_td_rise_edge = gpio_get_level(RADAR_TD_GPIO);
   
    /* Enable wake up from GPIO */
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(RADAR_TD_GPIO, GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "Configure gpio as wakeup source failed");

    /* Make sure the GPIO is inactive and it won't trigger wakeup immediately */
    //example_wait_gpio_inactive();
    ESP_LOGI(TAG, "gpio wakeup source is ready");

    return ESP_OK;
}

esp_err_t example_register_ext1_gpio_wakeup(void)
{
    // Configure GPIO_WAKEUP_NUM3 as input
    gpio_config_t config = {
        .pin_bit_mask = BIT64(EXT1_WAKEUP_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "Failed to configure GPIO %d", EXT1_WAKEUP_NUM);

    // Configure EXT1 to wake up when GPIO_WAKEUP_NUM3 goes high.
    // All pins in EXT1 must share the same wakeup level, so this works only if this is the only ext1 pin,
    // or if all ext1 pins have a matching active condition.
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup(BIT64(EXT1_WAKEUP_NUM), ESP_EXT1_WAKEUP_ANY_LOW),
                        TAG, "EXT1 configuration failed for GPIO %d", EXT1_WAKEUP_NUM);

    // Ensure the pin is in inactive state to avoid an immediate wakeup.
    //example_wait_gpio_inactive_3();
    ESP_LOGI(TAG, "EXT1 gpio wakeup source ready for GPIO %d", EXT1_WAKEUP_NUM);
    return ESP_OK;
}

esp_err_t initialize_gpio(int gpio_num, bool pull_up, bool pull_down, gpio_int_type_t intr_type)
{
    gpio_config_t config = {
        .pin_bit_mask = BIT64(gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = pull_down,
        .pull_up_en = pull_up,
        .intr_type = intr_type
    };

    esp_err_t ret = gpio_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialize GPIO%d failed: %s", gpio_num, esp_err_to_name(ret));
    }
    return ret;
}