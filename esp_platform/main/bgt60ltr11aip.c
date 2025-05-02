#include <string.h>
#include "driver/gpio.h"
#include "bgt60ltr11aip.h"
#include "esp_log.h"  // Add this include for ESP_LOGI and related macros
#include "esp_attr.h" // For IRAM_ATTR
#include <stdbool.h>
#include "driver/uart.h" // Include UART functions like uart_driver_delete

// Global flag indicating target detection
volatile bool target_detected = false;
volatile int radar_td_rise_edge = 0;   // <-- Declare global variable
static const char *TAG_BGT60LTR = "BGT60LTR11AIP";

// Radar interrupt handler (runs in IRAM)
static void IRAM_ATTR radar_int_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg; // GPIO number that triggered the interrupt

    int level = gpio_get_level(RADAR_TD_GPIO);

    // Handle radar interrupt here (e.g., set a flag or send an event)
    ESP_EARLY_LOGI(TAG_BGT60LTR, "Radar interrupt  on GPIO %d, level: %d", gpio_num, level);
}

// Function to configure the radar GPIOs
void configure_radar_gpio(void)
{
    // Configure the interrupt GPIO:
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;   // Interrupt on both rising and falling edges
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RADAR_TD_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf)); 
    
    // Add ISR handler for the radar interrupt GPIO
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADAR_TD_GPIO, radar_int_handler, (void*) RADAR_TD_GPIO));
    //ESP_ERROR_CHECK(gpio_isr_handler_add(TMOS_INT_GPIO, tmos_int_handler, (void*) TMOS_INT_GPIO));

    ESP_LOGI(TAG_BGT60LTR, "Motion interrupt GPIOs configured: bgt60ltr pin %d", RADAR_TD_GPIO);
}

bool read_radar_pd(void)
{
    int level = gpio_get_level(RADAR_PD_GPIO);
    bool is_high = level != 0;
    if (is_high) {
        ESP_LOGI(TAG_BGT60LTR, "RADAR_PD_GPIO PHASE is HIGH (Target approaching)");
    } else {
        ESP_LOGI(TAG_BGT60LTR, "RADAR_PD_GPIO PHASE is LOW (Target departing)");
    }

    // If target_detected is set, also read and log RADAR_INT_GPIO level
    if (target_detected) {
        int int_level = gpio_get_level(RADAR_TD_GPIO);
        ESP_LOGI(TAG_BGT60LTR, "RADAR_INT_GPIO level is %d", int_level);
        // Since RADAR_INT_GPIO is active low, update target_detected accordingly.
        target_detected = (int_level == 0);
    }
    return is_high;
}

void disable_bgt60ltr_interrupt_for_ota(void) {
    // Disable radar interrupt
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADAR_TD_GPIO));
    ESP_LOGI(TAG_BGT60LTR, "Radar interrupt disabled for OTA mode");
}
