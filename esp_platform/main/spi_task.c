#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "cc1101.h"   // <-- Include our CC1101 library
#include "bgt60l_spi.h" // Include the BGT60L SPI header
#include "driver/gpio.h"
#include "spi_task.h"

static const char *TAG_SPI = "spi_task";
TaskHandle_t spiTaskHandle = NULL; // Task handle for spi_task

#define SPI_TASK_STACK_SIZE 4096
#define SPI_TASK_PRIORITY   6  // Adjust priority as needed

// Function to initialize BGT60L device over SPI
void init_bgt60l(void)
{
    spi_bus_config_t buscfg_bgt60l = {
        .miso_io_num = CONFIG_SPI_BGT60L_MISO_GPIO, // Defined in sdkconfig
        .mosi_io_num = CONFIG_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_SPI_HOST, &buscfg_bgt60l, SPI_DMA_CH_AUTO));
    //ESP_LOGI(TAG_SPI, "SPI bus initialized for BGT60L");

    ESP_ERROR_CHECK(bgt60l_spi_init());
    ESP_ERROR_CHECK(mtb_s2go_radar_bgt60ltr11_pulsed_mode_init());
    //ESP_LOGI(TAG_SPI, "BGT60L pulsed mode initialized successfully");

    // Deinitialize the BGT60L SPI devices
    ESP_ERROR_CHECK(bgt60l_spi_deinit());
    ESP_ERROR_CHECK(spi_bus_free(CONFIG_SPI_HOST));
    //ESP_LOGI(TAG_SPI, "SPI bus freed after BGT60L init");
}

// Function to initialize CC1101 device over SPI
void init_cc1101(void)
{
    spi_bus_config_t buscfg_cc1101 = {
        .miso_io_num = CONFIG_SPI_MISO_GPIO,  // Adjust if needed
        .mosi_io_num = CONFIG_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_SPI_HOST, &buscfg_cc1101, SPI_DMA_CH_AUTO));
    //ESP_LOGI(TAG_SPI, "SPI bus initialized for CC1101");

    ESP_ERROR_CHECK(cc1101_init());
    //ESP_LOGI(TAG_SPI, "CC1101 initialized successfully");

}

void spi_task(void *pvParameters)
{
    
    //ESP_LOGI(TAG_SPI, "SPI bgt60l task started");

    // Initialize BGT60L and clean up
    init_bgt60l();
    
    // Add CC1101 Device

    //ESP_LOGI(TAG_SPI, "SPI CC1101 task started");

    // Initialize CC1101 after BGT60L
    init_cc1101();
    
    vTaskDelete(NULL);
}

void init_spi_task(void)
{
   xTaskCreate(spi_task, "spi_task", SPI_TASK_STACK_SIZE, NULL, SPI_TASK_PRIORITY, &spiTaskHandle);
}
