#include "moduledata.h"
#include "portmacro.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static void read_nvs_data();

static nvs_handle_t my_nvs_handle;
TaskHandle_t nvsTaskHandle = NULL;

void nvs_init(void)
{
    // Init non volatile storage
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
        xTaskCreate(&nvs_task, "nvs_task", 2048, NULL, 5, &nvsTaskHandle);
    }
}
void nvs_task(void *pvParameter)
{
    while (1) {
        read_nvs_data();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void read_nvs_data() 
{
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    esp_err_t err = nvs_get_i32(my_nvs_handle, "restart_counter", &restart_counter);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Restart counter = %" PRIu32 "\n", restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            //printf("The value is not initialized yet!\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    /*
    printf("Updating restart counter in NVS ... ");
    restart_counter++;
    err = nvs_set_i32(my_nvs_handle, "restart_counter", restart_counter);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_nvs_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    */
}