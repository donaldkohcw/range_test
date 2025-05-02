#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "gap.h"
#include "gatt_svr.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "ota_event_bits.h" // Include the common header file
#include "esp_sleep.h" // Include the deep sleep header
#include "moduledata.h"
#include "bgt60ltr11aip.h"
#include "cc1101.h"
#include "adc_bat.h"
#include "sths34pf80_task.h"
#include "spi_task.h"
#include "esp_bt.h"
#include "gpio_wakeup.h"
#include "app_event_group.h"
#include "driver/uart.h"

#define TAG_MAIN "main"
//#define DEEP_SLEEP_MODE

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
/*deep Sleep Wakeup: The ext1 wakeup feature used in your code works only on a subset of GPIOs (usually GPIO 7–14 on some ESP boards).*/
#define BUTTON_GPIO 9  // Define the GPIO number for the button here

//#define RESET_MODE_THRESHOLD 30000  // 30 seconds - place holder only, might remove this
#define OTA_MODE_THRESHOLD 5000    // 5 seconds
#define PAIRING_MODE_THRESHOLD 2000  // Adjust as needed
#define BLE_CONNECTION_TIMEOUT 30000 // 30 seconds
#define OTA_TRANSFER_TIMEOUT 300000 // 300 seconds
#define DEEP_SLEEP_DURATION 120000000 // 1 minutes in microseconds

static uint16_t conn_handle = 0; // Global variable to store the connection handle
static uint8_t s_led_state = 0;
static TimerHandle_t button_timer;
static bool button_pressed = false;
static int64_t button_press_time = 0;  // Declare the button_press_time variable
static bool ota_mode = false; // Flag to indicate OTA mode
static bool low_bat_mode = false; // Add a new global flag for low bat mode
EventGroupHandle_t ble_event_group; // Define the ble_event_group variable
TaskHandle_t sensorTaskHandle = NULL; // Global variable for sensor task handle
TaskHandle_t blinkTaskHandle = NULL; // Global variable for blink task handle
TaskHandle_t batteryTaskHandle = NULL; // Task handle for battery_task
TaskHandle_t sths34pf80_tmos_presence_detection_task_handle = NULL;
static bool ota_task_started = false; // Global variable for OTA task started flag

// Add SPI initialization function declaration
extern void init_spi_task(void);
void shutdown_ble_and_sleep(void);
static void deep_sleep_task(void *args);
static esp_sleep_wakeup_cause_t wakeup_reason; // Global variable to store wakeup reason

EventGroupHandle_t app_event_group;

bool run_diagnostics() {
    // do some diagnostics
    return true;
  }

// Function to get the current time in microseconds
uint32_t get_run_time_counter_value(void)
{
    return (uint32_t) (esp_timer_get_time() / 1000); // Convert to milliseconds
}

// Function to configure the timer for runtime statistics
void configure_timer_for_run_time_stats(void)
{
    // No additional configuration needed for ESP-IDF as esp_timer_get_time() is used
}

void print_task_runtime_stats(void)
{
    char *task_stats = (char *)malloc(1024);
    if (task_stats != NULL) {
        vTaskGetRunTimeStats(task_stats);
        ESP_LOGI(TAG_MAIN, "Task Runtime Stats:\n%s", task_stats);
        free(task_stats);
    } else {
        ESP_LOGE(TAG_MAIN, "Failed to allocate memory for task stats");
    }
}

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG_MAIN, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG_MAIN, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

static void blink_task(void *pvParameter)
{
    while (1) {

        // LED on phase
        s_led_state = 1;
        blink_led();
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // LED off phase based on mode:
        s_led_state = 0;
        blink_led();
        if (ota_mode) {
            vTaskDelay(5000 / portTICK_PERIOD_MS);  // OTA mode: 5-second cycle
            //ESP_LOGI(TAG_MAIN, "ota mode led  blink every 5 seconds");
        } else if (low_bat_mode) {
            vTaskDelay(29800 / portTICK_PERIOD_MS); // Low power mode: 30-second cycle
        } else {
            //ESP_LOGI(TAG_MAIN, "default mode led  blink every 1.5 seconds");
            vTaskDelay(1500 / portTICK_PERIOD_MS);   // Default mode: 1-second cycle
        }
        
    }
}

static void button_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG_MAIN, "Button timer callback triggered");
    int64_t press_duration = esp_timer_get_time() - button_press_time;
    ESP_LOGI(TAG_MAIN, "Press duration: %lld ms", press_duration / 1000);
    // No need to handle reset or OTA mode here
}

static void ota_task(void *pvParameter)
{
    // Initialize BLE controller and NimBLE stack
    ESP_LOGI(TAG_MAIN, "Initializing BLE controller and NimBLE stack...");
    nimble_port_init();

    // Register sync and reset callbacks
    ble_hs_cfg.sync_cb = sync_cb;
    ble_hs_cfg.reset_cb = reset_cb;

    // Initialize service table
    ESP_LOGI(TAG_MAIN, "Initializing service table...");
    gatt_svr_init();

    // Set device name and start host task
    ESP_LOGI(TAG_MAIN, "Setting device name and starting host task...");
    ble_svc_gap_device_name_set(device_name);
    nimble_port_freertos_init(host_task);

    // Wait for BLE connection with timeout
    ESP_LOGI(TAG_MAIN, "Waiting for BLE connection...");
    EventBits_t bits = xEventGroupWaitBits(ble_event_group, BLE_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(BLE_CONNECTION_TIMEOUT));
    if (bits & BLE_CONNECTED_BIT) {
        ESP_LOGI(TAG_MAIN, "BLE connected, proceeding with OTA...");

        #ifdef TMOS_SENSOR
            vTaskSuspend(sths34pf80_tmos_presence_detection_task_handle);
            disable_sths34pf80_interrupt_for_ota();
        #endif

        ESP_LOGI(TAG_MAIN, "Checking OTA bit status: 0x%x", (unsigned int)xEventGroupGetBits(ble_event_group));

        // Wait for OTA update request
        bits = xEventGroupWaitBits(ble_event_group, OTA_UPDATE_REQUEST_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(BLE_CONNECTION_TIMEOUT));
        if (bits & OTA_UPDATE_REQUEST_BIT) {
            ESP_LOGI(TAG_MAIN, "OTA update requested, waiting for completion...");

            // Wait for OTA update done with a timeout
            bits = xEventGroupWaitBits(ble_event_group, OTA_UPDATE_DONE_BIT|OTA_DISCONNECT_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(OTA_TRANSFER_TIMEOUT)); // 60-second timeout
            if (bits & OTA_UPDATE_DONE_BIT) {
                ESP_LOGI(TAG_MAIN, "OTA update completed, restarting...");
            } else if (bits & OTA_DISCONNECT_BIT) {
                ESP_LOGE(TAG_MAIN, "OTA failed due to BLE disconnection.");
            } else {
                ESP_LOGE(TAG_MAIN, "OTA update failed due to OTA_TRANSFER_TIMEOUT, restarting...");
            }        }
    } else {
        ESP_LOGE(TAG_MAIN, "BLE connection failed due to BLE_CONNECTION_TIMEOUT, aborting OTA...");
        ota_mode = false; // Clear OTA mode flag
    }
    // Reset the ota_task_started flag before deleting the task
    ota_task_started = false;
    ESP_LOGE(TAG_MAIN, "Finishing ota_task by esp restart");
    shutdown_ble_and_sleep();
    vTaskDelete(NULL);
}

// New button polling task function
static void button_task(void *pvParameter)
{
    while (1) {
        if (ota_mode) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        if (gpio_get_level(BUTTON_GPIO) == 0) {
            button_pressed = true;
            button_press_time = esp_timer_get_time();
            ESP_LOGI(TAG_MAIN, "Button pressed, starting timer");
            if (xTimerStart(button_timer, 0) != pdPASS) {
                ESP_LOGE(TAG_MAIN, "Failed to start button timer");
            }

            // Resume blinking so the user sees feedback
            //vTaskResume(blinkTaskHandle);

            while (gpio_get_level(BUTTON_GPIO) == 0) {
                int64_t press_duration = esp_timer_get_time() - button_press_time;
                if (press_duration >= OTA_MODE_THRESHOLD * 1000) {
                    ESP_LOGI(TAG_MAIN, "Button pressed for 5 seconds, entering OTA mode");
                    ota_mode = true; // Set OTA mode flag
 
                    // Create the event group for BLE connection
                    ble_event_group = xEventGroupCreate();

                    // Start the OTA task if not already started
                    if (!ota_task_started) {
                        ota_task_started = true;
                        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
                    }
                    break;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            // If button is released before reaching the OTA threshold, check if it was a short press for pairing.
            if (gpio_get_level(BUTTON_GPIO) != 0) {
                int64_t total_press_duration = esp_timer_get_time() - button_press_time;
                // Only trigger pairing if the press is longer than a minimal threshold, but shorter than OTA threshold
                if ((total_press_duration >= PAIRING_MODE_THRESHOLD * 1000) && (total_press_duration < OTA_MODE_THRESHOLD * 1000)) {
                ESP_LOGI(TAG_MAIN, "Button pressed for %lld ms, entering Pairing mode", total_press_duration / 1000);
                // Place your pairing mode code here, for example:
                // start_pairing();                                           
                } else if (total_press_duration < OTA_MODE_THRESHOLD * 1000) {
                    ESP_LOGI(TAG_MAIN, "Button released after %lld ms", total_press_duration / 1000);
                }
            }

            button_pressed = false;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// RTOS task wrapper for presence detection
static void tmos_presence_detection_task(void *pvParameters)
{
    
    if (ota_mode) {
        ESP_LOGI(TAG_MAIN, "tmos task is suspended for OTA mode");
        vTaskSuspend(NULL); // Suspend itself
    }
    // Call the presence detection function (will run indefinitely)
    sths34pf80_tmos_presence_detection();
    
    // This line will never be reached; delete task if needed.
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t err;
    esp_log_level_set("gpio", ESP_LOG_WARN); // Only show warnings and errors for GPIO
    esp_log_level_set("sleep", ESP_LOG_WARN); // Suppress INFO logs from app_init    
    esp_log_level_set("spi_flash", ESP_LOG_WARN); // Suppress INFO logs from app_init    
    esp_log_level_set("main_task", ESP_LOG_WARN); // Suppress INFO logs from app_init    




    esp_reset_reason_t reset_reason = esp_reset_reason();
    //ESP_LOGI(TAG_MAIN, "Reset reason: %d", reset_reason);

    // Create the button event group for light sleep handling
    app_event_group = xEventGroupCreate();
    if (app_event_group == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create app event group");
        return;
    }
    const esp_partition_t *partition = esp_ota_get_running_partition();

    switch (partition->address) {
      case 0x00010000:
        //ESP_LOGI(TAG_MAIN, "Running partition: factory");
        break;
      case 0x00110000:
        ESP_LOGI(TAG_MAIN, "Running partition: ota_0");
        break;
      case 0x00210000:
        ESP_LOGI(TAG_MAIN, "Running partition: ota_1");
        break;
  
      default:
        ESP_LOGE(TAG_MAIN, "Running partition: unknown");
        break;
    }
  
    // check if an OTA has been done, if so run "diagnostics" x implemented
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
      if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG_MAIN, "An OTA update has been detected.");
        if (run_diagnostics()) {
            ESP_LOGI(TAG_MAIN, "Diagnostics completed successfully! Continuing execution.");
            //wakeup_reason = ESP_SLEEP_WAKEUP_UNDEFINED; // Force first boot behavior
        
          esp_ota_mark_app_valid_cancel_rollback();
        } else {
          ESP_LOGE(TAG_MAIN,
                   "Diagnostics failed! Start rollback to the previous version.");
          esp_ota_mark_app_invalid_rollback_and_reboot();
        }
      }
    }

    // Initialize GPIO for RADAR_TD_GPIO
    err = initialize_gpio(RADAR_TD_GPIO, false, false, GPIO_INTR_DISABLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize RADAR_TD_GPIO");
        return;
    }

    // Configure the peripheral according to the LED type
    configure_led();
    
    button_timer = xTimerCreate("button_timer", pdMS_TO_TICKS(100), pdFALSE, (void *)0, button_timer_callback);

    // Instead, create the button polling task:
    xTaskCreate(&button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, &blinkTaskHandle);
 
    err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to install GPIO ISR service: %d", err);
        // Handle error
    }
    
    // Configure the bgt60ltr11aip interrupt GPIO pins
    //configure_radar_gpio();

    nvs_init();

    #ifdef TMOS_SENSOR
        // Initialize the I2C master bus
        err = i2c_master_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "I2C initialization failed: %s", esp_err_to_name(err));
            return;
        } else {
            ESP_LOGI(TAG_MAIN, "I2C initialized successfully");
        }

        // Initialize the sths34pf80 parameters
        sths34pf80_init();

        // Initialzie, create the tmos motion detection task
        xTaskCreate(tmos_presence_detection_task, "PresenceDetectionTask", 4096, NULL, 5, &sths34pf80_tmos_presence_detection_task_handle);

        configure_tmos_int();
    #endif

    xTaskCreate(battery_task, "BatteryTask", 2048, NULL, 3, &batteryTaskHandle);
    xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);

}

static void deep_sleep_task(void *args)
{
    EventBits_t bits;
    static bool first_boot_handled = false; // Static flag to track first boot handling


    while (1) {

        if (ota_mode) {
            //ESP_LOGI(TAG_MAIN, "OTA mode active, skipping deep sleep...");
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to avoid busy looping
            continue; // Skip the rest of the loop
        }

        // Wait for the button to be released before entering deep sleep
        while (button_pressed) {
            ESP_LOGI(TAG_MAIN, "Button is pressed, waiting for release...");
            vTaskDelay(500 / portTICK_PERIOD_MS); // Add a small delay to avoid busy looping
        }

        wakeup_reason = esp_sleep_get_wakeup_cause();
        //ESP_LOGI(TAG_MAIN, "Wakeup reason: %d", wakeup_reason);
        
        #ifndef DEEP_SLEEP_MODE
        if (!first_boot_handled) {
            wakeup_reason = ESP_SLEEP_WAKEUP_UNDEFINED; // Force first boot behavior
        } else {
            wakeup_reason = ESP_SLEEP_WAKEUP_UART; // Skip first boot behavior in subsequent loops
        }
        #endif

        switch (wakeup_reason) {
            case ESP_SLEEP_WAKEUP_UNDEFINED:
                //ESP_LOGI(TAG_MAIN, "Wakeup reason: Undefined (first boot or reset)");
                init_bgt60l();   
                init_cc1101(); // Initialize CC1101 after BGT60L
                //ESP_LOGI(TAG_MAIN, "Triggering CC1101 tramission in first boot...");
                xEventGroupSetBits(app_event_group, TRANSMIT_START_BIT);
                do {
                    bits = xEventGroupWaitBits(app_event_group, TRANSMIT_COMPLETE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
                } while (!(bits & TRANSMIT_COMPLETE_BIT)); // Keep waiting until the bit is set
                //ESP_LOGI(TAG_MAIN, "ESP_SLEEP_WAKEUP_UNDEFINED:Transmit task completed");
                #ifdef DEEP_SLEEP_MODE
                    cc1101_power_down();                                  
                #endif
                first_boot_handled = true; // Mark first boot as handled

                break;
            case ESP_SLEEP_WAKEUP_EXT1:
                //ESP_LOGI(TAG_MAIN, "Wakeup reason: External signal using EXT1");
                break;
            case ESP_SLEEP_WAKEUP_TIMER:
                //ESP_LOGI(TAG_MAIN, "Wakeup reason: Timer");
                // Initialize CC1101, not BGT60L
                init_cc1101();
                //ESP_LOGI(TAG_MAIN, "Triggering transmission...");
                xEventGroupSetBits(app_event_group, TRANSMIT_START_BIT);
          
                // Wait indefinitely for the TRANSMIT_COMPLETE_BIT
                do {
                    bits = xEventGroupWaitBits(app_event_group, TRANSMIT_COMPLETE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
                } while (!(bits & TRANSMIT_COMPLETE_BIT)); // Keep waiting until the bit is set
            
                ESP_LOGI(TAG_MAIN, "ESP_SLEEP_WAKEUP_TIMER:Transmit task completed");
                cc1101_power_down();                           
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                ESP_LOGI(TAG_MAIN, "Wakeup reason: GPIO");
                break;
            case ESP_SLEEP_WAKEUP_UART:
                ESP_LOGI(TAG_MAIN, "Wakeup reason: UART");
                //ESP_LOGI(TAG_MAIN, "Triggering transmission...");
                xEventGroupSetBits(app_event_group, TRANSMIT_START_BIT);
          
                // Wait indefinitely for the TRANSMIT_COMPLETE_BIT
                do {
                    bits = xEventGroupWaitBits(app_event_group, TRANSMIT_COMPLETE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
                } while (!(bits & TRANSMIT_COMPLETE_BIT)); // Keep waiting until the bit is set
            
                //ESP_LOGI(TAG_MAIN, "ESP_SLEEP_WAKEUP_TIMER:Transmit task completed");
                break;
            default:
                ESP_LOGI(TAG_MAIN, "Wakeup reason: Other (%d)", wakeup_reason);
                break;
        }



        // Skip deep sleep logic if OTA mode is active
        if (ota_mode) {
            ESP_LOGI(TAG_MAIN, "OTA mode active, skipping deep sleep...");
            continue; // Skip the rest of the loop
        }        

        // Check if the button is pressed   
        //ESP_LOGI(TAG_MAIN, "Preparing to enter deep sleep...");

        //cc1101_power_down();
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

        #ifdef DEEP_SLEEP_MODE
        /* Configure wakeup sources */
        esp_sleep_enable_ext1_wakeup((1ULL << BUTTON_GPIO) , ESP_EXT1_WAKEUP_ANY_LOW);

        /* Enter deep sleep */
        ESP_LOGI(TAG_MAIN, "Entering deep sleep...");
        s_led_state = 0;
        blink_led();
        esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);  // 60s deep sleep
        esp_deep_sleep_start();
        #else

        //ESP_LOGI(TAG_MAIN, "Debug mode active, skipping deep sleep...");
        vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60 seconds
        //ESP_LOGI(TAG_MAIN, "After delay...");

        #endif
        
    }
}
