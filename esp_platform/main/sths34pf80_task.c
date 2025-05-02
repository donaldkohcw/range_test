#include "driver/i2c.h"
#include "esp_log.h"
#include "sths34pf80_task.h"
#include "sths34pf80/sths34pf80_reg.h"

#define STHS34PF80_POLLING
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 22      // SCL on GPIO1
#define I2C_MASTER_SDA_IO 23      // SDA on GPIO3
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TIMEOUT_MS 1000 // Define the timeout in milliseconds for I2C transactions
#define BOOT_TIME 10 //ms
#define TMOS_INT_GPIO  10  // Changed to GPIO30 for the Interrupt pin

static const char *TAG_I2C = "I2C_STHS34PF80";
static uint8_t tx_buffer[1000];
static uint8_t sensor_addr_global = 0x5A;  // Fixed 7-bit I²C address for the sensor
static stmdev_ctx_t dev_ctx_global;

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C driver install failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_I2C, "I2C initialized on SCL GPIO %d and SDA GPIO %d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    }
    return ret;
}

// Function to force SCL high
esp_err_t force_scl_high(void)
{
    // Reset and configure the SCL pin as an output
    gpio_reset_pin(I2C_MASTER_SCL_IO);
    esp_err_t ret = gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_set_level(I2C_MASTER_SCL_IO, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG_I2C, "Forced SCL high on GPIO %d", I2C_MASTER_SCL_IO);
    return ESP_OK;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    esp_err_t ret;
    uint8_t dev_addr = *((uint8_t *)handle);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG_I2C, "Failed to create I2C command link");
        return -1;
    }
    i2c_master_start(cmd);
    // Write device address with write bit and register address
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    // Restart for read
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    // Read bytes; send ACK for all but the last byte
    if (len > 1) {
        i2c_master_read(cmd, bufp, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, bufp + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        return 0;
    } else {
        ESP_LOGE(TAG_I2C, "platform_read error: %s", esp_err_to_name(ret));
        return ret;
    }
}

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    esp_err_t ret;
    // Assume handle points to the 7-bit device address
    uint8_t dev_addr = *((uint8_t *)handle);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG_I2C, "Failed to create I2C command link");
        return -1;
    }
    i2c_master_start(cmd);
    // Write device address with write bit
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    // Write register address
    i2c_master_write_byte(cmd, reg, true);
    // Write data buffer
    if (len > 0) {
        i2c_master_write(cmd, (uint8_t *)bufp, len, true);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        return 0;
    } else {
        ESP_LOGE(TAG_I2C, "platform_write error: %s", esp_err_to_name(ret));
        return ret;
    }
}

void platform_delay(uint32_t millisec)
{
    vTaskDelay(millisec / portTICK_PERIOD_MS);
}


void sths34pf80_tmos_presence_detection(void)
{

    #ifdef STHS34PF80_POLLING
   
    /* Main loop: wait for presence/motion events, likely signaled by an interrupt */
    while (1)
    {
        sths34pf80_func_status_t func_status;
        uint8_t motion = 0;
        uint8_t presence = 0;

        /* Assume wakeup_thread is set by an interrupt handler elsewhere */
        //extern volatile uint8_t wakeup_thread;
        //if (wakeup_thread)
        //{
            //wakeup_thread = 0;
            motion = 0;
            presence = 0;

            do
            {
                sths34pf80_func_status_get(&dev_ctx_global, &func_status);
                #ifdef STHS34PF80_PRESENCE
                    if (func_status.pres_flag != presence)
                    {
                        presence = func_status.pres_flag;
                        if (presence)
                        {
                            snprintf((char *)tx_buffer, sizeof(tx_buffer), "Start of Presence\r\n");
                            ESP_LOGI(TAG_I2C, "%s", tx_buffer);
                            read_and_print_ambient();
                        }
                        else
                        {
                            snprintf((char *)tx_buffer, sizeof(tx_buffer), "End of Presence\r\n");
                            ESP_LOGI(TAG_I2C, "%s", tx_buffer);
                        }
                    }
                #endif

                if (func_status.mot_flag != motion)
                {
                    motion = func_status.mot_flag;
                    if (motion)
                    {
                        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Polling Motion flag changed!\r\n");
                        ESP_LOGI(TAG_I2C, "%s", tx_buffer);
                        read_and_print_ambient();                        
                    } 

                }
                // Add a delay to avoid excessive polling
                vTaskDelay(200 / portTICK_PERIOD_MS);
            } while (func_status.pres_flag);
        //}
    }
    #endif /* STHS34PF80_POLLING */
    vTaskDelete(NULL);
}

void read_and_print_ambient(void)
{

    // Variable to store the raw ambient temperature reading.
    int16_t ambient_raw = 0;

    // Call the function and check for success.
    if (sths34pf80_tambient_raw_get(&dev_ctx_global, &ambient_raw) == 0)
    {
        // Convert the raw value (LSB) to ambient temperature in °C.
        // According to the datasheet: TAMBIENT °C = Ambient Raw Value / 100 LSB/°C.
        float ambient_temp = (float)ambient_raw / 100.0f;
        ESP_LOGI("TEMP", "Ambient Temperature: %.2f °C", ambient_temp);
    }
    else
    {
        ESP_LOGE(TAG_I2C, "Failed to read ambient raw value");
    }
}

// Separate handler for TMOS_INT_GPIO (GPIO 0)
static void IRAM_ATTR tmos_int_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    int level = gpio_get_level(gpio_num);
    ESP_EARLY_LOGI(TAG_I2C, "TMOS interrupt on GPIO %d, level: %d", gpio_num, level);
    // TMOS-specific handling...
}

// Function to configure the radar GPIOs
void configure_tmos_int(void)
{

    gpio_config_t io_conf = {0};

    // Reconfigure bgt's miso to tmos's int pin
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising and falling edges
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TMOS_INT_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    ESP_ERROR_CHECK(gpio_isr_handler_add(TMOS_INT_GPIO, tmos_int_handler, (void*) TMOS_INT_GPIO));

    ESP_LOGI(TAG_I2C, "Motion interrupt GPIOs configured: tmos pin %d", TMOS_INT_GPIO);


}

void sensor_init_parameters(stmdev_ctx_t *dev_ctx)
{
    uint8_t whoami;
    sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;

    /* Wait for sensor boot time */
    platform_delay(BOOT_TIME);

    /* Check device ID */
    sths34pf80_device_id_get(dev_ctx, &whoami);
    ESP_LOGI(TAG_I2C, "Sensor ID returned: 0x%02X", whoami);
    if (whoami != STHS34PF80_ID)
    {
        // Device not found, hang here
        while (1);
    }

    /* Set averages: AVG_TMOS = 32 and AVG_TAMB = 8 */
    sths34pf80_avg_tobject_num_set(dev_ctx, STHS34PF80_AVG_TMOS_32);
    sths34pf80_avg_tambient_num_set(dev_ctx, STHS34PF80_AVG_T_8);

    /* Read current filter settings */
    sths34pf80_lpf_m_bandwidth_get(dev_ctx, &lpf_m);
    sths34pf80_lpf_p_bandwidth_get(dev_ctx, &lpf_p);
    sths34pf80_lpf_p_m_bandwidth_get(dev_ctx, &lpf_p_m);
    sths34pf80_lpf_a_t_bandwidth_get(dev_ctx, &lpf_a_t);

    snprintf((char *)tx_buffer, sizeof(tx_buffer),
             "lpf_m: %02d, lpf_p: %02d, lpf_p_m: %02d, lpf_a_t: %02d\r\n",
             lpf_m, lpf_p, lpf_p_m, lpf_a_t);
    ESP_LOGI(TAG_I2C, "%s", tx_buffer);

    /* Enable Block Data Update */
    sths34pf80_block_data_update_set(dev_ctx, 1);

    /* Set thresholds and hysteresis */
    sths34pf80_presence_threshold_set(dev_ctx, 150);
    sths34pf80_presence_hysteresis_set(dev_ctx, 20);
    sths34pf80_motion_threshold_set(dev_ctx, 150);
    sths34pf80_motion_hysteresis_set(dev_ctx, 20);

    /* Reset the sensor algorithm */
    sths34pf80_algo_reset(dev_ctx);

    /* Configure interrupts for presence detection */
    sths34pf80_int_or_set(dev_ctx, STHS34PF80_INT_MOTION);
    sths34pf80_route_int_set(dev_ctx, STHS34PF80_INT_OR);

    /* Set Output Data Rate (ODR) to 30Hz */
    sths34pf80_odr_set(dev_ctx, STHS34PF80_ODR_AT_30Hz);
}

void sths34pf80_init(void)
{
    // Set up the device context.
    dev_ctx_global.handle    = &sensor_addr_global;
    dev_ctx_global.write_reg = platform_write;
    dev_ctx_global.read_reg  = platform_read;
    dev_ctx_global.mdelay    = platform_delay;

    // Call the separate initialization function.
    sensor_init_parameters(&dev_ctx_global);

    // Optionally, you can perform additional initialization or store dev_ctx globally.
}

void disable_sths34pf80_interrupt_for_ota(void) {
    // Disable TMOS interrupt
    ESP_ERROR_CHECK(gpio_isr_handler_remove(TMOS_INT_GPIO));
    ESP_LOGI(TAG_I2C, "TMOS interrupt disabled for OTA mode");
}