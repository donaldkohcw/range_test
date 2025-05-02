#define TRANSMIT
#include "cc1101.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "driver/gpio.h"
#include "esp_system.h"  // for esp_random()
#include "spi_task.h"
#include "ota_event_bits.h" // Include the common header file
#include "bgt60ltr11aip.h"
#include "app_event_group.h"


static const char *TAG_CC1101 = "cc1101";
static volatile bool gdo0_triggered = false;

// Declare the internal helper function prototype as static
static void set_rf_mode(rf_mode_t new_rf_mode);
static void receiverOn(void);
static void setIdle(void);
static void pollTask(void *pvParameters);
static void transmitTask(void *pvParameters);

static int64_t lastTxTime;
static rf_tx_message_union gTxMsgQueue[RF_TX_MSG_QUEUE_SIZE];
// When the circular buffer is empty, head=tail.
// When head+1=tail, that means it is full.
static uint8_t gTxMsgQueueHead, gTxMsgQueueTail;
static rf_rx_packetData_t gRxMsgQueue[RF_RX_MSG_QUEUE_SIZE];
static uint8_t gRxMsgQueueHead, gRxMsgQueueTail;
static uint16_t saved_sync_word = 0;

// Declare the SPI handle and current RF mode as static globals
static spi_device_handle_t rf_spi = NULL;
static rf_mode_t current_rf_mode =RF_MODE_TX_AND_RX;
static rf_mode_t selected_rf_mode = RF_MODE_TX_AND_RX;

// Define the default RF settings (update these values according to your design)
static const RF_SETTINGS defaultRF_Settings = {
    .fsctrl1   = 0x06,
    .fsctrl0   = 0x00,
    .freq2     = 0x10,
    .freq1     = 0xB1,
    .freq0     = 0x3B,
    .mdmcfg4   = 0xCA,
    .mdmcfg3   = 0x83,
    .mdmcfg2   = 0x93,
    .mdmcfg1   = 0x22,
    .mdmcfg0   = 0xF8,
    .channr    = 0x00,
    .deviatn   = 0x35,
    .frend1    = 0x56,
    .frend0    = 0x10,
    .mcsm1     = 0x0F,
    .mcsm0     = 0x18,
    .foccfg    = 0x16,
    .bscfg     = 0x6C,
    .agcctrl2  = 0x43,
    .agcctrl1  = 0x60,
    //.agcctrl1  = 0x40,    
    .agcctrl0  = 0x91,
    .fscal3    = 0xE9,
    .fscal2    = 0x2A,
    .fscal1    = 0x00,
    .fscal0    = 0x1F,
    .fstest    = 0x59,
    .test2     = 0x81,
    .test1     = 0x35,
    .test0     = 0x09,
    //.fifothr   = 0x47,
    .fifothr   = 0x67,    
    .iocfg2    = 0x29,
    .iocfg0    = 0x06,
    .pktctrl1  = 0x04,
    .pktctrl0  = 0x05,
    .addr      = 0x00,
    .pktlen    = CC1101_MAX_PACKET_LEN
};

//static const char *TAG_CC1101 = "cc1101";

// Example strobe command for reset (SRES). Refer to your datasheet for details.

/**
 * Status registers
 * 
 *
 */
/* @brief Set the selected transmission mode, TX only (for battery device i.e. QTS or QGS ) or TX and RX (for powered device and QGDS)*/

// Define your interrupt service routine
static void IRAM_ATTR cc1101_gdo0_isr(void* arg)
{
    // Minimal ISR processing here.
    // For example, notify a task or set a flag.

    
//    ESP_LOGI(TAG_CC1101, "In the interrupt handler");

gdo0_triggered = true;

}

/*
 * @brief Set CC1101 into idle mode (turn off receiver)
 */
static void setIdle()
{
    cc1101_strobe(rf_spi, CC1101_SIDLE);
    cc1101_strobe(rf_spi, CC1101_SFRX);
}


void rf_set_mode(rf_mode_t new_rf_mode)
{
   selected_rf_mode = new_rf_mode;
   set_rf_mode(new_rf_mode);
}

/* @brief Set CC1101 into receive mode
*/
static void receiverOn()
{
   cc1101_strobe (rf_spi, CC1101_SRX);
}

/*
 * This static function configures the chip for the given RF mode.
 * It first forces the chip to idle and then builds a new MCSM1 value.
 * If the mode requires RX to be turned on, it calls receiverOn.
 */
static void set_rf_mode(rf_mode_t new_rf_mode)
{
    uint8_t new_mcsm1_value;

    // Force the chip to go idle and flush the RX FIFO.
    setIdle();

    // Initialize new_mcsm1_value with the always-clear and stay-RX bits.
    new_mcsm1_value = CCA_MODE_ALWAYS_CLEAR | RXOFF_MODE_STAY_RX;
    if (new_rf_mode == RF_MODE_TX_ONLY)
    {
        new_mcsm1_value |= TXOFF_MODE_IDLE;
    }
    else
    {
        new_mcsm1_value |= TXOFF_MODE_RX;
    }
    cc1101_writeSingleRegister(rf_spi, CC1101_MCSM1, new_mcsm1_value);

    // If the mode is TX and RX, turn the receiver on.
    if (new_rf_mode == RF_MODE_TX_AND_RX)
    {
        receiverOn();
    }
    current_rf_mode = new_rf_mode;
}

static spi_device_handle_t cc1101_spi_handle;
TaskHandle_t transmitTaskHandle = NULL; // Task handle for transmitTask

esp_err_t cc1101_init(void)
{

    esp_err_t err;
    esp_err_t ret;

    spi_device_interface_config_t devcfg_cc1101 = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = CONFIG_SPI_CS_GPIO, // Defined above
        .queue_size = 3,
    };

    spi_device_handle_t cc1101_spi_handle;
    ret = spi_bus_add_device(CONFIG_SPI_HOST, &devcfg_cc1101, &cc1101_spi_handle);

     if (ret != ESP_OK) {
        ESP_LOGE(TAG_CC1101, "Failed to add CC1101 SPI device: %d", ret);
        vTaskDelete(NULL);
     } else {
        //ESP_LOGI(TAG_CC1101, "CC1101 SPI initialized using CS GPIO %d", CONFIG_SPI_CS_GPIO);
     }

     rf_spi = cc1101_spi_handle; // save SPI handle

    //ESP_LOGI(TAG_CC1101, "Initializing CC1101");

    // Configure GPIO10 for CC1101 GDO0 interrupt
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,  // Choose trigger edge as needed
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << 3),
        .pull_up_en = GPIO_PULLUP_DISABLE,    // Enable pull-up if required
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    
    err = gpio_isr_handler_add(3, cc1101_gdo0_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_CC1101, "Failed to add GPIO ISR handler: %d", err);
        // Handle error
    } else {
        //ESP_LOGI(TAG_CC1101, "GPIO ISR handler added successfully!");
    }

    gpio_config(&io_conf);

    cc1101_wake_up();
    
    uint8_t marcstate = 0;
    cc1101_readBurstRegister(CC1101_MARCSTATE, &marcstate, 1);
    //ESP_LOGI(TAG_CC1101, "CC1101 MARCSTATE (via burst): 0x%02X", marcstate);

     uint8_t partnum = 0;
     cc1101_readBurstRegister(CC1101_PARTNUM, &partnum, 1);
     //ESP_LOGI(TAG_CC1101, "CC1101 Part Number (via burst): 0x%02X", partnum);
    // Delay for the reset to take effect - increased from 1ms to 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t version = 0;
    cc1101_readBurstRegister(CC1101_VERSION, &version, 1);
    //ESP_LOGI(TAG_CC1101, "CC1101 Version (via burst): 0x%02X", version);

    // Call to write all configuration registers from the default RF settings
    cc1101_writeRegistersSetting(rf_spi, &defaultRF_Settings);

    // Write PATABLE and sync word registers for proper operation
    cc1101_writeSingleRegister(rf_spi, CC1101_PATABLE, PATABLE_VAL);
    cc1101_writeSingleRegister(rf_spi, CC1101_SYNC1, (uint8_t)(defaultSyncWord >> 8));
    cc1101_writeSingleRegister(rf_spi, CC1101_SYNC0, (uint8_t)defaultSyncWord);

    // Optionally, set an initial RF mode here:
    rf_set_mode(RF_MODE_TX_AND_RX);

    saved_sync_word = 0;
    lastTxTime = esp_timer_get_time();

    memset(gTxMsgQueue, 0, sizeof(gTxMsgQueue));
    gTxMsgQueueHead = 0;
    gTxMsgQueueTail = 0;
    memset(gRxMsgQueue, 0, sizeof(gRxMsgQueue));
    gRxMsgQueueHead = 0;
    gRxMsgQueueTail = 0;

    //Dump configuration registers from 0x00 to 0x2E for comparison
    // ESP_LOGI(TAG_CC1101, "Dumping configuration registers:");
    // for (uint8_t reg = 0x00; reg <= 0x2E; reg++) {
    //     uint8_t value = 0;
    //     ret = cc1101_readSingleRegister(rf_spi, reg, &value);
    //     if (ret == ESP_OK) {
    //         ESP_LOGI(TAG_CC1101, "Reg 0x%02X: 0x%02X", reg, value);
    //     } else {
    //         ESP_LOGE(TAG_CC1101, "Failed to read reg 0x%02X", reg);
    //     }
    //     // Optional small delay between reads
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
  

    // Delay for the reset to take effect - increased from 1ms to 100ms
    vTaskDelay(pdMS_TO_TICKS(100));
    gdo0_triggered = false; // Clear the flag

#ifdef TRANSMIT
    // Update the xTaskCreate call to store the task handle
    xTaskCreate(transmitTask, "ccTransmitTask", 2048, NULL, tskIDLE_PRIORITY + 1, &transmitTaskHandle);
#else
    gdo0_triggered = false; // Clear the flag      
    // Create a FreeRTOS task to poll for data every 3 seconds.
    xTaskCreate(pollTask, "ccPollTask", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
#endif
        
    return ESP_OK;
}

esp_err_t cc1101_write_reg(spi_device_handle_t spi, uint8_t addr, uint8_t value)
{
    // For a write, send address and value (as two bytes).
    uint8_t tx_buf[2] = { addr, value };

    spi_transaction_t t = {
        .length = 16,         // 2 bytes = 16 bits
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };

    return spi_device_transmit(spi, &t);
}

esp_err_t cc1101_readSingleRegister(spi_device_handle_t spi, uint8_t addr, uint8_t *value)
{
    // For a read, set the read flag (typically setting MSB, see datasheet)
    uint8_t tx_buf[2] = { addr | 0x80, 0x00 };
    uint8_t rx_buf[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,         // 2 bytes = 16 bits
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret == ESP_OK && value) {
        *value = rx_buf[1];
    }
    return ret;
}

esp_err_t cc1101_strobe(spi_device_handle_t spi, uint8_t strobe)
{
    spi_transaction_t t = {
       .length = 8,  // 1 byte = 8 bits
       .tx_buffer = &strobe,
       .rx_buffer = NULL,
    };

    return spi_device_transmit(spi, &t);
}

/* 
 * Write a single register. This is a simple wrapper around cc1101_write_reg.
 * You can use it to write configuration registers one at a time.
 */
esp_err_t cc1101_writeSingleRegister(spi_device_handle_t spi, uint8_t addr, uint8_t value)
{
    return cc1101_write_reg(spi, addr, value);
}

/*
 * Write the RF settings from an RF_SETTINGS structure.
 * This function writes each configuration register one by one.
 */
void cc1101_writeRegistersSetting(spi_device_handle_t spi, RF_SETTINGS *pRfSettings)
{
    cc1101_writeSingleRegister(spi, CC1101_FSCTRL1,   pRfSettings->fsctrl1);
    cc1101_writeSingleRegister(spi, CC1101_FSCTRL0,   pRfSettings->fsctrl0);
    cc1101_writeSingleRegister(spi, CC1101_FREQ2,     pRfSettings->freq2);
    cc1101_writeSingleRegister(spi, CC1101_FREQ1,     pRfSettings->freq1);
    cc1101_writeSingleRegister(spi, CC1101_FREQ0,     pRfSettings->freq0);
    cc1101_writeSingleRegister(spi, CC1101_MDMCFG4,   pRfSettings->mdmcfg4);
    cc1101_writeSingleRegister(spi, CC1101_MDMCFG3,   pRfSettings->mdmcfg3);
    cc1101_writeSingleRegister(spi, CC1101_MDMCFG2,   pRfSettings->mdmcfg2);
    cc1101_writeSingleRegister(spi, CC1101_MDMCFG1,   pRfSettings->mdmcfg1);
    cc1101_writeSingleRegister(spi, CC1101_MDMCFG0,   pRfSettings->mdmcfg0);
    cc1101_writeSingleRegister(spi, CC1101_CHANNR,    pRfSettings->channr);
    cc1101_writeSingleRegister(spi, CC1101_DEVIATN,   pRfSettings->deviatn);
    cc1101_writeSingleRegister(spi, CC1101_FREND1,    pRfSettings->frend1);
    cc1101_writeSingleRegister(spi, CC1101_FREND0,    pRfSettings->frend0);
    cc1101_writeSingleRegister(spi, CC1101_MCSM1,     pRfSettings->mcsm1);
    cc1101_writeSingleRegister(spi, CC1101_MCSM0,     pRfSettings->mcsm0);
    cc1101_writeSingleRegister(spi, CC1101_FOCCFG,    pRfSettings->foccfg);
    cc1101_writeSingleRegister(spi, CC1101_BSCFG,     pRfSettings->bscfg);
    cc1101_writeSingleRegister(spi, CC1101_AGCCTRL2,  pRfSettings->agcctrl2);
    cc1101_writeSingleRegister(spi, CC1101_AGCCTRL1,  pRfSettings->agcctrl1);
    cc1101_writeSingleRegister(spi, CC1101_AGCCTRL0,  pRfSettings->agcctrl0);
    cc1101_writeSingleRegister(spi, CC1101_FSCAL3,    pRfSettings->fscal3);
    cc1101_writeSingleRegister(spi, CC1101_FSCAL2,    pRfSettings->fscal2);
    cc1101_writeSingleRegister(spi, CC1101_FSCAL1,    pRfSettings->fscal1);
    cc1101_writeSingleRegister(spi, CC1101_FSCAL0,    pRfSettings->fscal0);
    cc1101_writeSingleRegister(spi, CC1101_FSTEST,    pRfSettings->fstest);
    cc1101_writeSingleRegister(spi, CC1101_TEST2,     pRfSettings->test2);
    cc1101_writeSingleRegister(spi, CC1101_TEST1,     pRfSettings->test1);
    cc1101_writeSingleRegister(spi, CC1101_TEST0,     pRfSettings->test0);
    cc1101_writeSingleRegister(spi, CC1101_FIFOTHR,   pRfSettings->fifothr);
    cc1101_writeSingleRegister(spi, CC1101_IOCFG2,    pRfSettings->iocfg2);
    cc1101_writeSingleRegister(spi, CC1101_IOCFG0,    pRfSettings->iocfg0);
    cc1101_writeSingleRegister(spi, CC1101_PKTCTRL1,  pRfSettings->pktctrl1);
    cc1101_writeSingleRegister(spi, CC1101_PKTCTRL0,  pRfSettings->pktctrl0);
    cc1101_writeSingleRegister(spi, CC1101_ADDR,      pRfSettings->addr);
    cc1101_writeSingleRegister(spi, CC1101_PKTLEN,    pRfSettings->pktlen);
}

esp_err_t cc1101_reset(spi_device_handle_t spi)
{
    esp_err_t ret = cc1101_strobe(spi, CC1101_SRES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CC1101, "Failed to send reset strobe");
        return ret;
    }
    // Delay to allow the chip to settle after reset.
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

// Transmit function: writes bytes to TX FIFO, starts transmission, checks MARCSTATE, then prints the data.
esp_err_t cc1101_transmitData(const uint8_t *data, size_t len)
{
    esp_err_t ret;
    
    ESP_LOG_BUFFER_HEXDUMP(TAG_CC1101, data, (len), ESP_LOG_INFO);  
    //ESP_LOGI(TAG_CC1101, "Transmitting data (%d bytes)", (len));
     // Flush TX FIFO before writing new data
     ret = cc1101_strobe(rf_spi, CC1101_SIDLE);
     if(ret != ESP_OK) {
         return ret;
     }

    // Check that TX FIFO is clear before flushing it
    {
        uint8_t tx_bytes = 0;
        do {
//            ret = cc1101_readSingleRegister(rf_spi, CC1101_TXBYTES | READ_BURST, &tx_bytes);
            ret = cc1101_readBurstRegister(CC1101_TXBYTES, &tx_bytes, 1);

            if(ret != ESP_OK) {
                ESP_LOGE(TAG_CC1101, "Failed to read TXBYTES");
                return ret;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        } while(tx_bytes != 0);
        //ESP_LOGI(TAG_CC1101, "TX FIFO is clear");
    }
    
    // Flush TX FIFO before writing new data
    ret = cc1101_strobe(rf_spi, CC1101_SFTX);
    if(ret != ESP_OK) {
        return ret;
    }
    // Log the payload length as a uint8_t
    //ESP_LOGI(TAG_CC1101, "Payload length (uint8_t)len: %d", (uint8_t)len);

    // Write the payload length as the first byte in the TX FIFO
    ret = cc1101_writeSingleRegister(rf_spi, CC1101_TXFIFO, (uint8_t)len);
    if(ret != ESP_OK) {
        return ret;
    }
    
    cc1101_writeBurstRegister(CC1101_TXFIFO, (uint8_t *)data, (uint8_t)len);
    
    // Issue the TX strobe to start transmission
    ret = cc1101_strobe(rf_spi, CC1101_STX);
    if(ret != ESP_OK) {
        return ret;
    }
    
    // Check the MARCSTATE after issuing TX strobe
    {
        uint8_t marcstate = 0;
        //ret = cc1101_readSingleRegister(rf_spi, (CC1101_MARCSTATE | READ_BURST), &marcstate);
        ret = cc1101_readBurstRegister(CC1101_MARCSTATE, &marcstate, 1);

        // if(ret == ESP_OK) {
        //     ESP_LOGI(TAG_CC1101, "Transmit MARCSTATE: 0x%02X", marcstate);
        // } else {
        //     ESP_LOGE(TAG_CC1101, "Failed to read MARCSTATE after TX strobe");
        // }
    }
    
    return ESP_OK;
}

// Receive function: flushes RX FIFO, checks MARCSTATE, polls for data then prints the received hex dump.
esp_err_t cc1101_receiveData(uint8_t *buffer, size_t max_len, size_t *received_len)
{
    esp_err_t ret;
    
    // Flush the RX FIFO before starting
    ret = cc1101_strobe(rf_spi, CC1101_SFRX);
    if(ret != ESP_OK) {
        return ret;
    }
    
    // Check the MARCSTATE after flushing RX FIFO
    {
        uint8_t marcstate = 0;
        //ret = cc1101_readSingleRegister(rf_spi, (CC1101_MARCSTATE | READ_BURST), &marcstate);
        ret = cc1101_readBurstRegister(CC1101_MARCSTATE, &marcstate, 1);

        if(ret == ESP_OK) {
            ESP_LOGI(TAG_CC1101, "Receive MARCSTATE after flush: 0x%02X", marcstate);
        } else {
            ESP_LOGE(TAG_CC1101, "Failed to read MARCSTATE after RX flush");
        }
    }
    
    const TickType_t timeout = pdMS_TO_TICKS(500);
    TickType_t start_time = xTaskGetTickCount();
    *received_len = 0;
    
    // Poll for data - this simplified example assumes a nonzero value indicates available data.
    while ((xTaskGetTickCount() - start_time) < timeout) {
        uint8_t byte;
        ret = cc1101_readSingleRegister(rf_spi, CC1101_RXFIFO, &byte);
        if (ret == ESP_OK && byte != 0) {
            size_t idx = 0;
            buffer[idx++] = byte;
            
            // Read subsequent bytes until FIFO is empty or max_len is reached.
            while (idx < max_len) {
                ret = cc1101_readSingleRegister(rf_spi, CC1101_RXFIFO, &byte);
                if (ret != ESP_OK || byte == 0) {
                    break;
                }
                buffer[idx++] = byte;
            }
            
            *received_len = idx;
            ESP_LOGI(TAG_CC1101, "Received data (%d bytes)", idx);
            ESP_LOG_BUFFER_HEXDUMP(TAG_CC1101, buffer, idx, ESP_LOG_INFO);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGW(TAG_CC1101, "Receive timeout: no data received");
    return ESP_ERR_TIMEOUT;
}

esp_err_t cc1101_readBurstRegister(uint8_t startAddress, uint8_t *buffer, uint8_t length)
{
    // Create TX and RX buffers; +1 for the status byte.
    uint8_t tx_buf[length + 1];
    uint8_t rx_buf[length + 1];

    // Set the burst read command (READ_BURST flag should be defined appropriately in cc1101.h)
    tx_buf[0] = startAddress | READ_BURST;
    memset(&tx_buf[1], 0, length);  // Fill remaining bytes with zero

    spi_transaction_t t = {
        .length = (length + 1) * 8,   // Total number of bits
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_transmit(rf_spi, &t);

    if (ret == ESP_OK) {
        // Copy the burst read data (excluding the status byte) to the caller's buffer
        memcpy(buffer, &rx_buf[1], length);
    } else {
        ESP_LOGE(TAG_CC1101, "Burst register read failed with error: %d", ret);
    }
    return ret;
}

void cc1101_writeBurstRegister(uint8_t startAddress, uint8_t *buffer, uint8_t length)
{
    // Create TX buffer with space for the command byte plus data bytes.
    uint8_t tx_buf[length + 1];

    // Set the burst write command (WRITE_BURST flag should be defined in cc1101.h)
    tx_buf[0] = startAddress | WRITE_BURST;
    memcpy(&tx_buf[1], buffer, length);

    spi_transaction_t t = {
        .length = (length + 1) * 8,   // Total number of bits to transfer.
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };

    if (spi_device_transmit(rf_spi, &t) != ESP_OK) {
        ESP_LOGE(TAG_CC1101, "Burst register write failed");
    }
}

static void pollTask(void *pvParameters)
{
   //int8_t rxBuffer[RF_BUFFER_SIZE];
    size_t rxLen = 0;
    uint8_t available = 0;  
    uint8_t *rxBuffer = NULL;  // Declare outside

    ESP_LOGI(TAG_CC1101, "Poll task created.");


    while (1) 
    {

        if (gdo0_triggered)
        {
            gdo0_triggered = false; // Clear the flag
            ESP_LOGI(TAG_CC1101, "Interrupt occurred: flag was set.");
            // In your code, for example inside a task or function:
            uint8_t rxBytes = 0;
            //esp_err_t ret = cc1101_read_burst_reg(rf_spi, CC1101_RXBYTES, &rxBytes);
            esp_err_t ret = cc1101_readBurstRegister(CC1101_RXBYTES, &rxBytes, 1);
            if(ret == ESP_OK) {
                available = rxBytes & 0x7F;
                if (available == 64) {
                    cc1101_strobe(rf_spi, CC1101_SIDLE);           
                    ESP_LOGW(TAG_CC1101, "RX FIFO is full");
                    cc1101_strobe(rf_spi, CC1101_SFRX);
                    receiverOn();
                    available = 0;
                    continue;
                }
                ESP_LOGI(TAG_CC1101, "RX FIFO contains %d bytes", available);

            }  else {
                ESP_LOGE(TAG_CC1101, "Failed to read RX bytes count");
            }

            if (available > 0) {

                vTaskDelay(pdMS_TO_TICKS(250));

                
                //memset(rxBuffer, 0, sizeof(rxBuffer));
                //available = rxBytes & 0x7F;
                uint8_t payload_length = 0;
                ret = cc1101_readSingleRegister(rf_spi, CC1101_RXFIFO, &payload_length);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG_CC1101, "Error reading payload length");
                    continue;
                }
                ESP_LOGI(TAG_CC1101, "Payload length: %d", payload_length);
            

                //ESP_LOGI(TAG_CC1101, "Received data (%d bytes):", available);                
                rxBuffer = malloc(payload_length);

                // Read the payload bytes from RX FIFO into the allocated buffer

                ret = cc1101_readBurstRegister(CC1101_RXFIFO, rxBuffer, payload_length);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG_CC1101, "Error reading burst from RX FIFO");
                    free(rxBuffer);
                    continue;
                }

                //Now read the two appended bytes: RSSI and status
                uint8_t rssi = 0, status = 0;
                ret = cc1101_readSingleRegister(rf_spi, CC1101_RXFIFO, &rssi);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG_CC1101, "Error reading RSSI byte");
                    free(rxBuffer);
                    continue;
                }
                ret = cc1101_readSingleRegister(rf_spi, CC1101_RXFIFO, &status);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG_CC1101, "Error reading status byte");
                    free(rxBuffer);
                    continue;
                }
            
                //sESP_LOGI(TAG_CC1101, "RSSI: 0x%02X, Status: 0x%02X", rssi, status);
                bool crc_ok = (status & 0x80) != 0;
                uint8_t lqi = status & 0x7F;
                ESP_LOGI(TAG_CC1101, "RSSI: 0x%02X, CRC_OK: %s, LQI: %d", rssi, crc_ok ? "true" : "false", lqi);

                ESP_LOG_BUFFER_HEXDUMP(TAG_CC1101, rxBuffer, payload_length, ESP_LOG_INFO);
                receiverOn();
            }
        }
        //receiverOn();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void transmitTask(void *pvParameters)
{
    const uint8_t *txData; 

    while (1) {

        // Wait for the TRANSMIT_START_BIT to be set
        EventBits_t bits = xEventGroupWaitBits(
            app_event_group,
            TRANSMIT_START_BIT, // Wait for the start signal
            pdTRUE,             // Clear the bit after it is received
            pdFALSE,            // Wait for any bit
            portMAX_DELAY       // Wait indefinitely
        );

        if (bits & TRANSMIT_START_BIT) {
            radar_td_rise_edge = gpio_get_level(RADAR_TD_GPIO);
            //ESP_LOGI(TAG_CC1101, "xradar_td_level value: %d", radar_td_rise_edge);
            if (radar_td_rise_edge) {
                txData = (const uint8_t *)"fac mmwave clear";
            } else {
                txData = (const uint8_t *)"fac mmwave det";
            }
            size_t txLen = strlen((const char *)txData); // Use strlen to calculate the string length
            //ESP_LOGI(TAG_CC1101, "txLen: %d", txLen);  // Added log for txLen
            //ESP_LOGI(TAG_CC1101, "Transmitting data...");
            cc1101_transmitData(txData, txLen);

            uint8_t marcstate = 0;
            esp_err_t ret = cc1101_readBurstRegister(CC1101_MARCSTATE, &marcstate, 1);

            // if (ret == ESP_OK) {
            //     ESP_LOGI(TAG_CC1101, "CC1101 MARCSTATE aft transmit: 0x%02X", marcstate);
            // } else {
            //     ESP_LOGE(TAG_CC1101, "Failed to read CC1101_MARCSTATE");
            // }
            xEventGroupSetBits(app_event_group, TRANSMIT_COMPLETE_BIT);
        }
    }
}

void cc1101_power_down() {

    esp_err_t ret;
    uint8_t marcstate = 0;

    cc1101_strobe(rf_spi, CC1101_SIDLE);  // Enter IDLE
    vTaskDelay(pdMS_TO_TICKS(1));

    // Read the MARCSTATE register to check the current state
    ret = cc1101_readBurstRegister(CC1101_MARCSTATE, &marcstate, 1);
    if (ret == ESP_OK) {
        //ESP_LOGI(TAG_CC1101, "MARCSTATE before power down: 0x%02X", marcstate);
    } else {
        //ESP_LOGE(TAG_CC1101, "Failed to read MARCSTATE before power down");
    }

    cc1101_strobe(rf_spi, CC1101_SPWD);   // Enter Power Down

    // Pull CS pin high to finalize the command transaction.
    gpio_set_level(CONFIG_SPI_CS_GPIO, 1);

    // Use RTC gpio hold to freeze the level during sleep
    gpio_hold_en(CONFIG_SPI_CS_GPIO);

    vTaskDelay(pdMS_TO_TICKS(500));
}

esp_err_t cc1101_wake_up(void)
{
    esp_err_t ret;

    //ESP_LOGI(TAG_CC1101, "Waking up CC1101");

    // Disable the CS hold so that the pin can toggle.
    gpio_hold_dis(CONFIG_SPI_CS_GPIO);
    
    // Toggle CS to wake up the chip. Some CC1101 designs require a low pulse on CS.
    gpio_set_level(CONFIG_SPI_CS_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CONFIG_SPI_CS_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Issue a reset strobe to reset and wake up the chip.
    ret = cc1101_reset(rf_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CC1101, "Failed to reset CC1101 during wake up");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t marcstate = 0;
    cc1101_readBurstRegister(CC1101_MARCSTATE, &marcstate, 1);
    //ESP_LOGI(TAG_CC1101, "CC1101 MARCSTATE (via burst): 0x%02X", marcstate);

     uint8_t partnum = 0;
     cc1101_readBurstRegister(CC1101_PARTNUM, &partnum, 1);
     //ESP_LOGI(TAG_CC1101, "CC1101 Part Number (via burst): 0x%02X", partnum);
    // Delay for the reset to take effect - increased from 1ms to 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

      uint8_t version = 0;
        cc1101_readBurstRegister(CC1101_VERSION, &version, 1);
    //ESP_LOGI(TAG_CC1101, "CC1101 Version (via burst): 0x%02X", version);

    // Reinitialize the device settings:
    cc1101_writeRegistersSetting(rf_spi, &defaultRF_Settings);
    cc1101_writeSingleRegister(rf_spi, CC1101_PATABLE, PATABLE_VAL);
    cc1101_writeSingleRegister(rf_spi, CC1101_SYNC1, (uint8_t)(defaultSyncWord >> 8));
    cc1101_writeSingleRegister(rf_spi, CC1101_SYNC0, (uint8_t)defaultSyncWord);

    // Optionally set the appropriate RF mode (for example, TX and RX)
    rf_set_mode(RF_MODE_TX_AND_RX);

    //ESP_LOGI(TAG_CC1101, "CC1101 wake-up complete");
    return ESP_OK;
}


