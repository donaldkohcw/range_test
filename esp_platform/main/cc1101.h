#ifndef CC1101_H
#define CC1101_H

#include "driver/spi_master.h"

#define RF_TX_MSG_QUEUE_SIZE                (16)     // Size must be in power of 2
#define RF_RX_MSG_QUEUE_SIZE                (8)     // Size must be in power of 2

extern TaskHandle_t transmitTaskHandle;

/**
 * Type of transfers
 */
#define WRITE_BURST              0x40
#define READ_SINGLE              0x80
#define READ_BURST               0xC0

/**
 * Type of register
 */
#define CC1101_CONFIG_REGISTER   READ_SINGLE
#define CC1101_STATUS_REGISTER   READ_BURST

/**
 * PATABLE & FIFO's
 */
#define CC1101_PATABLE           0x3E        // PATABLE address
#define CC1101_TXFIFO            0x3F        // TX FIFO address
#define CC1101_RXFIFO            0x3F        // RX FIFO address

/**
 * Command strobes
 */
#define CC1101_SRES              0x30        // Reset CC1101 chip
#define CC1101_SFSTXON           0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
                                             // Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32        // Turn off crystal oscillator
#define CC1101_SCAL              0x33        // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
                                             // setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35        // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                             // If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC1101_SIDLE             0x36        // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38        // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
                                             // WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39        // Enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A        // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B        // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C        // Reset real time clock to Event1 value
#define CC1101_SNOP              0x3D        // No operation. May be used to get access to the chip status byte

/**
 * CC1101 configuration registers
 */
#define CC1101_IOCFG2            0x00        // GDO2 Output Pin Configuration
#define CC1101_IOCFG1            0x01        // GDO1 Output Pin Configuration
#define CC1101_IOCFG0            0x02        // GDO0 Output Pin Configuration
#define CC1101_FIFOTHR           0x03        // RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1             0x04        // Sync Word, High Byte
#define CC1101_SYNC0             0x05        // Sync Word, Low Byte
#define CC1101_PKTLEN            0x06        // Packet Length
#define CC1101_PKTCTRL1          0x07        // Packet Automation Control
#define CC1101_PKTCTRL0          0x08        // Packet Automation Control
#define CC1101_ADDR              0x09        // Device Address
#define CC1101_CHANNR            0x0A        // Channel Number
#define CC1101_FSCTRL1           0x0B        // Frequency Synthesizer Control
#define CC1101_FSCTRL0           0x0C        // Frequency Synthesizer Control
#define CC1101_FREQ2             0x0D        // Frequency Control Word, High Byte
#define CC1101_FREQ1             0x0E        // Frequency Control Word, Middle Byte
#define CC1101_FREQ0             0x0F        // Frequency Control Word, Low Byte
#define CC1101_MDMCFG4           0x10        // Modem Configuration
#define CC1101_MDMCFG3           0x11        // Modem Configuration
#define CC1101_MDMCFG2           0x12        // Modem Configuration
#define CC1101_MDMCFG1           0x13        // Modem Configuration
#define CC1101_MDMCFG0           0x14        // Modem Configuration
#define CC1101_DEVIATN           0x15        // Modem Deviation Setting
#define CC1101_MCSM2             0x16        // Main Radio Control State Machine Configuration
#define CC1101_MCSM1             0x17        // Main Radio Control State Machine Configuration
#define CC1101_MCSM0             0x18        // Main Radio Control State Machine Configuration
#define CC1101_FOCCFG            0x19        // Frequency Offset Compensation Configuration
#define CC1101_BSCFG             0x1A        // Bit Synchronization Configuration
#define CC1101_AGCCTRL2          0x1B        // AGC Control
#define CC1101_AGCCTRL1          0x1C        // AGC Control
#define CC1101_AGCCTRL0          0x1D        // AGC Control
#define CC1101_WOREVT1           0x1E        // High Byte Event0 Timeout
#define CC1101_WOREVT0           0x1F        // Low Byte Event0 Timeout
#define CC1101_WORCTRL           0x20        // Wake On Radio Control
#define CC1101_FREND1            0x21        // Front End RX Configuration
#define CC1101_FREND0            0x22        // Front End TX Configuration
#define CC1101_FSCAL3            0x23        // Frequency Synthesizer Calibration
#define CC1101_FSCAL2            0x24        // Frequency Synthesizer Calibration
#define CC1101_FSCAL1            0x25        // Frequency Synthesizer Calibration
#define CC1101_FSCAL0            0x26        // Frequency Synthesizer Calibration
#define CC1101_RCCTRL1           0x27        // RC Oscillator Configuration
#define CC1101_RCCTRL0           0x28        // RC Oscillator Configuration
#define CC1101_FSTEST            0x29        // Frequency Synthesizer Calibration Control
#define CC1101_PTEST             0x2A        // Production Test
#define CC1101_AGCTEST           0x2B        // AGC Test
#define CC1101_TEST2             0x2C        // Various Test Settings
#define CC1101_TEST1             0x2D        // Various Test Settings
#define CC1101_TEST0             0x2E        // Various Test Settings

/**
 * Status registers
 */
#define CC1101_PARTNUM           0x30        // Chip ID
#define CC1101_VERSION           0x31        // Chip ID
#define CC1101_FREQEST           0x32        // Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        // Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        // Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        // Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        // High Byte of WOR Time
#define CC1101_WORTIME0          0x37        // Low Byte of WOR Time
#define CC1101_PKTSTATUS         0x38        // Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        // Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        // Underflow and Number of Bytes
#define CC1101_RXBYTES           0x3B        // Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS    0x3C        // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        // Last RC Oscillator Calibration Result

/**
 * MCSM1 value (Only used values are listed below - check cc1101 datasheet for complete list of values)
 */
#define CCA_MODE_ALWAYS_CLEAR    0x00
#define RXOFF_MODE_STAY_RX       0x0C
#define TXOFF_MODE_IDLE          0x00
#define TXOFF_MODE_RX            0x03

#define PATABLE_VAL              0xC6        // Example value (adjust as needed)
#define defaultSyncWord          0xD392      // Example 16-bit sync word (adjust as needed)

#define RF_TX_QUEUE_MESSAGE_SIZE	(sizeof(rf_tx_packetData_t))

#define CC1101_RSSI_OFFSET                  (74)
#define CC1101_MAX_FIFO_SIZE                (64)
#define CC1101_APPENDED_STATUS_BYTES_SIZE   (2)
#define CC1101_HEADER_SIZE                  (1)     // 1 byte length
#define CC1101_MAX_PACKET_LEN               (CC1101_MAX_FIFO_SIZE - CC1101_HEADER_SIZE - CC1101_APPENDED_STATUS_BYTES_SIZE)
#define RF_BUFFER_SIZE                      (CC1101_MAX_PACKET_LEN)

typedef struct s_rf_rx_packetData
{
    uint8_t msg_len;
    uint8_t encryptedData[RF_BUFFER_SIZE];
    uint8_t rssi;
    uint8_t status;
} rf_rx_packetData_t;

typedef struct s_rf_tx_packetData
{
    uint8_t msg_len;
    uint8_t encryptedData[RF_BUFFER_SIZE];
} rf_tx_packetData_t;

typedef union
{
    rf_tx_packetData_t packet;
    uint8_t bytes[RF_TX_QUEUE_MESSAGE_SIZE];
} rf_tx_message_union;

typedef struct S_RF_SETTINGS
{
    unsigned char fsctrl1;
    unsigned char fsctrl0;
    unsigned char freq2;
    unsigned char freq1;
    unsigned char freq0;
    unsigned char mdmcfg4;
    unsigned char mdmcfg3;
    unsigned char mdmcfg2;
    unsigned char mdmcfg1;
    unsigned char mdmcfg0;
    unsigned char channr;
    unsigned char deviatn;
    unsigned char frend1;
    unsigned char frend0;
    unsigned char mcsm1;
    unsigned char mcsm0;
    unsigned char foccfg;
    unsigned char bscfg;
    unsigned char agcctrl2;
    unsigned char agcctrl1;
    unsigned char agcctrl0;
    unsigned char fscal3;
    unsigned char fscal2;
    unsigned char fscal1;
    unsigned char fscal0;
    unsigned char fstest;
    unsigned char test2;
    unsigned char test1;
    unsigned char test0;
    unsigned char fifothr;
    unsigned char iocfg2;
    unsigned char iocfg0;
    unsigned char pktctrl1;
    unsigned char pktctrl0;
    unsigned char addr;
    unsigned char pktlen;
} RF_SETTINGS;

typedef enum {
    RF_MODE_TX_ONLY = 0,
    RF_MODE_TX_AND_RX
} rf_mode_t;

void rf_set_mode(rf_mode_t new_rf_mode);
void rf_set_idle(void);
void cc1101_power_down(void);


esp_err_t cc1101_init(void);
esp_err_t cc1101_write_reg(spi_device_handle_t spi, uint8_t addr, uint8_t value);
esp_err_t cc1101_readSingleRegister(spi_device_handle_t spi, uint8_t addr, uint8_t *value);
esp_err_t cc1101_strobe(spi_device_handle_t spi, uint8_t strobe);
esp_err_t cc1101_writeSingleRegister(spi_device_handle_t spi, uint8_t addr, uint8_t value);
esp_err_t cc1101_reset(spi_device_handle_t spi);
esp_err_t cc1101_transmitData(const uint8_t *data, size_t len);
esp_err_t cc1101_receiveData(uint8_t *buffer, size_t max_len, size_t *received_len);
esp_err_t cc1101_wake_up(void);
esp_err_t cc1101_spi_deinit(void);



/**
 * @brief Read multiple registers from the CC1101 in burst mode.
 *
 * This function reads a sequence of registers from the CC1101 starting from the provided
 * start address. The returned status byte from the CC1101 is discarded and only the
 * register data is written to the provided buffer.
 *
 * @param startAddress   Start address of the register to read.
 * @param buffer         Pointer to the buffer where register data will be stored.
 * @param length         Number of bytes to be read.
 */
esp_err_t cc1101_readBurstRegister(uint8_t startAddress, uint8_t *buffer, uint8_t length);

/**
 * @brief Write multiple registers to the CC1101 in burst mode.
 *
 * This function writes a sequence of registers starting from the specified address
 * using a burst SPI transfer.
 *
 * @param startAddress Starting register address.
 * @param buffer Pointer to the data to be written.
 * @param length Number of bytes to write.
 */
void cc1101_writeBurstRegister(uint8_t startAddress, uint8_t *buffer, uint8_t length);
/* Declaration for writing all RF register settings using an RF_SETTINGS structure */
void cc1101_writeRegistersSetting(spi_device_handle_t spi, RF_SETTINGS *pRfSettings);

#endif // CC1101_H