#include "bgt60l_spi.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "spi_task.h"

static const char *TAG_BGT60LTR_SPI = "BGT60L_SPI";
static spi_device_handle_t bgt60l_spi_handle = NULL;

esp_err_t bgt60l_spi_init(void)
{

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,       // 1 MHz clock
        .mode = 0,                               // SPI mode 0
        .spics_io_num = CONFIG_BGT60L_CS_GPIO,     // Chip Select is GPIO25
        .queue_size = 3,
    };

    esp_err_t ret = spi_bus_add_device(CONFIG_SPI_HOST, &devcfg, &bgt60l_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BGT60LTR_SPI, "Failed to add BGT60L SPI device: %d", ret);
        return ret;
    }
    //ESP_LOGI(TAG_BGT60LTR_SPI, "BGT60L SPI initialized using CS GPIO %d", CONFIG_BGT60L_CS_GPIO);

    //Perform hardware reset to initialize the device properly
    //ESP_ERROR_CHECK(bgt60l_hw_reset());

    //bgt60l_dump_registers();   

    return ESP_OK;
}

esp_err_t bgt60l_write(uint8_t reg, uint16_t data)
{
    // Command byte: (7-bit address << 1) | 1 (write bit set)
    uint8_t tx_buf[3];
    tx_buf[0] = (reg << 1) | 1;
    tx_buf[1] = (data >> 8) & 0xFF;  // Data high byte (bits 15..8)
    tx_buf[2] = data & 0xFF;         // Data low byte (bits 7..0)

    spi_transaction_t t = {
        .length = 24,         // 3 bytes * 8 bits per word
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,    // No data to receive for a write
    };

    esp_err_t ret = spi_device_transmit(bgt60l_spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BGT60LTR_SPI, "Failed to write to BGT60L register 0x%02X: %d", reg, ret);
    } else {
        //ESP_LOGI(TAG_BGT60LTR_SPI, "Write to reg 0x%02X, data 0x%04X successful", reg, data);
    }
    return ret;
}

esp_err_t bgt60l_read(uint8_t reg, uint16_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For read mode, the command byte is (reg << 1) with the R/W bit clear (0)
    uint8_t tx_buf[3] = { (reg << 1) | 0, 0x00, 0x00 };
    uint8_t rx_buf[3] = { 0 };

    spi_transaction_t t = {
        .length = 24,         // 3 bytes = 24 bits
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_transmit(bgt60l_spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BGT60LTR_SPI, "Failed to read from register 0x%02X: %d", reg, ret);
        return ret;
    }
    
    // Assuming the first byte received is a status/dummy byte,
    // and the next two bytes form the 16-bit register data (MSB first)
    *data = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    //ESP_LOGI(TAG_BGT60LTR_SPI, "Read reg 0x%02X: Data = 0x%04X", reg, *data);
    return ESP_OK;
}

/**
 * @brief Dump all BGT60UTR11AIP registers.
 *
 * Reads and logs the values of the following registers:
 * - MAIN (0x00)
 * - ADC0 (0x01)
 * - CHIP_Version (0x02)
 * - STAT1 (0x03)
 * - PACR1 (0x04)
 * - PACR2 (0x05)
 * - SFCTL (0x06)
 * - CSI_0 (0x08)
 * - CSI_1 (0x09)
 * - CSI_2 (0x0A)
 * - CSCI (0x0B)
 * - CSDS_0 (0x0C)
 * - CSDS_1 (0x0D)
 * - CSDS_2 (0x0E)
 */
void bgt60l_dump_registers(void)
{
    struct {
        uint8_t reg;
        const char *name;
    } regs[] = {
        { S2GO_RADAR_BGT60LTR11_REG_REG00,              "REG00" },
        { S2GO_RADAR_BGT60LTR11_REG_REG01,              "REG01" },
        { S2GO_RADAR_BGT60LTR11_REG_REG02,              "REG02" },
        { S2GO_RADAR_BGT60LTR11_REG_REG03,              "REG03 (Reserved)" },
        { S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1,     "PLL_CFG1" },
        { S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2,     "PLL_CFG2" },
        { S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3,     "PLL_CFG3" },
        { S2GO_RADAR_BGT60LTR11_REG_REG07,              "REG07" },
        { S2GO_RADAR_BGT60LTR11_REG_REG08_DIV,          "DIV" },
        { S2GO_RADAR_BGT60LTR11_REG_REG09_BB,           "BB" },
        { S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME,     "HOLDTIME" },
        { S2GO_RADAR_BGT60LTR11_REG_REG11,              "REG11 (Reserved)" },
        { S2GO_RADAR_BGT60LTR11_REG_REG12,              "REG12" },
        { S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1,        "ALGO1" },
        { S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2,        "ALGO2" },
        { S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL,      "DIGCTRL" },
        { S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0,         "ADC0" },
        { S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1,         "ADC1" },
        { S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT,     "ADC_STAT" },
        { S2GO_RADAR_BGT60LTR11_REG_REG38_ADC_RESULT,   "ADC_RESULT" },
        { S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES,      "E_FUSES" },
        { S2GO_RADAR_BGT60LTR11_REG_REG56_CHIP_STAT,    "CHIP_STAT" }
    };
    int num_regs = sizeof(regs) / sizeof(regs[0]);
    uint16_t data;
    
    for (int i = 0; i < num_regs; i++) {
        esp_err_t ret = bgt60l_read(regs[i].reg, &data);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_BGT60LTR_SPI, "%s (register 0x%02X): 0x%04X", regs[i].name, regs[i].reg, data);
        } else {
            ESP_LOGE(TAG_BGT60LTR_SPI, "Error reading register 0x%02X (%s)", regs[i].reg, regs[i].name);
        }
    }
}

/**
 * @brief Perform a hardware reset for the BGT60UTR11AIP.
 *
 * The reset sequence requires that while CSN is held high, RST transitions:
 * High (for T_CS_BRES) -> Low (for T_RES) -> High (for T_CS_ARES).
 * Datasheet timing parameters are 100 ns each. In software, we use 1 µs delays
 * to safely meet these minimal timing requirements.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bgt60l_hw_reset(void)
{
    // Configure the RST pin as an output
    gpio_reset_pin(CONFIG_BGT60L_RST_GPIO);
    gpio_set_level(CONFIG_BGT60L_RST_GPIO, 1);
    gpio_set_direction(CONFIG_BGT60L_RST_GPIO, GPIO_MODE_OUTPUT);

    //Ensure CSN remains high (assumed handled by the SPI hardware)
    //Start the reset sequence:
    gpio_set_level(CONFIG_BGT60L_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));  // T_CS_BRES (≥100 ns)
    
    gpio_set_level(CONFIG_BGT60L_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));  // T_RES (≥100 ns)
    
    gpio_set_level(CONFIG_BGT60L_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));  // T_CS_ARES (≥100 ns)

    ESP_LOGI("BGT60L_RST", "Hardware reset sequence performed");
    return ESP_OK;
}

esp_err_t mtb_s2go_radar_bgt60ltr11_threshold(Reg02_detector_threshold_level_t threshold)
{
    uint16_t spi_word;
    // Build SPI word: set the THRS field in REG02 with the provided threshold value
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG02, THRS, threshold);
    // Write the SPI word to register REG02 (address 0x02)
    return bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG02, spi_word);
}

esp_err_t mtb_s2go_radar_bgt60ltr11_hold_time(Reg10_hold_time_t hold_time)
{
    uint16_t spi_word;
    // Build SPI word: set the HOLD field in REG10_HOLDTIME with the provided hold_time value
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG10_HOLDTIME, HOLD, hold_time);
    // Write the SPI word to register REG10_HOLDTIME (address 0x0A)
    return bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME, spi_word);
}

esp_err_t mtb_s2go_radar_bgt60ltr11_frequency(Reg05_pll_frequency_word_t frequency)
{
    uint16_t spi_word;
    // Build SPI word: set the FCW field in REG05_PLL_CFG2 with the provided frequency value.
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG05_PLL_CFG2, FCW, frequency);
    // Write the SPI word to register REG05_PLL_CFG2 (address 0x05)
    return bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2, spi_word);
}

esp_err_t mtb_s2go_radar_bgt60ltr11_PGA_gain(Reg09_baseband_ctrl_t PGA_gain)
{
    uint16_t spi_word;
    // Build SPI word: set the CTRL_GAIN field in REG09_BB with the provided PGA_gain value.
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG09_BB, CTRL_GAIN, PGA_gain);
    // Write the SPI word to register REG09_BB (address, for example, defined as S2GO_RADAR_BGT60LTR11_REG_REG09_BB)
    return bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG09_BB, spi_word);
}

esp_err_t mtb_s2go_radar_bgt60ltr11_pulsed_mode_init(void)
{
    uint16_t spi_word;
    esp_err_t error;

    uint16_t reg56;
    uint16_t reg0=0;

    // (Optionally) Enable soft reset (assumes a similar no-obj version exists)
      error = mtb_s2go_radar_bgt60ltr11_enable_soft_reset();
    if(error != ESP_OK) {
        return error;
    }

    // Register 0: Clear all settings in REG00
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG00, CLEAR_ALL, 0);
    //printf("Register 0 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG00, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 1: Clear all settings in REG01
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG01, CLEAR_ALL, 0);
    //printf("Register 1 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG01, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 2: Set HPRT, APRT, DIR_MODE and THRS fields in REG02
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG02, HPRT, hprt_off) |
              S2GO_RADAR_BGT60LTR11_SET(REG02, APRT, aprt_off) |
              S2GO_RADAR_BGT60LTR11_SET(REG02, DIR_MODE, dir_mode_2) |
              S2GO_RADAR_BGT60LTR11_SET(REG02, THRS, det_thrs_80);
    
            //   spi_word = S2GO_RADAR_BGT60LTR11_SET(REG02, HPRT, hprt_on) |
            //   S2GO_RADAR_BGT60LTR11_SET(REG02, APRT, aprt_on) |
            //   S2GO_RADAR_BGT60LTR11_SET(REG02, DIR_MODE, dir_mode_2) |
            //   S2GO_RADAR_BGT60LTR11_SET(REG02, THRS, det_thrs_2560);              
    //printf("Register 2 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG02, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 3: Reserved field in REG03
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG03, RESERVED, 0);
    //printf("Register 3 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG03, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 4: Configure REG04_PLL_CFG1 with multiple fields
    spi_word =
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, PFD_RDT_SEL, pfd_rst_dly_375ps) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, CP_MODE, bias_regulation_loop_active) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, CP_ICP_SEL, charge_pump_current_55uA) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, FBDIV_CNT, feedback_divider_counter_21dec) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, XOSC_MODE, xosc_mode_amplitude_1) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, LF_R2_SEL, loopfilter_R2_18_7Kohm) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, CL_LOOP_PMODE, closed_loop_pusled_mode) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, LF_R4_SEL, loopfilter_R4_0_1Kohm) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, LF_ISO, loopfilter_isolation_with_switches_only) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, BIAS_DIS, biassing_on) |
       S2GO_RADAR_BGT60LTR11_SET(REG04_PLL_CFG1, DFT_DMUX, mux_functional_mode);
    //printf("Register 4 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 5: Set frequency in REG05_PLL_CFG2
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG05_PLL_CFG2, FCW, frequency_61_25Ghz);
    //printf("Register 5 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 6: Configure lock detection in REG06_PLL_CFG3
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG06_PLL_CFG3, LD_EN, lock_detection_on) |
              S2GO_RADAR_BGT60LTR11_SET(REG06_PLL_CFG3, LD_LEN, lock_detection_24_clock) |
              S2GO_RADAR_BGT60LTR11_SET(REG06_PLL_CFG3, LD_TW_SEL, lock_detection_time_window_1_5ns);
    //printf("Register 6 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 7: Configure REG07 with multiple fields
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG07, MPA_CTRL, mpa_4_5dBm) |
              S2GO_RADAR_BGT60LTR11_SET(REG07, PD_EN, pd_off) |
              S2GO_RADAR_BGT60LTR11_SET(REG07, MPA2SH_DLY, map_enable_to_sample_hold_delay_1000ns) |
              S2GO_RADAR_BGT60LTR11_SET(REG07, VCO2PLL_DLY, vco_to_pll_delay_1000ns) |
              //S2GO_RADAR_BGT60LTR11_SET(REG07, DC_ON_PULSE_LEN, duty_cycle_on_pulse_10us) |
              S2GO_RADAR_BGT60LTR11_SET(REG07, DC_ON_PULSE_LEN, duty_cycle_on_pulse_5us) |
              S2GO_RADAR_BGT60LTR11_SET(REG07, DC_REP_RATE, duty_cycle_rep_500us);
    //printf("Register 7 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG07, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 8: Configure REG08_DIV
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG08_DIV, OUT_EN, 1) |
              S2GO_RADAR_BGT60LTR11_SET(REG08_DIV, SEL, divider_select_2_power_20);
    //printf("Register 8 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG08_DIV, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 9: Configure REG09_BB - PGA gain, LPF, clock chop, and HP filter
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG09_BB, CTRL_GAIN, baseband_PGA_gain_50dB) |
              S2GO_RADAR_BGT60LTR11_SET(REG09_BB, LPF_BW, low_pass_filter_10kHz) |
              S2GO_RADAR_BGT60LTR11_SET(REG09_BB, CLK_CHOP_SEL, clock_chop_freq_200kHz) |
              S2GO_RADAR_BGT60LTR11_SET(REG09_BB, HP_RES, high_pass_filter_resistor_1Mohm);
    //printf("Register 9 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG09_BB, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 10: Set hold time in REG10_HOLDTIME  
    //spi_word = S2GO_RADAR_BGT60LTR11_SET(REG10_HOLDTIME, HOLD, hold_time_5min);
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG10_HOLDTIME, HOLD, hold_time_60_032sec);
    //spi_word = S2GO_RADAR_BGT60LTR11_SET(REG10_HOLDTIME, HOLD, hold_time_40_05sec);
  //  spi_word = S2GO_RADAR_BGT60LTR11_SET(REG10_HOLDTIME, HOLD, hold_time_512msec);

    //printf("Register 10 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 12: Clear REG12
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG12, CLEAR_ALL, 0);
    //printf("Register 12 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG12, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 13: Clear REG13_ALGO1
    // spi_word = S2GO_RADAR_BGT60LTR11_SET(REG13_ALGO1, CLEAR_ALL, 0);
    // printf("Register 13 value: 0x%X\r\n", spi_word);
    // error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1, 0x0000);
    
    // Set Register 13 (ALGO1) with the prt_mult field set to prt_mult_3D
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG13_ALGO1, PRT_MULT,prt_mult_3D);
    //printf("Register 13 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1, spi_word);

    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 14: Clear REG14_ALGO2
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG14_ALGO2, CLEAR_ALL, 0);
    //printf("Register 14 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 15: Set START_PM inactive in REG15_DIGCTRL first
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG15_DIGCTRL, START_PM, start_pm_inactive);
    //printf("Register 15 value (start_pm inactive): 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 34: Clear REG34_ADC0
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG34_ADC0, CLEAR_ALL, 0);
    //printf("Register 34 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0, 0x0000);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 35: Clear REG35_ADC1
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG35_ADC1, CLEAR_ALL, 0);
    //printf("Register 35 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1, 0x0000);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Register 36: Clear REG36_ADC_STAT
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG36_ADC_STAT, CLEAR_ALL, 0);
    //printf("Register 36 value: 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT, 0x0000);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));



    // Register 15 again: Set START_PM active in REG15_DIGCTRL
    spi_word = S2GO_RADAR_BGT60LTR11_SET(REG15_DIGCTRL, START_PM, start_pm_active);
    //printf("Register 15 value (start_pm active): 0x%X\r\n", spi_word);
    error = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL, spi_word);
    if(error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(1));


    // Record the start time (in microseconds)
    uint64_t start_time = esp_timer_get_time();
    // Loop to confirm soft reset is complete
    for (volatile unsigned short i = 0; i < 2048; i++)
    {
        error = bgt60l_read(0x38, &reg56);
        if(error != ESP_OK){
            return error;
        }
        
        // Check if register 0 is 0 and bit 13 of reg56 is set
        if((reg0 == 0) && (reg56 & (1 << 13)))
        {
            break;
        }
    }

    // Record the end time and calculate elapsed time
    uint64_t elapsed_us = esp_timer_get_time() - start_time;
    //ESP_LOGI(TAG_BGT60LTR_SPI, "Total wait time: %llu microseconds (%.3f seconds)", elapsed_us, elapsed_us / 1000000.0);

    return ESP_OK;
}

esp_err_t mtb_s2go_radar_bgt60ltr11_enable_soft_reset(void)
{
    esp_err_t error;
    uint16_t data;
    uint16_t reg56;
    uint16_t reg0;
    
    // Read register 0x0F
    error = bgt60l_read(0x0F, &data);
    if(error != ESP_OK){
        return error;
    }
    
    // Set bit 15 (0x8000) in the read value
    data |= 0x8000;
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms delay
    
    // Write the updated value back to register 0x0F
    error = bgt60l_write(0x0F, data);
    if(error != ESP_OK){
        return error;
    }   
    return ESP_OK;
}

esp_err_t mtb_s2go_radar_bgt60ltr11_set_miso_drv(Reg15_spi_force_miso_driver_t miso_drv)
{
    esp_err_t ret;
    uint16_t reg_val;

    // Read the current value of REG15_DIGCTRL
    ret = bgt60l_read(S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL, &reg_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_BGT60LTR_SPI, "Failed to read REG15_DIGCTRL");
        return ret;
    }

    // Clear the current MISO_DRV field bits
    reg_val &= ~S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_MISO_DRV_msk;
    // Set the new MISO_DRV value using the provided macro
    reg_val |= S2GO_RADAR_BGT60LTR11_SET(REG15_DIGCTRL, MISO_DRV, miso_drv);

    // Write the updated value back to REG15_DIGCTRL
    ret = bgt60l_write(S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL, reg_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_BGT60LTR_SPI, "Failed to write REG15_DIGCTRL");
    }
    return ret;
}

esp_err_t bgt60l_spi_deinit(void)
{
    esp_err_t ret = ESP_OK;
    if (bgt60l_spi_handle != NULL) {
        ret = spi_bus_remove_device(bgt60l_spi_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_BGT60LTR_SPI, "Failed to remove BGT60L SPI device: %d", ret);
        }
        bgt60l_spi_handle = NULL;
    }
    return ret;
}