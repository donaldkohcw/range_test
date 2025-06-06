#ifndef BGT60L_SPI_H
#define BGT60L_SPI_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    det_thrs_66		= 66,
    det_thrs_80		= 80,
    det_thrs_90		= 90,
    det_thrs_112	= 112,
    det_thrs_136	= 136,
    det_thrs_192	= 192,
    det_thrs_248	= 248,
    det_thrs_320	= 320,
    det_thrs_384	= 384,
    det_thrs_480	= 480,
    det_thrs_640	= 640,
    det_thrs_896	= 896,
    det_thrs_1344	= 1344,
    det_thrs_1920	= 1920,
    det_thrs_2560	= 2560
} Reg02_detector_threshold_level_t;

typedef enum
{
    dir_mode_1	= 0,
    dir_mode_2	= 1
} Reg02_direction_detection_mode_t;

typedef enum
{
    aprt_off	= 0,
    aprt_on		= 1
} Reg02_adaptive_pulse_repetition_time_t;

typedef enum
{
    hprt_off	= 0,
    hprt_on		= 1
} Reg02_high_pulse_repetition_time_t;

typedef enum
{
    pfd_rst_dly_175ps	= 0,
    pfd_rst_dly_275ps	= 1,
    pfd_rst_dly_375ps	= 2,
    pfd_rst_dly_470ps	= 3
} Reg04_pfd_rst_dly_select_t;

typedef enum
{
    bias_regulation_loop_active	= 0,
    bias_regulation_loop_off	= 1
} Reg04_pll_cp_mode_t;

typedef enum
{
    charge_pump_current_20uA	= 0,
    charge_pump_current_25uA	= 1,
    charge_pump_current_30uA	= 2,
    charge_pump_current_35uA	= 3,
    charge_pump_current_40uA	= 4,
    charge_pump_current_45uA	= 5,
    charge_pump_current_50uA	= 6,
    charge_pump_current_55uA	= 7
} Reg04_charge_pump_current_t;

typedef enum
{
    feedback_divider_counter_21dec	= 0,	/* 38.4 MHz */
    feedback_divider_counter_20dec	= 1		/* 40 MHz */
} Reg04_pll_feedback_divider_counter_t;

typedef enum
{
    xosc_mode_amplitude_1	= 0,
    xosc_mode_amplitude_2	= 1
} Reg04_pll_xosc_mode_t;

typedef enum
{
    loopfilter_R2_21_6Kohm	= 0,
    loopfilter_R2_18_7Kohm	= 1
} Reg04_pll_loopfilter_R2_t;

typedef enum
{
    loopfilter_R4_12_4Kohm	= 0,
    loopfilter_R4_0_1Kohm	= 1
} Reg04_pll_loopfilter_R4_t;

typedef enum
{
    open_loop_pusled_mode	= 0,
    closed_loop_pusled_mode	= 1
} Reg04_pll_close_loop_pusled_mode_t;

typedef enum
{
    loopfilter_isolation_with_charge_keeping_buffer_enabled	= 0,
    loopfilter_isolation_with_switches_only					= 1
} Reg04_pll_loopfilter_isolation_t;

typedef enum
{
    biassing_on = 0,
    biassing_off = 1
} Reg04_pll_bias_dis_t;

typedef enum
{
    mux_functional_mode				= 0,
    pll_lock						= 1,
    reference_clock_divided_by_4	= 2,
    divider_clock_divided_by_4		= 3
} Reg04_pll_dft_dmux_t;

typedef enum
{
    frequency_60_6Ghz	= 0xea2,
    frequency_60_7Ghz	= 0xecc,
    frequency_60_8Ghz	= 0xef5,
    frequency_60_9Ghz	= 0xf1f,
    frequency_61_1Ghz	= 0xf72,
    frequency_61_2Ghz	= 0xf9c,
    frequency_61_25Ghz	= 0xfb0,
    frequency_61_3Ghz	= 0xfc6,
    frequency_61_4Ghz	= 0xfef
} Reg05_pll_frequency_word_t;


typedef enum
{
    lock_detection_off	= 0,
    lock_detection_on	= 1
} Reg06_pll_lock_detect_en_t;

typedef enum
{
    lock_detection_24_clock	= 0,
    lock_detection_16_clock	= 1
} Reg06_pll_lock_detect_len_t;

typedef enum
{
    lock_detection_time_window_0_26ns	= 0,
    lock_detection_time_window_0_5ns	= 1,
    lock_detection_time_window_1ns		= 2,
    lock_detection_time_window_1_5ns	= 3,
    lock_detection_time_window_2ns		= 4,
    lock_detection_time_window_2_8ns	= 5,
    lock_detection_time_window_3_8ns	= 6,
    lock_detection_time_window_4_6ns	= 7
} Reg06_pll_lock_detect_time_window_t;

typedef enum
{
    mpa_minus_34dBm		= 0,
    mpa_minus_31_5dBm	= 1,
    mpa_minus_25dBm		= 2,
    mpa_minus_18dBm		= 3,
    mpa_minus_11dBm		= 4,
    mpa_minus_5dBm		= 5,
    mpa_0dBm			= 6,
    mpa_4_5dBm			= 7
} Reg07_mpa_ctrl_t;

typedef enum
{
    pd_off	= 0,
    pd_on	= 1
} Reg07_pd_en_t;

typedef enum
{
    map_enable_to_sample_hold_delay_500ns	= 0,
    map_enable_to_sample_hold_delay_1000ns	= 1,
    map_enable_to_sample_hold_delay_2000ns	= 2,
    map_enable_to_sample_hold_delay_4000ns	= 3
} Reg07_mpa_2sh_dly_t;

typedef enum
{
    vco_to_pll_delay_500ns	= 0,
    vco_to_pll_delay_1000ns	= 1
} Reg07_vco_to_pll_delay_t;

typedef enum
{
    duty_cycle_on_pulse_5us		= 0, //5us
    duty_cycle_on_pulse_10us	= 1, //10us
    duty_cycle_on_pulse_20us	= 2, //3us
    duty_cycle_on_pulse_40us	= 3  //4us
} Reg07_duty_cycle_on_pulse_length_t;

typedef enum
{
    duty_cycle_rep_250us	= 0,
    duty_cycle_rep_500us	= 1,
    duty_cycle_rep_1000us	= 2,
    duty_cycle_rep_2000us	= 3
} Reg07_duty_cycle_repetition_rate_t;

typedef enum
{
    internal_9_6Mhz_clock_out	= 0,
    divider_select_2_power_13	= 1,
    divider_select_2_power_16	= 2,
    divider_select_2_power_20	= 3
} Reg08_divider_select_t;

typedef enum
{
    baseband_PGA_gain_10dB	= 0,
    baseband_PGA_gain_15dB	= 1,
    baseband_PGA_gain_20dB	= 2,
    baseband_PGA_gain_25dB	= 3,
    baseband_PGA_gain_30dB	= 4,
    baseband_PGA_gain_35dB	= 5,
    baseband_PGA_gain_40dB	= 6,
    baseband_PGA_gain_45dB	= 7,
    baseband_PGA_gain_50dB	= 8,
} Reg09_baseband_ctrl_t;

typedef enum
{
    low_pass_filter_10kHz	= 0,
    low_pass_filter_60kHz	= 1
} Reg09_low_pass_filter_t;

typedef enum
{
    clock_chop_freq_100kHz	= 0,
    clock_chop_freq_200kHz	= 1
} Reg09_bb_clock_chop_freq_t;

typedef enum
{
    high_pass_filter_resistor_8Mohm	= 0,
    high_pass_filter_resistor_4Mohm	= 1,
    high_pass_filter_resistor_2Mohm	= 2,
    high_pass_filter_resistor_1Mohm	= 3
} Reg09_high_pass_filter_resistor_t;

typedef enum
{
    hold_time_minimum	= 0,
    hold_time_512msec	= 4,
    hold_time_1_024sec	= 8,
    hold_time_2_048sec 	= 16,
    hold_time_3_072sec	= 24,
    hold_time_5_1sec 	= 40,
    hold_time_10_11sec 	= 79,
    hold_time_20_08sec 	= 235,
    hold_time_40_05sec 	= 352,
    hold_time_60_032sec = 469,
    hold_time_90_112sec = 704,
    hold_time_2min 		= 938,
    hold_time_5min 		= 2345,
    hold_time_10min 	= 4690,
    hold_time_15min 	= 7035,
    hold_time_30_016min = 14070
} Reg10_hold_time_t;

typedef enum
{
    detector_hit_counter_10		= 0,
    detector_hit_counter_20		= 1,
    detector_hit_counter_40		= 2,
    detector_hit_counter_80		= 3
} Reg11_detector_hit_counter_t;

typedef enum
{
    detector_hit_counter_reset_10ms		= 0,
    detector_hit_counter_reset_50ms		= 1,
    detector_hit_counter_reset_100ms	= 2,
    detector_hit_counter_reset_200ms	= 3
} Reg11_detector_hit_counter_reset_t;

typedef enum
{
    detector_direction_as_same_as_detector_hit_counter			= 0,
    detector_direction_as_same_as_half_of_detector_hit_counter	= 1
} Reg11_detector_direction_hit_counter_t;

/**
 * @brief Pulse Repetition Time Multiplier values for Register 13 (ALGO1).
 *
 * If APRT is enabled (Reg2[14] == 1), the prt_mult field (bits [1:0])
 * controls the multiplier as follows:
 * - 0D (0): Multiplier of 4
 * - 1D (1): Multiplier of 8
 * - 2D (2): Multiplier of 16
 * - 3D (3): Multiplier of 2
 */
typedef enum {
    prt_mult_0D = 0,  /**< Multiplier of 4 */
    prt_mult_1D = 1,  /**< Multiplier of 8 */
    prt_mult_2D = 2,  /**< Multiplier of 16 */
    prt_mult_3D = 3   /**< Multiplier of 2 */
} Reg13_prt_mult_t;

typedef enum
{
    dir_pol_tdet_low_when_departing		= 0,
    dir_pol_tdet_low_when_approaching	= 1
} Reg15_direction_polarity_t;

typedef enum
{
    mot_pol_tdet_active_low		= 0,
    mot_pol_tdet_active_high	= 1
} Reg15_motion_polarity_t;

typedef enum
{
    miso_drv_high_z		= 0,
    miso_drv_some_level	= 1
} Reg15_spi_force_miso_driver_t;

typedef enum
{
    fast_phase_start_when_target_detected	= 0,
    fast_phase_start_before_target_detected	= 1
} Reg15_spi_fast_mode_t;

typedef enum
{
    fastmode_spi_miso_changes_rising_edge	= 0,
    fastmode_spi_miso_changes_falling_edge	= 1
} Reg15_faster_phase_evaluation_t;

typedef enum
{
    start_cw_inactive	= 0,
    start_cw_active		= 1
} Reg15_start_cw_mode_t;

typedef enum
{
    clk_ext_not_disabled	= 0,
    clk_ext_disabled		= 1
} Reg15_start_pulsed_mode_t;

typedef enum
{
    start_pm_inactive	= 0,
    start_pm_active		= 1
} Reg15_disable_external_clock_t;

typedef enum
{
    soft_reset_inactive	= 0,
    soft_reset_active	= 1
} Reg15_soft_reset_t;


typedef struct
{
    Reg05_pll_frequency_word_t			output_frequency;
    Reg07_mpa_ctrl_t 					output_power;
    Reg07_pd_en_t 						pd;
    Reg07_mpa_2sh_dly_t 				map_enable_to_sample_hold_delay;
    Reg07_vco_to_pll_delay_t 			vco_to_pll_delay;
    Reg07_duty_cycle_on_pulse_length_t 	pulse_on_time;
    Reg07_duty_cycle_repetition_rate_t 	pulse_repetition_time;
    Reg09_baseband_ctrl_t 				baseband_gain;
    Reg09_low_pass_filter_t 			low_pass_filter;
    Reg09_bb_clock_chop_freq_t 			clock_chop_freq;
    Reg09_high_pass_filter_resistor_t 	high_pass_filter_resistor;
} S2GO_RADAR_BGT60LTR11_Config_t;


/**
 * @brief Initialize the BGT60L SPI device.
 *
 * This function adds the BGT60L device to the SPI bus using the chip-select defined as
 * CONFIG_BGT60L_CS_GPIO.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bgt60l_spi_init(void);

/**
 * @brief Dump all BGT60UTR11AIP registers.
 *
 * Reads and logs the values of various registers.
 */
void bgt60l_dump_registers(void);

/**
 * @brief Write a 16-bit value to a BGT60L register.
 *
 * The SPI word for the BGT60L is composed of 1 command byte and 2 data bytes.
 * This function writes a 16-bit data value to the specified register.
 *
 * @param reg Register address (command byte; include R/W bit if required).
 * @param data 16-bit data to write.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bgt60l_write(uint8_t reg, uint16_t data);

/**
 * @brief Read a 16-bit value from a BGT60L register.
 *
 * The SPI word for the BGT60L is composed of 1 command byte and 2 data bytes.
 * This function reads a 16-bit value from the specified register.
 *
 * @param reg Register address (command byte; include R/W bit if required).
 * @param data Pointer to a 16-bit variable to store the data.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bgt60l_read(uint8_t reg, uint16_t *data);

/**
 * @brief Perform a hardware reset for the BGT60UTR11AIP.
 *
 * Executes the reset sequence required by the device.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bgt60l_hw_reset(void);

/**
 * @brief Set the detector threshold for the BGT60LTR11 device.
 *
 * This function builds a 16-bit SPI word using the THRS field in REG02
 * and writes it to the device.
 *
 * @param threshold A value of type Reg02_detector_threshold_level_t.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mtb_s2go_radar_bgt60ltr11_threshold(Reg02_detector_threshold_level_t threshold);

/**
 * @brief Set the hold time for the BGT60LTR11 device.
 *
 * This function builds a 16-bit SPI word using the HOLD field in REG10_HOLDTIME
 * and writes it to the device.
 *
 * @param hold_time A value of type Reg10_hold_time_t.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mtb_s2go_radar_bgt60ltr11_hold_time(Reg10_hold_time_t hold_time);

/**
 * @brief Set the PLL frequency for the BGT60LTR11 device.
 *
 * This function builds a 16-bit SPI word using the FCW field in REG05_PLL_CFG2
 * and writes it to the device.
 *
 * @param frequency A value of type Reg05_pll_frequency_word_t.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mtb_s2go_radar_bgt60ltr11_frequency(Reg05_pll_frequency_word_t frequency);

/**
 * @brief Set the baseband PGA gain for the BGT60LTR11 device.
 *
 * This function builds a 16‐bit SPI word using the CTRL_GAIN field in REG09_BB
 * and writes it to the device.
 *
 * @param PGA_gain A value of type Reg09_baseband_ctrl_t.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mtb_s2go_radar_bgt60ltr11_PGA_gain(Reg09_baseband_ctrl_t PGA_gain);

/**
 * @brief Initialize the BGT60LTR11 device in pulsed mode.
 *
 * This function configures various registers for pulsed mode operation.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mtb_s2go_radar_bgt60ltr11_pulsed_mode_init(void);
esp_err_t mtb_s2go_radar_bgt60ltr11_pulsed_mode_init2(void);
/**
 * @brief Enable soft reset for the BGT60LTR11 device.
 *
 * This function sets bit 15 in register 0x0F to trigger a soft reset and
 * loops until the reset is completed (i.e. register 0x00 equals 0 and bit 13 in register 0x38 is set).
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mtb_s2go_radar_bgt60ltr11_enable_soft_reset(void);

esp_err_t mtb_s2go_radar_bgt60ltr11_set_miso_drv(Reg15_spi_force_miso_driver_t miso_drv);

esp_err_t bgt60l_spi_deinit(void);

typedef enum
{
    XENSIV_OK         =  0,       /**< No error */
    XENSIV_INTF_ERROR = -1,       /**< Interface error */
    XENSIV_CONF_ERROR = -2,       /**< Configuration error */
    XENSIV_READ_ERROR = -3,       /**< Read error */
    XENSIV_WRITE_ERROR= -4        /**< Write error */
} xensiv_s2go_radar_bgt60ltr11_error_t;


// Example register definitions
#define S2GO_RADAR_BGT60LTR11_REG_REG00               0u
#define S2GO_RADAR_BGT60LTR11_REG_REG01               1u
#define S2GO_RADAR_BGT60LTR11_REG_REG02               2u
#define S2GO_RADAR_BGT60LTR11_REG_REG03               3u    /* Reserved */
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1      4u
#define S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2      5u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3      6u
#define S2GO_RADAR_BGT60LTR11_REG_REG07               7u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV           8u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB            9u
#define S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME     10u
#define S2GO_RADAR_BGT60LTR11_REG_REG11              11u  /* Reserved */
#define S2GO_RADAR_BGT60LTR11_REG_REG12              12u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1        13u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2        14u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL      15u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0         34u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1         35u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT     36u
#define S2GO_RADAR_BGT60LTR11_REG_REG38_ADC_RESULT   38u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES      55u
#define S2GO_RADAR_BGT60LTR11_REG_REG56_CHIP_STAT    56u

#define S2GO_RADAR_BGT60LTR11_NUMBER_REGISTERS 						57
#define BIT_READ 											0
#define BIT_WRITE 											1

#define S2GO_RADAR_BGT60LTR11_REG_REG00								0u
#define S2GO_RADAR_BGT60LTR11_REG_REG01								1u
#define S2GO_RADAR_BGT60LTR11_REG_REG02								2u
#define S2GO_RADAR_BGT60LTR11_REG_REG03								3u	/* Reserved */
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1						4u
#define S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2						5u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3						6u
#define S2GO_RADAR_BGT60LTR11_REG_REG07								7u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV							8u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB								9u
#define S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME						10u
#define S2GO_RADAR_BGT60LTR11_REG_REG11								11u  /* Reserved */
#define S2GO_RADAR_BGT60LTR11_REG_REG12								12u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1							13u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2							14u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL						15u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0							34u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1							35u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT						36u
#define S2GO_RADAR_BGT60LTR11_REG_REG38_ADC_RESULT						38u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES						55u
#define S2GO_RADAR_BGT60LTR11_REG_REG56_CHIP_STAT						56u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG0 */
#define S2GO_RADAR_BGT60LTR11_REG_REG00_RXBUF_EN_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_RXBUF_EN_msk					0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_LNA_EN_pos						1u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_LNA_EN_msk						0x0002u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_MIXI_EN_pos					2u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_MIXI_EN_msk					0x0004u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_MIXQ_EN_pos					3u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_MIXQ_EN_msk					0x0008u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_TXBUF_EN_pos					4u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_TXBUF_EN_msk					0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_MPA_EN_pos						5u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_MPA_EN_msk						0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_EN_pos						8u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_EN_msk						0x0100u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_ACTIVE_pos					9u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_ACTIVE_msk					0x0200u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_CLK_GATE_EN_pos			10u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_CLK_GATE_EN_msk			0x0400u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_OPEN_LOOP_pos				11u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_PLL_OPEN_LOOP_msk				0x0800u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_VCO_EN_pos						12u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_VCO_EN_msk						0x1000u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_VCOBUF_EN_pos					13u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_VCOBUF_EN_msk					0x2000u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_CLEAR_ALL_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG00_CLEAR_ALL_msk					0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG01 */
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_SAMPLE_EN_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_SAMPLE_EN_msk				0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_AMP_EN_pos					1u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_AMP_EN_msk					0x0002u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_STRUP_HP_pos				2u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_STRUP_HP_msk				0x0004u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_CLK_CHOP_EN_pos				4u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_CLK_CHOP_EN_msk				0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_BOOST_DIS_pos				5u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_BOOST_DIS_msk				0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_DIG_DET_EN_pos				7u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_BB_DIG_DET_EN_msk				0x0080u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_QS_RD_EN_pos					8u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_QS_RD_EN_msk					0x0100u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_DIV_BIAS_EN_pos				12u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_DIV_BIAS_EN_msk				0x1000u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_CLEAR_ALL_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG01_CLEAR_ALL_msk					0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG02 */
#define S2GO_RADAR_BGT60LTR11_REG_REG02_THRS_pos 						0u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_THRS_msk 						0x1fffu
#define S2GO_RADAR_BGT60LTR11_REG_REG02_DIR_MODE_pos 					13u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_DIR_MODE_msk 					0x2000u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_APRT_pos						14u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_APRT_msk 						0x4000u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_HPRT_pos 						15u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_HPRT_msk 						0x8000u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_CLEAR_ALL_pos 					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG02_CLEAR_ALL_msk 					0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG03 */
#define S2GO_RADAR_BGT60LTR11_REG_REG03_RESERVED_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG03_RESERVED_msk					0xffffu
#define S2GO_RADAR_BGT60LTR11_REG_REG03_CLEAR_ALL_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG03_CLEAR_ALL_msk					0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1 */
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_PFD_RDT_SEL_pos		0u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_PFD_RDT_SEL_msk		0x0003u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_CP_MODE_pos			2u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_CP_MODE_msk			0x0004u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_CP_ICP_SEL_pos		3u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_CP_ICP_SEL_msk		0x0038u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_FBDIV_CNT_pos			6u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_FBDIV_CNT_msk			0x0040u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_XOSC_MODE_pos			7u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_XOSC_MODE_msk			0x0080u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_LF_R2_SEL_pos			8u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_LF_R2_SEL_msk			0x0100u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_CL_LOOP_PMODE_pos		9u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_CL_LOOP_PMODE_msk		0x0200u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_LF_R4_SEL_pos			10u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_LF_R4_SEL_msk			0x0400u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_LF_ISO_pos			11u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_LF_ISO_msk			0x0800u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_BIAS_DIS_pos			12u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_BIAS_DIS_msk			0x1000u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_RESERVED_pos			13u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_RESERVED_msk			0x2000u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_DFT_DMUX_pos			14u
#define S2GO_RADAR_BGT60LTR11_REG_REG04_PLL_CFG1_DFT_DMUX_msk			0xc000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2 */
#define S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2_FCW_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG05_PLL_CFG2_FCW_msk				0x0fffu

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3 */
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3_LD_EN_pos				11u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3_LD_EN_msk				0x0800u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3_LD_LEN_pos			12u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3_LD_LEN_msk			0x1000u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3_LD_TW_SEL_pos			13u
#define S2GO_RADAR_BGT60LTR11_REG_REG06_PLL_CFG3_LD_TW_SEL_msk			0xe000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG07 */
#define S2GO_RADAR_BGT60LTR11_REG_REG07_MPA_CTRL_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_MPA_CTRL_msk					0x0007u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_PD_EN_pos							3u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_PD_EN_msk							0x0008u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_MPA2SH_DLY_pos					4u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_MPA2SH_DLY_msk					0x0030u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_VCO2PLL_DLY_pos					6u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_VCO2PLL_DLY_msk					0x0040u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_DC_ON_PULSE_LEN_pos			8u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_DC_ON_PULSE_LEN_msk			0x0300u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_DC_REP_RATE_pos					10u
#define S2GO_RADAR_BGT60LTR11_REG_REG07_DC_REP_RATE_msk					0x0c00u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG08_DIV */
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_TESTMODE_EN_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_TESTMODE_EN_msk			0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_OUT_EN_pos					1u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_OUT_EN_msk					0x0002u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_SEL_pos						2u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_SEL_msk						0x000cu
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_CLEAR_ALL_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG08_DIV_CLEAR_ALL_msk				0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG09_BB */
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_CTRL_GAIN_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_CTRL_GAIN_msk				0x000fu
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_LPF_BW_pos					4u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_LPF_BW_msk					0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_CLK_CHOP_SEL_pos			5u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_CLK_CHOP_SEL_msk			0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_HP_RES_pos					6u
#define S2GO_RADAR_BGT60LTR11_REG_REG09_BB_HP_RES_msk					0x00c0u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME */
#define S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME_HOLD_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME_HOLD_msk				0xffffu
#define S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME_CLEAR_ALL_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG10_HOLDTIME_CLEAR_ALL_msk			0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE */
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BITE_EN_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BITE_EN_msk			0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BITE_CTRL_pos			1u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BITE_CTRL_msk			0x000eu
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BITE_PD_EN_pos			4u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BITE_PD_EN_msk			0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BB_AMUX_EN_pos			5u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BB_AMUX_EN_msk			0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BB_AMUX_CTRL_pos		6u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_BB_BITE_BB_AMUX_CTRL_msk		0x00c0u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_CLEAR_ALL_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG12_CLEAR_ALL_msk					0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1 */
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_PRT_MULT_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_PRT_MULT_msk				0x0003u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_MEAN_WIN_LEN_pos			2u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_MEAN_WIN_LEN_msk			0x001cu
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_PHASE_WIN_LEN_pos		5u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_PHASE_WIN_LEN_msk		0x00e0u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_CLEAR_ALL_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG13_ALGO1_CLEAR_ALL_msk			0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2 */
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_PHASE_THRS_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_PHASE_THRS_msk			0x0003u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_PULSE_MON_pos			2u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_PULSE_MON_msk			0x0004u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_AUTOBLIND_pos			3u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_AUTOBLIND_msk			0x0008u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_SWAP_IQ_pos				4u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_SWAP_IQ_msk				0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_HOLD_X32_pos				5u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_HOLD_X32_msk				0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_DIR_KEEP_pos				6u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_DIR_KEEP_msk				0x0040u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_DIR_HYST_DIS_pos			7u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_DIR_HYST_DIS_msk			0x0080u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_THRS_OFFSET_pos			8u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_THRS_OFFSET_msk			0xff00u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_CLEAR_ALL_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG14_ALGO2_CLEAR_ALL_msk			0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL */
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_STAT_MUX_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_STAT_MUX_msk			0x000fu
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_DIR_POL_pos			4u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_DIR_POL_msk			0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_MOT_POL_pos			5u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_MOT_POL_msk			0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_MISO_DRV_pos			6u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_MISO_DRV_msk			0x0040u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_ADC_MOM_pos			7u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_ADC_MOM_msk			0x0080u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_FASTMODE_pos			8u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_FASTMODE_msk			0x0100u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_DIR_C2_1_pos			9u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_DIR_C2_1_msk			0x0600u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_FAST_PHASE_pos			11u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_FAST_PHASE_msk			0x0800u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_START_CW_pos			12u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_START_CW_msk			0x1000u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_CLK_EXT_DIS_pos		13u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_CLK_EXT_DIS_msk		0x2000u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_START_PM_pos			14u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_START_PM_msk			0x4000u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_SOFT_RESET_pos			15u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_SOFT_RESET_msk			0x8000u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_CLEAR_ALL_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG15_DIGCTRL_CLEAR_ALL_msk			0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0 */
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_CLK_EN_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_CLK_EN_msk				0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_BANDGAP_EN_pos			1u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_BANDGAP_EN_msk			0x0002u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_ADC_EN_pos				2u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_ADC_EN_msk				0x0004u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_CLEAR_ALL_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG34_ADC0_CLEAR_ALL_msk				0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1 */
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_CH_NUM_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_CH_NUM_msk				0x000fu
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_CH_NUM_ALL_pos			4u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_CH_NUM_ALL_msk			0x0010u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_LV_GAIN_pos				7u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_LV_GAIN_msk				0x0080u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_CLEAR_ALL_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG35_ADC1_CLEAR_ALL_msk				0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT */
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT_BG_UP_pos				0u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT_BG_UP_msk				0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT_ADC_READY_pos			1u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT_ADC_READY_msk			0x0002u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT_CLEAR_ALL_pos			0u
#define S2GO_RADAR_BGT60LTR11_REG_REG36_ADC_STAT_CLEAR_ALL_msk			0x0000u

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG38_ADC_RESULT */
#define S2GO_RADAR_BGT60LTR11_REG_REG38_ADC_RESULT_pos					0u
#define S2GO_RADAR_BGT60LTR11_REG_REG38_ADC_RESULT_msk					0x03ffu

/* Fields of register S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES */
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_DIRECTION_OUTPUT_pos	0u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_DIRECTION_OUTPUT_msk	0x0001u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_ADAPTIVE_PRT_pos		1u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_ADAPTIVE_PRT_msk		0x0002u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_BB_HP_RES_pos			2u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_BB_HP_RES_msk			0x0004u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_BB_CTRL_GAIN_pos		3u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_BB_CTRL_GAIN_msk		0x0018u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_DC_REP_RATE_pos		5u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_DC_REP_RATE_msk		0x0020u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_DC_ON_PULSE_LEN_pos	6u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_DC_ON_PULSE_LEN_msk	0x0040u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_MPA_CTRL_pos			7u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_MPA_CTRL_msk			0x0180u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_LD_TW_SEL_pos		9u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_LD_TW_SEL_msk		0x0600u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_LD_LEN_pos			11u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_LD_LEN_msk			0x0800u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_LF_R4_SEL_pos		12u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_LF_R4_SEL_msk		0x1000u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_CP_ICP_SEL_pos		13u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_CP_ICP_SEL_msk		0x2000u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_CP_MODE_pos		14u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_CP_MODE_msk		0x4000u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_JAPAN_MODE_pos		15u
#define S2GO_RADAR_BGT60LTR11_REG_REG55_E_FUSES_PLL_JAPAN_MODE_msk		0x8000u

#define S2GO_RADAR_BGT60LTR11_SET(REGISTER, FIELD, X) \
    ((((unsigned short)(X) << S2GO_RADAR_BGT60LTR11_REG_##REGISTER##_##FIELD##_pos) & S2GO_RADAR_BGT60LTR11_REG_##REGISTER##_##FIELD##_msk))

 
    
#ifdef __cplusplus
}
#endif

#endif // BGT60L_SPI_H