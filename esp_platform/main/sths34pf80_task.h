#ifndef I2C_STHS34PF80_H
#define I2C_STHS34PF80_H

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include "sths34pf80_reg.h"  // Add this include so that stmdev_ctx_t is defined



esp_err_t i2c_master_init(void);
esp_err_t force_scl_high(void);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
/** Optional (may be required by driver) **/
void platform_delay(uint32_t millisec);
/** Initialize the sensor context for the STHS34PF80 sensor **/
void sths34pf80_init(void);
/** Presence detection (RTOS task can call this) **/
void sths34pf80_tmos_presence_detection(void);
void read_and_print_ambient(void);
void configure_tmos_int(void);
void sensor_init_parameters(stmdev_ctx_t *dev_ctx);
void disable_sths34pf80_interrupt_for_ota(void);



#endif /* I2C_STHS34PF80_H */