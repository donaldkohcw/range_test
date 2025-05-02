#ifndef GPIO_WAKEUP_H
#define GPIO_WAKEUP_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EXT1_WAKEUP_NUM         9


esp_err_t example_register_gpio_wakeup(void);
esp_err_t example_register_gpio_wakeup_high(void);
esp_err_t example_register_ext1_gpio_wakeup(void);
esp_err_t initialize_gpio(int gpio_num, bool pull_up, bool pull_down, gpio_int_type_t intr_type);



void example_wait_gpio_inactive(void);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_WAKEUP_H */