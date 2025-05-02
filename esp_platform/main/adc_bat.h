#ifndef ADC_BAT_H
#define ADC_BAT_H

#include <stdint.h>

uint32_t read_battery_voltage(void);
void battery_task(void *pvParameter);


#endif // ADC_BAT_H