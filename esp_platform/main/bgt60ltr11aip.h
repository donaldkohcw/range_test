#ifndef BGT60LTR11AIP_H
#define BGT60LTR11AIP_H

/* Configure the GPIO pins for the radar shield */
#define RADAR_TD_GPIO    11  // Changed to GPIO2 for the TD pin
#define RADAR_PD_GPIO  3  // Changed to GPIO3 for the PD pin

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool target_detected;
extern volatile int radar_td_rise_edge;


void configure_radar_gpio(void);
bool read_radar_pd(void);
void disable_bgt60ltr_interrupt_for_ota(void);


#ifdef __cplusplus
}
#endif

#endif // BGT60LTR11AIP_H