#ifndef OTA_EVENT_BITS_H
#define OTA_EVENT_BITS_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"



#define BLE_CONNECTED_BIT BIT0
#define OTA_UPDATE_REQUEST_BIT BIT1
#define OTA_UPDATE_DONE_BIT BIT2
#define OTA_DISCONNECT_BIT BIT3
#define TRANSMIT_COMPLETE_BIT BIT4
#define TRANSMIT_START_BIT BIT5




extern EventGroupHandle_t ble_event_group;

#endif // OTA_EVENT_BITS_H