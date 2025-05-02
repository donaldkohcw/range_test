#pragma once

#include "esp_log.h"
//#include "esp_nimble_hci.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"


static const char device_name[] = "esp32";

void advertise();
void reset_cb(int reason);
void sync_cb(void);
void host_task(void *param);
void shutdown_ble_and_sleep(void);
