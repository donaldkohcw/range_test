#include "gap.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "ota_event_bits.h"



extern EventGroupHandle_t ble_event_group;
static const char *TAG_GAP = "gap";
uint8_t addr_type;
static uint16_t conn_handle = 0; // Declare the connection handle


int gap_event_handler(struct ble_gap_event *event, void *arg);

void advertise() {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    // flags: discoverability + BLE only
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // include power levels
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // include device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG_GAP, "Error setting advertisement data: rc=%d", rc);
        return;
    }

    // start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG_GAP, "Error enabling advertisement data: rc=%d", rc);
        return;
    }
}

void reset_cb(int reason) {
    ESP_LOGE(TAG_GAP, "BLE reset callback triggered, reason = %d", reason);
    // Signal that BLE is disconnected
    xEventGroupClearBits(ble_event_group, BLE_CONNECTED_BIT);
}

void sync_cb(void) {
    // determine best address type
    ble_hs_id_infer_auto(0, &addr_type);

    // start advertising
    advertise();

    ESP_LOGI(TAG_GAP, "BLE sync callback triggered");
}

int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            // A new connection was established or a connection attempt failed
            ESP_LOGI(TAG_GAP, "GAP: Connection %s: status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle; // Save the connection handle

                // Connection established
                xEventGroupSetBits(ble_event_group, BLE_CONNECTED_BIT);

                // Log the current connection interval
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
                    ESP_LOGI(TAG_GAP, "Connection interval: %d units (%.2f ms)",
                             desc.conn_itvl, desc.conn_itvl * 1.25);
                } else {
                    ESP_LOGE(TAG_GAP, "Failed to retrieve connection description");
                }

                // Update connection parameters to lower the interval
                struct ble_gap_upd_params params = {
                    .itvl_min = 6,  // Minimum connection interval (7.5ms)
                    .itvl_max = 6,  // Maximum connection interval (7.5ms)
                    .latency = 0,   // Slave latency
                    .supervision_timeout = 100, // Supervision timeout (1 second)
                };
                int rc = ble_gap_update_params(event->connect.conn_handle, &params);
                if (rc != 0) {
                    ESP_LOGE(TAG_GAP, "Failed to update connection parameters: rc=%d", rc);
                } else {
                    ESP_LOGI(TAG_GAP, "Connection parameters updated to lower interval.");
                }


            } else {
                // Connection attempt failed; resume advertising
                advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG_GAP, "GAP: Disconnect: reason=%d\n",
                     event->disconnect.reason);

            // Restart the ESP on disconnection during OTA
            // Connection terminated; clear the connection bit and resume advertising
            xEventGroupClearBits(ble_event_group, BLE_CONNECTED_BIT);
            xEventGroupSetBits(ble_event_group, OTA_DISCONNECT_BIT);
            ESP_LOGI(TAG_GAP, "BLE_GAP_EVENT_DISCONNECT.");
            //advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG_GAP, "GAP: adv complete");
            advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG_GAP, "GAP: Subscribe: conn_handle=%d",
                     event->connect.conn_handle);
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG_GAP, "GAP: MTU update: conn_handle=%d, mtu=%d",
                     event->mtu.conn_handle, event->mtu.value);
            break;
    }

    return 0;
}

void host_task(void *param) {
    // returns only when nimble_port_stop() is executed
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void shutdown_ble_and_sleep(void)
{
    ble_gap_adv_stop(); // Stop advertising
    ESP_LOGI(TAG_GAP, "after adv stop...");

    // Close and deinitialize NimBLE stack
    nimble_port_stop();
    ESP_LOGI(TAG_GAP, "after nimble stop...");

    nimble_port_deinit();
    ESP_LOGI(TAG_GAP, "after nimble deinit...");

    esp_sleep_enable_timer_wakeup(1000);  // 1 ms deep sleep
    esp_deep_sleep_start();               // Full power cycle
}