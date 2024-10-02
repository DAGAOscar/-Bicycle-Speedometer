#include "ble_data.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_hs_adv.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BLE"

// Données externes
extern float total_distance;  // Distance (Długość)
extern float avg_speed;       // Vitesse moyenne (VitMoyen)
extern float max_speed;       // Vitesse maximale (VitMax)
extern float min_speed;       // Vitesse minimale (VitMin)
extern char sensor_speed_data[8]; // String containing the current speed

static int device_read (uint16_t conn_handle, uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint32_t *data = (uint32_t*)arg;
    char data_str[8];
    snprintf(data_str, sizeof(data_str), "%u", *data);
    os_mbuf_append(ctxt->om, data_str, strlen(data_str));
    return 0; 
}

static int device_read_speed (uint16_t conn_handle, uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    os_mbuf_append(ctxt->om, sensor_speed_data, strlen(sensor_speed_data));
    return 0; 
}

static int device_read_distance (uint16_t conn_handle, uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    float *data = (float*)arg;
    static char distance_str[16];
    snprintf(distance_str, sizeof(distance_str), "%.2f", *data);
    os_mbuf_append(ctxt->om, distance_str, strlen(distance_str));
    return 0; 
}

static int device_read_average_speed (uint16_t conn_handle, uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    float *data = (float*)arg;
    static char avg_speed_str[16];
    snprintf(avg_speed_str, sizeof(avg_speed_str), "%.2f", *data);
    os_mbuf_append(ctxt->om, avg_speed_str, strlen(avg_speed_str));
    return 0; 
}

static int device_read_max_speed (uint16_t conn_handle, uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    float *data = (float*)arg;
    static char max_speed_str[16];
    snprintf(max_speed_str, sizeof(max_speed_str), "%.2f", *data);
    os_mbuf_append(ctxt->om, max_speed_str, strlen(max_speed_str));
    return 0; 
}

static int device_read_min_speed (uint16_t conn_handle, uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    float *data = (float*)arg;
    static char min_speed_str[16];
    snprintf(min_speed_str, sizeof(min_speed_str), "%.2f", *data);
    os_mbuf_append(ctxt->om, min_speed_str, strlen(min_speed_str));
    return 0; 
}

static int device_write (uint16_t conn_handle, uint16_t attr_handle, 
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const char *value = (const char*)os_mbuf_extend(ctxt->om, ctxt->om->om_len);
    if (value) {
        ESP_LOGI(TAG, "Received data: %s", value);
    }
    return 0;
}

struct ble_gatt_chr_def gatt_char_defs[] = {
    {.uuid = BLE_UUID16_DECLARE(0xFEF4),
     .flags = BLE_GATT_CHR_F_READ,
     .access_cb = device_read,
     .arg = &sensor_data
    },
    {
     .uuid = BLE_UUID16_DECLARE(0xFEF5),
     .flags = BLE_GATT_CHR_F_READ,
     .access_cb = device_read_speed,
     .arg = NULL
    },
    {
     .uuid = BLE_UUID16_DECLARE(0xFEF6),
     .flags = BLE_GATT_CHR_F_READ,
     .access_cb = device_read_distance,
     .arg = &total_distance
    },
    {
     .uuid = BLE_UUID16_DECLARE(0xFEF7),
     .flags = BLE_GATT_CHR_F_READ,
     .access_cb = device_read_average_speed,
     .arg = &avg_speed
    },
    {
     .uuid = BLE_UUID16_DECLARE(0xFEF8),
     .flags = BLE_GATT_CHR_F_READ,
     .access_cb = device_read_max_speed,
     .arg = &max_speed
    },
    {
     .uuid = BLE_UUID16_DECLARE(0xFEF9),
     .flags = BLE_GATT_CHR_F_READ,
     .access_cb = device_read_min_speed,
     .arg = &min_speed
    },
    {.uuid = BLE_UUID16_DECLARE(0xDEAD),
     .flags = BLE_GATT_CHR_F_WRITE,
     .access_cb = device_write
    },
    {0}
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29),
                .access_cb = device_read,
                .flags = BLE_GATT_CHR_F_READ,
                .arg = NULL
            },
            {0} // No more characteristics in this service
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFF0),
        .characteristics = gatt_char_defs,
    },
    {0} // No more services
};

void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    // Set the advertisement data
    memset(&fields, 0, sizeof(fields));
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)"ESP32-BLE-SPEED";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    // Begin advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertisement; rc=%d", rc);
        return;
    }
}

void bleprph_host_task(void *param) {
    nimble_port_run(); // This function will return only when nimble_port_stop() is called.
    nimble_port_freertos_deinit();
}

void ble_init(void) {
    esp_nimble_hci_and_controller_init();

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    ble_svc_gap_device_name_set("ESP32-BLE-SPEED");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    nimble_port_freertos_init(bleprph_host_task);
}
