#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define SERVO_GPIO       14
#define SERVO_FREQ_HZ    50
#define SERVO_TIMER_RES  LEDC_TIMER_13_BIT
#define SERVO_NEUTRAL_US 1500   /* MG995 stop */
#define SERVO_RUN_US     2000   /* speed/direction while dispensing */
#define SERVO_RUN_MS     2000   /* how long to run per "ON" command */

static const char *TAG = "feeder";
static QueueHandle_t dispense_queue;

/* ---------------- Servo (MG995 continuous-rotation) via LEDC ---------------- */

static void servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_TIMER_RES,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t channel_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel_cfg);
}

static void servo_set_pulse_us(uint32_t pulse_us)
{
    uint32_t max_duty = (1 << SERVO_TIMER_RES) - 1;
    uint32_t duty = (pulse_us * max_duty) / (1000000 / SERVO_FREQ_HZ);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void servo_stop(void) { servo_set_pulse_us(SERVO_NEUTRAL_US); }
static void servo_run(void)  { servo_set_pulse_us(SERVO_RUN_US); }

/* ---------------- Dispense task: runs the servo for SERVO_RUN_MS on signal ---------------- */

static void dispense_task(void *arg)
{
    uint8_t signal;
    while (1) {
        if (xQueueReceive(dispense_queue, &signal, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Dispensing...");
            servo_run();
            vTaskDelay(pdMS_TO_TICKS(SERVO_RUN_MS));
            servo_stop();
            ESP_LOGI(TAG, "Done");
        }
    }
}

/* ---------------- BLE GATT server ---------------- */

static const ble_uuid128_t feeder_svc_uuid =
    BLE_UUID128_INIT(0x4f, 0xaf, 0xc2, 0x01, 0x1f, 0xb5, 0x45, 0x9e,
                      0x8f, 0xcc, 0xc5, 0xc9, 0xc3, 0x31, 0x91, 0x4b);

static const ble_uuid128_t feeder_chr_uuid =
    BLE_UUID128_INIT(0xbe, 0xb5, 0x48, 0x3e, 0x36, 0xe1, 0x46, 0x88,
                      0xb7, 0xf5, 0xea, 0x07, 0x36, 0x1b, 0x26, 0xa8);

static int feeder_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    char buf[32] = {0};
    uint16_t len = 0;
    uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
    if (om_len >= sizeof(buf)) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "Received: \"%s\"", buf);

    if (strcmp(buf, "ON") == 0) {
        uint8_t signal = 1;
        xQueueSend(dispense_queue, &signal, 0);
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &feeder_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &feeder_chr_uuid.u,
                .access_cb = feeder_chr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            { 0 },
        },
    },
    { 0 },
};

static uint8_t own_addr_type;

static void feeder_advertise(void);

static int feeder_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        if (event->connect.status != 0) {
            feeder_advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; restarting advertising");
        feeder_advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        feeder_advertise();
        return 0;
    default:
        return 0;
    }
}

static void feeder_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                            &adv_params, feeder_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    }
}

static void feeder_on_sync(void)
{
    int rc;
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }
    feeder_advertise();
}

static void feeder_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void feeder_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int gatt_svr_init(void)
{
    int rc;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void app_main(void)
{
    printf("Hello world\n");

    servo_init();
    servo_stop();

    dispense_queue = xQueueCreate(4, sizeof(uint8_t));
    xTaskCreate(dispense_task, "dispense_task", 4096, NULL, 5, NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = feeder_on_reset;
    ble_hs_cfg.sync_cb = feeder_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    gatt_svr_init();

    ble_svc_gap_device_name_set("SparkCatFeeder");

    nimble_port_freertos_init(feeder_host_task);
}
