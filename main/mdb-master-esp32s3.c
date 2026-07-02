/*
 * VMflow.xyz
 *
 * mdb-master-esp32s3.c - MDB controller <-> target bridge
 *
 * Two independent MDB ports:
 *  - controller port (UART2 hardware, master): drives physical peripherals
 *    (external cashless, coin changer, bill validator).
 *  - target port (bit-banged GPIO, slave): emulates cashless, coin changer
 *    and bill validator toward the vending machine's real VMC.
 *
 * Coin changer and bill validator are bridged as pure repeaters: real
 * events detected on the controller port are relayed as MDB poll events on
 * the target port, and vice-versa for type-enable/dispense/escrow commands.
 *
 * Cashless (address 0x10) is arbitrated: the target port always answers as
 * a single cashless device, but its session can be driven by either the
 * VMflow internal payment engine or an external cashless device wired on
 * the controller port (e.g. Nayax). Whichever opens a session first owns
 * it until the session ends; the other source is parked meanwhile.
 *
 * The VMflow internal payment engine (cashless_internal_task) bridges the
 * cashless arbitration queues to:
 *  - BLE (NimBLE GATT, nimble.c): phone app vend/session channel + passive
 *    scan foot-traffic counter ("paxcounter").
 *  - WiFi (STA, plain credentials set over BLE) as the sole uplink.
 *  - MQTT (mqtt.vmflow.xyz): telemetry publish + signed remote-RPC command
 *    channel.
 *  - RPC/HMAC signing (rpc-auth.c): authenticates every RPC command and
 *    signs outbound telemetry with the device passkey.
 *  - OTA (esp_https_ota): pulls a signed release binary over HTTPS,
 *    triggered by the "ota" RPC command.
 */

#include <esp_log.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <rom/ets_sys.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_app_desc.h>
#include <esp_https_ota.h>
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <mqtt_client.h>
#include <esp_sntp.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include "led_strip.h"
#include "nimble.h"
#include "rpc-auth.h"

#define TAG "mdb_bridge"

#define pin_mdb_target_rx      GPIO_NUM_4
#define pin_mdb_target_tx      GPIO_NUM_5
#define pin_mdb_controller_rx  GPIO_NUM_1
#define pin_mdb_controller_tx  GPIO_NUM_2
#define pin_mdb_led            GPIO_NUM_48 // LED to indicate MDB state

// Functions for scale factor conversion
#define TO_SCALE_FACTOR(p, scale_to, dec_to) (p / scale_to / pow(10, -(dec_to) ))               // Converts to scale factor
#define FROM_SCALE_FACTOR(p, scale_from, dec_from) (p * scale_from * pow(10, -(dec_from) ))     // Converts from scale factor

#define ACK     0x00  // Acknowledgment / Checksum correct;
#define RET     0xAA  // Retransmit the previously sent data. Only the VMC can transmit this byte;
#define NAK     0xFF  // Negative acknowledge.

#define BIT_MODE_SET    0b100000000
#define BIT_ADD_SET     0b011111000
#define BIT_CMD_SET     0b000000111

#define ADDR_CASHLESS   CONFIG_CASHLESS_DEVICE_ADDRESS
#define ADDR_CHANGER    0x08
#define ADDR_VALIDATOR  0x30

// Cashless Device (peripheral 0x10) command set
enum CASHLESS_CMD {
    CSHL_RESET     = 0x00,
    CSHL_SETUP     = 0x01,
    CSHL_POLL      = 0x02,
    CSHL_VEND      = 0x03,
    CSHL_READER    = 0x04,
    CSHL_EXPANSION = 0x07,
};

// Coin Changer (peripheral 0x08) command set - note POLL/TUBE differ from cashless
enum CHANGER_CMD {
    CHGR_RESET       = 0x00,
    CHGR_SETUP       = 0x01,
    CHGR_TUBE_STATUS = 0x02,
    CHGR_POLL        = 0x03,
    CHGR_COIN_TYPE   = 0x04,
    CHGR_DISPENSE    = 0x05,
    CHGR_EXPANSION   = 0x07,
};

// Bill Validator (peripheral 0x30) command set
enum VALIDATOR_CMD {
    VLD_RESET     = 0x00,
    VLD_SETUP     = 0x01,
    VLD_SECURITY  = 0x02,
    VLD_POLL      = 0x03,
    VLD_BILL_TYPE = 0x04,
    VLD_ESCROW    = 0x05,
    VLD_STACKER   = 0x06,
    VLD_EXPANSION = 0x07,
};

typedef enum MACHINE_STATE {
    INACTIVE_STATE, DISABLED_STATE, ENABLED_STATE, IDLE_STATE, VEND_STATE
} machine_state_t;

// Cashless Device (0x10) - snapshot of the physical/external device, written
// only by the controller task, read by the target task to mirror scale
// factor / decimal places at SETUP time.
typedef struct {
    uint8_t feature_level;
    uint16_t country_code;
    uint8_t scale_factor;
    uint8_t decimal_places;
    uint8_t response_time_sec;
    uint8_t miscellaneous;

    uint8_t poll_fail_count;
    machine_state_t machine_state;
} cashless_t;

// Coin Changer (0x08) - snapshot of the physical device.
typedef struct {
    uint8_t feature_level;
    uint16_t country_code;
    uint8_t scale_factor;
    uint8_t decimal_places;
    uint16_t coin_type_routing;
    uint16_t tube_full_status;
    uint8_t tube_counts[16];
    uint8_t coin_credit[16];   // per-type credit (scaled units)
    uint16_t credit;           // accumulated deposited credit (diagnostics only)

    uint8_t poll_fail_count;
    machine_state_t machine_state;
} changer_t;

// Bill Validator (0x30) - snapshot of the physical device.
typedef struct {
    uint8_t feature_level;
    uint16_t country_code;
    uint16_t scale_factor;
    uint8_t decimal_places;
    uint16_t bill_stacker_capacity;
    uint16_t bill_security_levels;
    uint8_t escrow_capability;
    uint8_t bill_credit[16];
    uint16_t credit;            // accumulated deposited credit (diagnostics only)
    uint16_t stacker_count;     // bills currently in the stacker (from VLD_STACKER; no per-type value)
    bool stacker_full;

    uint8_t poll_fail_count;
    machine_state_t machine_state;
} validator_t;

static changer_t   reader0x08 = { .machine_state = INACTIVE_STATE };
static cashless_t  reader0x10 = { .machine_state = INACTIVE_STATE };
static validator_t reader0x30 = { .machine_state = INACTIVE_STATE };

// Fixed defaults used by the target-side cashless emulation when no
// external physical cashless has been detected on the controller port yet
// (i.e. only the VMflow internal engine is available).
#define CASHLESS_DEFAULT_SCALE_FACTOR   CONFIG_MDB_SCALE_FACTOR
#define CASHLESS_DEFAULT_DECIMAL_PLACES CONFIG_MDB_DECIMAL_PLACES

//------------------------------------------------------------------------//
// Cashless bridge: arbitration between the VMflow internal engine (added
// separately) and an external physical cashless wired on the controller
// port. Whichever opens a session first owns it until it ends.
//------------------------------------------------------------------------//

typedef enum { CASHLESS_SRC_NONE, CASHLESS_SRC_INTERNAL, CASHLESS_SRC_EXTERNAL } cashless_src_t;

typedef enum {
    CSHL_EVT_BEGIN_SESSION,
    CSHL_EVT_VEND_APPROVED,
    CSHL_EVT_VEND_DENIED,
    CSHL_EVT_SESSION_END,
    CSHL_EVT_SESSION_CANCEL,
} cashless_to_target_evt_type_t;

typedef struct {
    cashless_src_t source;
    cashless_to_target_evt_type_t type;
    uint16_t value; // funds_available (BEGIN_SESSION) or item_price (VEND_APPROVED)
} cashless_to_target_evt_t;

typedef enum {
    CSHL_REQ_VEND_REQUEST,
    CSHL_REQ_VEND_CANCEL,
    CSHL_REQ_VEND_SUCCESS,
    CSHL_REQ_VEND_FAILURE,
    CSHL_REQ_SESSION_COMPLETE,
} cashless_from_target_evt_type_t;

typedef struct {
    cashless_from_target_evt_type_t type;
    uint16_t item_price;
    uint16_t item_number;
} cashless_from_target_evt_t;

static QueueHandle_t cashless_to_target_queue;                 // controller + internal engine -> target
static QueueHandle_t cashless_from_target_to_controller_queue; // target -> controller (consumed only while source==EXTERNAL)

// Non-static: the VMflow internal payment engine (BLE module, added
// separately) reads vend requests meant for it from this queue and pushes
// its own session/vend events into cashless_to_target_queue tagged
// CASHLESS_SRC_INTERNAL.
QueueHandle_t cashless_from_target_to_internal_queue;

static volatile cashless_src_t g_active_cashless_source = CASHLESS_SRC_NONE;

//------------------------------------------------------------------------//
// Coin changer / bill validator bridge: pure repeater, no arbitration.
//------------------------------------------------------------------------//

typedef struct { uint8_t coin_type; uint8_t tube_count; } coin_deposit_evt_t;
static QueueHandle_t coin_to_target_queue;    // controller -> target (coin inserted)

typedef struct {
    enum { COIN_REQ_TYPE_ENABLE, COIN_REQ_DISPENSE } type;
    uint16_t coin_enable;
    uint16_t dispense_enable;
    uint8_t dispense_value;
} coin_from_target_evt_t;
static QueueHandle_t coin_from_target_queue;  // target -> controller

typedef struct { uint8_t bill_type; } bill_stack_evt_t;
static QueueHandle_t bill_to_target_queue;    // controller -> target (bill stacked)

typedef struct {
    enum { BILL_REQ_TYPE_ENABLE, BILL_REQ_ESCROW } type;
    uint16_t bill_enable;
    uint16_t bill_escrow_enable;
    uint8_t escrow_command;
} bill_from_target_evt_t;
static QueueHandle_t bill_from_target_queue;  // target -> controller

//------------------------------------------------------------------------//
// VMflow internal payment engine: BLE (phone app + PAX scan), WiFi, MQTT,
// RPC/HMAC signing, OTA. This is CASHLESS_SRC_INTERNAL's implementation.
//------------------------------------------------------------------------//

enum BIT_STATUS {
    BIT_STATUS_MQTT     = (1 << 0),
    BIT_STATUS_PASSKEY  = (1 << 1),
    BIT_STATUS_DOMAIN   = (1 << 2),
    BIT_STATUS_TRIGGER  = (1 << 3),
    MASK_STATUS_INSTALLED = (BIT_STATUS_PASSKEY | BIT_STATUS_DOMAIN),
};

enum BIT_WIFI {
    BIT_STA_GOT_IP  = (1 << 0),
    BIT_STA_LOST_IP = (1 << 1),
};

static EventGroupHandle_t xLedEventGroup;
static EventGroupHandle_t xWifiEventGroup;

#define WIFI_BACKOFF_MIN_MS  5000
#define WIFI_BACKOFF_MAX_MS  300000
static uint32_t wifi_backoff_ms = WIFI_BACKOFF_MIN_MS;
static esp_timer_handle_t wifi_retry_timer;

static char my_subdomain[32];
#define PASSKEY_LEN 18
static char my_passkey[PASSKEY_LEN + 1];

static char s_ip_wifi[16] = "";

#define RPC_FRESHNESS_SEC   10
#define BLE_FRESHNESS_SEC   60

static esp_mqtt_client_handle_t mqtt_client = NULL;

static uint16_t last_sale_price = 0;
static uint16_t last_sale_item = 0;
static time_t   last_vend_success_time = 0;

// Big-endian (de)serialization helpers for the BLE wire payload.
static inline uint32_t read_u32(const uint8_t *p) {
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | p[3];
}
static inline uint16_t read_u16(const uint8_t *p) {
    return ((uint16_t) p[0] << 8) | p[1];
}
static inline void write_u32(uint8_t *p, uint32_t v) {
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}
static inline void write_u16(uint8_t *p, uint16_t v) {
    p[0] = v >> 8; p[1] = v;
}

/*
 * BLE wire payload (phone app) — 19 bytes:
 *   [0] CMD | [1-4] PRICE u32 | [5-6] ITEM u16 | [7-10] TIME u32 |
 *   [11-14] reserved=0 | [15-18] HMAC-SHA256(passkey, bytes 0-14)[:4]
 *
 * Price is carried on the wire in "cents" (scale=1, decimal=2) and
 * converted to/from the internal cashless engine's own MDB scale
 * (CASHLESS_DEFAULT_SCALE_FACTOR/DECIMAL_PLACES — the internal engine only
 * ever owns a session when no external physical cashless is present, see
 * the target task's SETUP handler).
 */
static esp_err_t ble_decode_with_passkey(uint16_t *item_price, uint16_t *item_number, uint8_t *payload) {
    unsigned char hmac[32];
    calculate_hmac((const char*) payload, 15, hmac);

    uint8_t diff = 0;
    for (int x = 0; x < 4; x++) {
        diff |= hmac[x] ^ payload[15 + x];
    }

    if (diff != 0) {
        return ESP_ERR_INVALID_CRC;
    }

    int32_t timestamp = read_u32(&payload[7]);

    time_t now = time(NULL);

    if (abs((int32_t) now - timestamp) > BLE_FRESHNESS_SEC) {
        return ESP_ERR_TIMEOUT;
    }

    int32_t item_price_32 = read_u32(&payload[1]);

    if (item_price)
        *item_price = TO_SCALE_FACTOR(FROM_SCALE_FACTOR(item_price_32, 1, 2), CASHLESS_DEFAULT_SCALE_FACTOR, CASHLESS_DEFAULT_DECIMAL_PLACES);

    if (item_number)
        *item_number = read_u16(&payload[5]);

    return ESP_OK;
}

static void ble_encode_with_passkey(uint8_t cmd, uint16_t item_price, uint16_t item_number, uint8_t *payload) {
    uint32_t item_price_32 = TO_SCALE_FACTOR(FROM_SCALE_FACTOR(item_price, CASHLESS_DEFAULT_SCALE_FACTOR, CASHLESS_DEFAULT_DECIMAL_PLACES), 1, 2);

    time_t now = time(NULL);

    payload[0] = cmd;

    write_u32(&payload[1], item_price_32);
    write_u16(&payload[5], item_number);
    write_u32(&payload[7], (uint32_t) now);
    write_u32(&payload[11], 0);

    unsigned char hmac[32];
    calculate_hmac((const char*) payload, 15, hmac);
    memcpy(payload + 15, hmac, 4);
}

// Grants credit to the internal cashless session, respecting arbitration:
// parked (logged, dropped) if an external physical cashless already owns
// the session.
static void cashless_internal_grant_credit(uint16_t funds_available) {
    if (g_active_cashless_source != CASHLESS_SRC_NONE) {
        ESP_LOGW(TAG, "Internal cashless: credit grant parked (source=%d active)", g_active_cashless_source);
        return;
    }

    g_active_cashless_source = CASHLESS_SRC_INTERNAL;
    cashless_to_target_evt_t evt = { CASHLESS_SRC_INTERNAL, CSHL_EVT_BEGIN_SESSION, funds_available };
    xQueueSend(cashless_to_target_queue, &evt, 0);
}

static void ble_pax_event_handler(uint16_t devices_count) {
    char topic[64], msg[48], line[128];
    snprintf(msg, sizeof(msg), "%u:%lld", devices_count, (long long) time(NULL));
    rpc_sign_text(msg, line, sizeof(line));

    snprintf(topic, sizeof(topic), "domain.vmflow.xyz/%s/paxcounter", my_subdomain);
    esp_mqtt_client_enqueue(mqtt_client, topic, line, 0, 1, 0, 1);
}

static void ble_event_handler(char *ble_payload) {
    switch ((uint8_t) ble_payload[0]) {
    case 0x00: { // Set subdomain
        nvs_handle_t handle;
        nvs_open("vmflow", NVS_READWRITE, &handle);

        size_t s_len;
        if (nvs_get_str(handle, "domain", NULL, &s_len) != ESP_OK) {
            strcpy(my_subdomain, ble_payload + 1);

            nvs_set_str(handle, "domain", my_subdomain);
            nvs_commit(handle);

            char myhost[64];
            snprintf(myhost, sizeof(myhost), "%s.vmflow.xyz", my_subdomain);

            ble_set_device_name(myhost);

            xEventGroupSetBits(xLedEventGroup, BIT_STATUS_DOMAIN | BIT_STATUS_TRIGGER);

            ESP_LOGI(TAG, "HOST= %s", myhost);
        }
        nvs_close(handle);
        break;
    }
    case 0x01: { // Set passkey
        nvs_handle_t handle;
        nvs_open("vmflow", NVS_READWRITE, &handle);

        size_t s_len;
        if (nvs_get_str(handle, "passkey", NULL, &s_len) != ESP_OK) {
            strcpy(my_passkey, ble_payload + 1);

            nvs_set_str(handle, "passkey", my_passkey);
            nvs_commit(handle);

            xEventGroupSetBits(xLedEventGroup, BIT_STATUS_PASSKEY | BIT_STATUS_TRIGGER);

            ESP_LOGI(TAG, "PASSKEY= %s", my_passkey);
        }
        nvs_close(handle);
        break;
    }
    case 0x02: // Unlimited/reset credit
        cashless_internal_grant_credit(0xffff);
        break;
    case 0x03: { // Vend approval (signed)
        uint16_t item_price = 0, item_number = 0;
        if (ble_decode_with_passkey(&item_price, &item_number, (uint8_t*) ble_payload) == ESP_OK
            && g_active_cashless_source == CASHLESS_SRC_INTERNAL) {
            cashless_to_target_evt_t evt = { CASHLESS_SRC_INTERNAL, CSHL_EVT_VEND_APPROVED, item_price };
            xQueueSend(cashless_to_target_queue, &evt, 0);
        }
        break;
    }
    case 0x04: // Cancel session
        if (g_active_cashless_source == CASHLESS_SRC_INTERNAL) {
            cashless_to_target_evt_t evt = { CASHLESS_SRC_INTERNAL, CSHL_EVT_VEND_DENIED, 0 };
            xQueueSend(cashless_to_target_queue, &evt, 0);
        }
        break;
    case 0x06: { // Set WiFi SSID
        esp_wifi_disconnect();

        wifi_config_t wifi_config = { 0 };
        esp_wifi_get_config(WIFI_IF_STA, &wifi_config);

        strcpy((char*) wifi_config.sta.ssid, ble_payload + 1);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

        ESP_LOGI(TAG, "SSID= %s", wifi_config.sta.ssid);
        break;
    }
    case 0x07: { // Set WiFi password
        wifi_config_t wifi_config = { 0 };
        esp_wifi_get_config(WIFI_IF_STA, &wifi_config);

        strcpy((char*) wifi_config.sta.password, ble_payload + 1);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

        esp_wifi_connect();

        ESP_LOGI(TAG, "PASSWORD= %s", wifi_config.sta.password);
        break;
    }
    }
}

// OTA worker: pulls the app image from a GitHub release asset over HTTPS, writes the inactive slot, then reboots.
// Runs in its own task because the download blocks for tens of seconds and must not stall the MQTT event task.
static void ota_task(void *arg) {
    const char *url = (const char *) arg;

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,  // GitHub redirects to release-assets.githubusercontent.com (S3)
        .timeout_ms = 30000,
        .keep_alive_enable = true,
        // The S3 redirect target is a ~900-char signed URL; the default 512 B tx buffer can't hold the request line.
        .buffer_size = 2048,
        .buffer_size_tx = 4096,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "OTA success, rebooting into new image");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
}

// Device snapshot (JSON) on .../rpc/info.
static void rpc_publish_info(void) {
    const esp_app_desc_t *app = esp_app_get_description();

    char topic[64], json[384];
    int n = snprintf(json, sizeof(json),
        "{\"version\":\"%s\",\"uptime_s\":%lld,"
        "\"free_heap\":%lu,\"min_free_heap\":%lu,"
        "\"cashless_source\":%d,\"changer_state\":%d,\"validator_state\":%d,"
        "\"last_sale_price\":%u,\"last_sale_item\":%u,"
        "\"last_vend_success_time\":%lld,\"ip_wifi\":\"%s\"}",
        app->version,
        (long long) (esp_timer_get_time() / 1000000),
        (unsigned long) esp_get_free_heap_size(),
        (unsigned long) esp_get_minimum_free_heap_size(),
        (int) g_active_cashless_source, (int) reader0x08.machine_state, (int) reader0x30.machine_state,
        last_sale_price, last_sale_item,
        (long long) last_vend_success_time,
        s_ip_wifi);

    snprintf(topic, sizeof(topic), "domain.vmflow.xyz/%s/rpc/info", my_subdomain);
    esp_mqtt_client_enqueue(mqtt_client, topic, json, n, 1, 0, 1);
}

// Money currently held in the coin tubes / bill stacker, on .../rpc/safe.
// Coin total is a live figure (tube count x credit, refreshed on every
// deposit). Bill total is best-effort: MDB never reports per-type bill
// counts in the stacker, only a raw count, so the value is the total
// accepted since the validator's last reset, not a live hardware read.
static void rpc_publish_safe(void) {
    char topic[64], json[320];

    bool coin_present = (reader0x08.machine_state != INACTIVE_STATE);
    double coin_total_cents = 0;
    if (coin_present) {
        uint32_t coin_total_scaled = 0;
        for (int i = 0; i < 16; i++)
            coin_total_scaled += (uint32_t) reader0x08.tube_counts[i] * reader0x08.coin_credit[i];
        coin_total_cents = TO_SCALE_FACTOR(FROM_SCALE_FACTOR(coin_total_scaled, reader0x08.scale_factor, reader0x08.decimal_places), 1, 2);
    }

    bool bill_present = (reader0x30.machine_state != INACTIVE_STATE);
    double bill_value_cents = 0;
    if (bill_present) {
        bill_value_cents = TO_SCALE_FACTOR(FROM_SCALE_FACTOR(reader0x30.credit, reader0x30.scale_factor, reader0x30.decimal_places), 1, 2);
    }

    int n = snprintf(json, sizeof(json),
        "{\"coin\":{\"present\":%s,\"tube_total_cents\":%.0f,\"tube_full\":%s},"
        "\"bill\":{\"present\":%s,\"stacker_count\":%u,\"stacker_full\":%s,"
        "\"stacker_value_cents_since_reset\":%.0f}}",
        coin_present ? "true" : "false",
        coin_total_cents,
        reader0x08.tube_full_status ? "true" : "false",
        bill_present ? "true" : "false",
        reader0x30.stacker_count,
        reader0x30.stacker_full ? "true" : "false",
        bill_value_cents);

    snprintf(topic, sizeof(topic), "domain.vmflow.xyz/%s/rpc/safe", my_subdomain);
    esp_mqtt_client_enqueue(mqtt_client, topic, json, n, 1, 0, 1);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t) event_id) {
    case MQTT_EVENT_CONNECTED: {
        char topic[64], buf[32];
        snprintf(topic, sizeof(topic), "%s.vmflow.xyz/#", my_subdomain);
        esp_mqtt_client_subscribe(client, topic, 0);

        snprintf(topic, sizeof(topic), "domain.vmflow.xyz/%s/status", my_subdomain);
        snprintf(buf, sizeof(buf), "online,%d", (int) esp_reset_reason());
        esp_mqtt_client_enqueue(client, topic, buf, 0, 1, 1, 1);

        xEventGroupSetBits(xLedEventGroup, BIT_STATUS_MQTT | BIT_STATUS_TRIGGER);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        xEventGroupClearBits(xLedEventGroup, BIT_STATUS_MQTT);
        xEventGroupSetBits(xLedEventGroup, BIT_STATUS_TRIGGER);
        break;
    case MQTT_EVENT_DATA: {
        ESP_LOGI(TAG, "MQTT data topic=%.*s len=%d", event->topic_len, event->topic, event->data_len);

        if (event->topic_len > 4 && strncmp(event->topic + event->topic_len - 4, "/rpc", 4) == 0) {
            // Wire format "<cmd>:<args>:<ts>:<hmac_hex>".
            int len = event->data_len < 127 ? event->data_len : 127;

            char *last_colon = memrchr(event->data, ':', len);
            if (last_colon == NULL) break;

            int prefix_len = last_colon - event->data;
            char hmac[65];
            snprintf(hmac, sizeof(hmac), "%.*s", len - prefix_len - 1, last_colon + 1);

            if (!rpc_verify_hmac(event->data, prefix_len, hmac)) {
                ESP_LOGW(TAG, "RPC rejected: bad HMAC");
                break;
            }

            char cmd[32], args[64];
            unsigned int ts;
            if (sscanf(event->data, "%31[^:]:%63[^:]:%u", cmd, args, &ts) != 3) {
                ESP_LOGW(TAG, "RPC rejected: malformed");
                break;
            }

            long dt = (long) (time(NULL) - (time_t) ts);
            if (labs(dt) > RPC_FRESHNESS_SEC) {
                ESP_LOGW(TAG, "RPC rejected: stale ts (dt=%ld)", dt);
                break;
            }

            bool has_args = (args[0] != '\0' && strcmp(args, "-") != 0);

            char topic_confirm[64];
            snprintf(topic_confirm, sizeof(topic_confirm), "domain.vmflow.xyz/%s/rpc/confirm", my_subdomain);

            if (strcmp(cmd, "info") == 0) {
                rpc_publish_info();
                ESP_LOGI(TAG, "RPC info published");
            } else if (strcmp(cmd, "safe") == 0) {
                rpc_publish_safe();
                ESP_LOGI(TAG, "RPC safe published");
            } else if (strcmp(cmd, "credit") == 0 && has_args) {
                int32_t price_wire = (int32_t) strtol(args, NULL, 10);
                uint16_t funds_available = TO_SCALE_FACTOR(FROM_SCALE_FACTOR(price_wire, 1, 2), CASHLESS_DEFAULT_SCALE_FACTOR, CASHLESS_DEFAULT_DECIMAL_PLACES);

                cashless_internal_grant_credit(funds_available);

                esp_mqtt_client_enqueue(client, topic_confirm, "ok", 0, 1, 0, 1);
                ESP_LOGI(TAG, "RPC credit: funds=%u", funds_available);
            } else if (strcmp(cmd, "echo") == 0) {
                char topic[64], buf[24];
                snprintf(topic, sizeof(topic), "domain.vmflow.xyz/%s/rpc/echo", my_subdomain);
                snprintf(buf, sizeof(buf), "%u", ts);
                esp_mqtt_client_enqueue(client, topic, buf, 0, 0, 0, 1);
                ESP_LOGI(TAG, "RPC echo");
            } else if (strcmp(cmd, "restart") == 0) {
                esp_mqtt_client_publish(client, topic_confirm, "ok", 0, 1, 0);
                ESP_LOGW(TAG, "RPC restart requested");

                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            } else if (strcmp(cmd, "ota") == 0) {
                // "ota:-" -> latest release; "ota:<tag>" -> pinned tag.
                static char ota_url[160];
                if (has_args)
                    snprintf(ota_url, sizeof(ota_url), "https://github.com/nodestark/mdb-esp32-master/releases/download/%s/mdb-master-esp32s3.bin", args);
                else
                    snprintf(ota_url, sizeof(ota_url), "https://github.com/nodestark/mdb-esp32-master/releases/latest/download/mdb-master-esp32s3.bin");

                esp_mqtt_client_enqueue(client, topic_confirm, "ok", 0, 1, 0, 1);
                ESP_LOGW(TAG, "RPC ota: %s", ota_url);
                xTaskCreate(ota_task, "ota_task", 8192, ota_url, 5, NULL);
            } else {
                ESP_LOGW(TAG, "RPC unknown command: %s", cmd);
            }
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "MQTT TCP error: errno=%d", event->error_handle->esp_transport_sock_errno);
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "MQTT connection refused: 0x%x", event->error_handle->connect_return_code);
        }
        break;
    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        snprintf(s_ip_wifi, sizeof(s_ip_wifi), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "wifi got IP: %s", s_ip_wifi);
        xEventGroupSetBits(xWifiEventGroup, BIT_STA_GOT_IP);
        break;
    }
    case IP_EVENT_STA_LOST_IP:
        s_ip_wifi[0] = '\0';
        xEventGroupClearBits(xWifiEventGroup, BIT_STA_GOT_IP);
        xEventGroupSetBits(xWifiEventGroup, BIT_STA_LOST_IP);
        ESP_LOGW(TAG, "wifi lost IP");
        break;
    }
}

static void wifi_retry_cb(void *arg) {
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        wifi_backoff_ms = WIFI_BACKOFF_MIN_MS;
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGW(TAG, "WiFi disconnected, retry in %lu ms", wifi_backoff_ms);
        esp_timer_start_once(wifi_retry_timer, (uint64_t) wifi_backoff_ms * 1000);
        wifi_backoff_ms *= 2;
        if (wifi_backoff_ms > WIFI_BACKOFF_MAX_MS) wifi_backoff_ms = WIFI_BACKOFF_MAX_MS;
        break;
    }
}

// Owns the MQTT client lifecycle: starts once WiFi has an IP, stops (and
// waits to restart) if WiFi drops.
static void mqtt_task(void *pvParameters) {
    char lwt_topic[64];
    snprintf(lwt_topic, sizeof(lwt_topic), "domain.vmflow.xyz/%s/status", my_subdomain);

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://mqtt.vmflow.xyz",
        .session.last_will.topic = lwt_topic,
        .session.last_will.msg = "offline",
        .session.last_will.qos = 1,
        .session.last_will.retain = 1,
        .session.keepalive = 120,
        .network.timeout_ms = 30000,
        .network.reconnect_timeout_ms = 15000,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    for (;;) {
        xEventGroupWaitBits(xWifiEventGroup, BIT_STA_GOT_IP, pdFALSE, pdFALSE, portMAX_DELAY);
        esp_mqtt_client_start(mqtt_client);

        xEventGroupWaitBits(xWifiEventGroup, BIT_STA_LOST_IP, pdTRUE, pdTRUE, portMAX_DELAY);
        esp_mqtt_client_stop(mqtt_client);
    }
}

#define LED_LVL 30

static void led_status_task(void *pvParameters) {
    led_strip_handle_t led_strip;
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin_mdb_led,
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    for (;;) {
        EventBits_t b = xEventGroupWaitBits(xLedEventGroup, BIT_STATUS_TRIGGER, pdTRUE, pdFALSE, portMAX_DELAY);

        bool installed = (b & MASK_STATUS_INSTALLED) == MASK_STATUS_INSTALLED;
        bool net = b & BIT_STATUS_MQTT;

        if (!installed) led_strip_set_pixel(led_strip, 0, LED_LVL, LED_LVL, LED_LVL); // white (not provisioned)
        else if (net)   led_strip_set_pixel(led_strip, 0,       0, LED_LVL,       0); // green (provisioned + MQTT)
        else            led_strip_set_pixel(led_strip, 0, LED_LVL,       0, LED_LVL); // magenta (provisioned, no MQTT)
        led_strip_refresh(led_strip);
    }
}

// Bridges the cashless arbitration queues to the internal payment engine
// (BLE phone app + backend MQTT RPC). Owns ble_event_handler/
// ble_pax_event_handler's session-facing side effects.
static void cashless_internal_task(void *pvParameters) {
    for (;;) {
        cashless_from_target_evt_t evt;
        xQueueReceive(cashless_from_target_to_internal_queue, &evt, portMAX_DELAY);

        uint8_t payload[19];

        switch (evt.type) {
        case CSHL_REQ_VEND_REQUEST:
            ble_encode_with_passkey(0x0a, evt.item_price, evt.item_number, payload);
            ble_notify_send((char*) payload, sizeof(payload));
            break;

        case CSHL_REQ_VEND_CANCEL: {
            // Real VMC canceled the vend; nothing for the phone to approve.
            cashless_to_target_evt_t out = { CASHLESS_SRC_INTERNAL, CSHL_EVT_VEND_DENIED, 0 };
            xQueueSend(cashless_to_target_queue, &out, 0);
            break;
        }
        case CSHL_REQ_VEND_SUCCESS:
            last_sale_price = evt.item_price;
            last_sale_item = evt.item_number;
            last_vend_success_time = time(NULL);

            ble_encode_with_passkey(0x0b, evt.item_price, evt.item_number, payload);
            ble_notify_send((char*) payload, sizeof(payload));
            break;

        case CSHL_REQ_VEND_FAILURE: {
            ble_encode_with_passkey(0x0c, evt.item_price, evt.item_number, payload);
            ble_notify_send((char*) payload, sizeof(payload));

            char topic[64], msg[64], line[160];
            snprintf(msg, sizeof(msg), "%u,%u:%lld", evt.item_price, evt.item_number, (long long) time(NULL));
            rpc_sign_text(msg, line, sizeof(line));
            snprintf(topic, sizeof(topic), "domain.vmflow.xyz/%s/vend_fail", my_subdomain);
            esp_mqtt_client_enqueue(mqtt_client, topic, line, 0, 1, 0, 1);
            break;
        }
        case CSHL_REQ_SESSION_COMPLETE: {
            ble_encode_with_passkey(0x0d, evt.item_price, evt.item_number, payload);
            ble_notify_send((char*) payload, sizeof(payload));

            // Tell the target's cashless FSM the internal session is done so
            // it emits End Session and releases the arbitration lock.
            cashless_to_target_evt_t out = { CASHLESS_SRC_INTERNAL, CSHL_EVT_SESSION_END, 0 };
            xQueueSend(cashless_to_target_queue, &out, 0);
            break;
        }
        }
    }
}

//------------------------------------------------------------------------//
// Target port: bit-banged 9-bit MDB I/O (this board is the slave here).
//------------------------------------------------------------------------//

static QueueHandle_t mdb_rx_queue;

static void IRAM_ATTR mdb_rx_falling_isr(void *arg) {
    gpio_intr_disable(pin_mdb_target_rx);

    uint16_t coming_read = 0x0000;

    ets_delay_us(156);

    for (int x = 0; x < 9; x++) {
        coming_read |= (gpio_get_level(pin_mdb_target_rx) << x);
        ets_delay_us(104);
    }
    xQueueSendFromISR(mdb_rx_queue, &coming_read, NULL);

    gpio_intr_enable(pin_mdb_target_rx);
}

static uint16_t read_9(uint8_t *checksum) {
    uint16_t coming_read = 0;
    xQueueReceive(mdb_rx_queue, &coming_read, portMAX_DELAY);

    if (checksum)
        *checksum += coming_read;

    return coming_read;
}

static void write_9(uint16_t nth9) {
    gpio_set_level(pin_mdb_target_tx, 0);
    ets_delay_us(104);

    for (uint8_t x = 0; x < 9; x++) {
        gpio_set_level(pin_mdb_target_tx, (nth9 >> x) & 1);
        ets_delay_us(104);
    }

    gpio_set_level(pin_mdb_target_tx, 1);
    ets_delay_us(104);
}

static void write_payload_9(uint8_t *mdb_payload, uint8_t length) {
    uint8_t checksum = 0x00;

    for (int x = 0; x < length; x++) {
        checksum += mdb_payload[x];
        write_9(mdb_payload[x]);
    }

    write_9(BIT_MODE_SET | checksum);
}

//------------------------------------------------------------------------//
// Controller port: hardware UART2 9-bit MDB I/O (this board is the master).
//------------------------------------------------------------------------//

static void write_controller_9(uint16_t nth9) {
    uint8_t ones = __builtin_popcount((uint8_t) nth9);

    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(250));

    if ((nth9 >> 8) & 1) {
        uart_set_parity(UART_NUM_2, ones % 2 ? UART_PARITY_EVEN : UART_PARITY_ODD);
    } else {
        uart_set_parity(UART_NUM_2, ones % 2 ? UART_PARITY_ODD : UART_PARITY_EVEN);
    }

    uart_write_bytes(UART_NUM_2, (uint8_t*) &nth9, 1);
}

static void write_payload_controller_9(uint8_t *mdb_payload_tx, uint8_t mdb_length) {

    uint8_t checksum = 0;

    write_controller_9((checksum = mdb_payload_tx[0]) | BIT_MODE_SET);
    for (uint8_t x = 1; x < mdb_length; x++) {
        write_controller_9(mdb_payload_tx[x]);

        checksum += mdb_payload_tx[x];
    }

    write_controller_9(checksum);
}

//------------------------------------------------------------------------//
// mdb_controller_task - master: drives external cashless, coin changer and
// bill validator on the physical MDB bus, bridging events to/from the
// target task via the queues above.
//------------------------------------------------------------------------//

void mdb_controller_task(void *pvParameters) {

    uint8_t mdb_payload_tx[36];
    uint8_t mdb_payload_rx[36];

    size_t len;

    uint8_t await = 125; // MDB response window (ms)

    for (;;) {

        uart_flush(UART_NUM_2);

        //--------------------------------------------------------------//
        // 0x10 External cashless
        //--------------------------------------------------------------//
        if (reader0x10.machine_state == INACTIVE_STATE) {

            mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_RESET & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*

            if (len == 1) {
                mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_POLL & BIT_CMD_SET);
                write_payload_controller_9(mdb_payload_tx, 1);

                len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 2, pdMS_TO_TICKS(await)); // CHK*
                if (len != 2) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                write_controller_9(ACK | BIT_MODE_SET);

                if (mdb_payload_rx[0] == 0x00 /*Just Reset*/) {

                    mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_SETUP & BIT_CMD_SET);
                    mdb_payload_tx[1] = 0x00;          // Config Data
                    mdb_payload_tx[2] = 1;             // VMC Feature Level
                    mdb_payload_tx[3] = 0;             // Columns on Display
                    mdb_payload_tx[4] = 0;             // Rows on Display
                    mdb_payload_tx[5] = 0b00000001;    // Display Information

                    write_payload_controller_9(mdb_payload_tx, 6);

                    len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 9, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 9) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    reader0x10.feature_level = mdb_payload_rx[1];
                    reader0x10.country_code = (mdb_payload_rx[2] << 8) | mdb_payload_rx[3];
                    reader0x10.scale_factor = mdb_payload_rx[4];
                    reader0x10.decimal_places = mdb_payload_rx[5];
                    reader0x10.response_time_sec = mdb_payload_rx[6];
                    reader0x10.miscellaneous = mdb_payload_rx[7];

                    reader0x10.machine_state = DISABLED_STATE;

                    ESP_LOGI(TAG, "External cashless: Reader Config");
                }
            }

        } else if (reader0x10.machine_state == DISABLED_STATE) {

            mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_SETUP & BIT_CMD_SET);
            mdb_payload_tx[1] = 0x01; // Max|Min Prices
            mdb_payload_tx[2] = 0xFF; // no max price limit
            mdb_payload_tx[3] = 0xFF;
            mdb_payload_tx[4] = 0x00; // no min price limit
            mdb_payload_tx[5] = 0x00;

            write_payload_controller_9(mdb_payload_tx, 6);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
            if (len != 1) { reader0x10.machine_state = INACTIVE_STATE; continue; }

            mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_EXPANSION & BIT_CMD_SET);
            mdb_payload_tx[1] = 0x00; // Request ID

            mdb_payload_tx[2] = 'V'; mdb_payload_tx[3] = 'M'; mdb_payload_tx[4] = 'F'; // Manufacturer code
            memset(&mdb_payload_tx[5], ' ', 12);  // Serial Number
            memset(&mdb_payload_tx[17], ' ', 12); // Model Number
            mdb_payload_tx[29] = '0'; mdb_payload_tx[30] = '1'; // Software Version

            write_payload_controller_9(mdb_payload_tx, 31);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 31, pdMS_TO_TICKS(await)); // CHK*
            if (len != 31) { reader0x10.machine_state = INACTIVE_STATE; continue; }

            write_controller_9(ACK | BIT_MODE_SET);

            ESP_LOGI(TAG, "External cashless: Manufacture=%.*s Serial=%.*s Model=%.*s SW=%.*s",
                    3, &mdb_payload_rx[1], 12, &mdb_payload_rx[4], 12, &mdb_payload_rx[16], 2, &mdb_payload_rx[28]);

            mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_READER & BIT_CMD_SET);
            mdb_payload_tx[1] = 0x01; // Reader Enable

            write_payload_controller_9(mdb_payload_tx, 2);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
            if (len != 1) { reader0x10.machine_state = INACTIVE_STATE; continue; }

            reader0x10.machine_state = ENABLED_STATE;
            ESP_LOGI(TAG, "External cashless: Reader Enable");

        } else {

            // Relay a vend outcome from the target bridge, if one is pending
            // and this device currently owns the session.
            cashless_from_target_evt_t out_evt;
            if (g_active_cashless_source == CASHLESS_SRC_EXTERNAL &&
                xQueueReceive(cashless_from_target_to_controller_queue, &out_evt, 0)) {

                switch (out_evt.type) {
                case CSHL_REQ_VEND_REQUEST: {
                    if (reader0x10.machine_state == IDLE_STATE) {
                        mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                        mdb_payload_tx[1] = 0x00; // Vend Request
                        mdb_payload_tx[2] = out_evt.item_price >> 8;
                        mdb_payload_tx[3] = out_evt.item_price;
                        mdb_payload_tx[4] = out_evt.item_number >> 8;
                        mdb_payload_tx[5] = out_evt.item_number;

                        write_payload_controller_9(mdb_payload_tx, 6);

                        len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
                        if (len == 1) reader0x10.machine_state = VEND_STATE;

                        ESP_LOGI(TAG, "External cashless: Vend Request forwarded");
                    }
                    break;
                }
                case CSHL_REQ_VEND_CANCEL: {
                    mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    mdb_payload_tx[1] = 0x01; // Vend Cancel
                    write_payload_controller_9(mdb_payload_tx, 2);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
                    break;
                }
                case CSHL_REQ_VEND_SUCCESS: {
                    mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    mdb_payload_tx[1] = 0x02; // Vend Success
                    mdb_payload_tx[2] = out_evt.item_number >> 8;
                    mdb_payload_tx[3] = out_evt.item_number;
                    write_payload_controller_9(mdb_payload_tx, 4);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
                    reader0x10.machine_state = IDLE_STATE;
                    break;
                }
                case CSHL_REQ_VEND_FAILURE: {
                    mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    mdb_payload_tx[1] = 0x03; // Vend Failure
                    write_payload_controller_9(mdb_payload_tx, 2);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
                    reader0x10.machine_state = IDLE_STATE;
                    break;
                }
                case CSHL_REQ_SESSION_COMPLETE: {
                    mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    mdb_payload_tx[1] = 0x04; // Session Complete
                    write_payload_controller_9(mdb_payload_tx, 2);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
                    break;
                }
                }

                uart_flush(UART_NUM_2);
            }

            mdb_payload_tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_POLL & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await));
            if (len == 1) {
                reader0x10.poll_fail_count = 0;

                if (mdb_payload_rx[0] == 0x07 /*End Session*/) {

                    len += uart_read_bytes(UART_NUM_2, mdb_payload_rx + len, 1, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 2) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    reader0x10.machine_state = ENABLED_STATE;

                    if (g_active_cashless_source == CASHLESS_SRC_EXTERNAL) {
                        cashless_to_target_evt_t evt = { CASHLESS_SRC_EXTERNAL, CSHL_EVT_SESSION_END, 0 };
                        xQueueSend(cashless_to_target_queue, &evt, 0);
                        g_active_cashless_source = CASHLESS_SRC_NONE;
                    }

                } else if (mdb_payload_rx[0] == 0x06 /*Vend Denied*/) {

                    len += uart_read_bytes(UART_NUM_2, mdb_payload_rx + len, 1, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 2) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    reader0x10.machine_state = IDLE_STATE;

                    if (g_active_cashless_source == CASHLESS_SRC_EXTERNAL) {
                        cashless_to_target_evt_t evt = { CASHLESS_SRC_EXTERNAL, CSHL_EVT_VEND_DENIED, 0 };
                        xQueueSend(cashless_to_target_queue, &evt, 0);
                    }

                } else if (mdb_payload_rx[0] == 0x05 /*Vend Approved*/) {

                    len += uart_read_bytes(UART_NUM_2, mdb_payload_rx + len, 3, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 4) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    uint16_t vend_amount = (mdb_payload_rx[1] << 8) | mdb_payload_rx[2];

                    if (g_active_cashless_source == CASHLESS_SRC_EXTERNAL) {
                        cashless_to_target_evt_t evt = { CASHLESS_SRC_EXTERNAL, CSHL_EVT_VEND_APPROVED, vend_amount };
                        xQueueSend(cashless_to_target_queue, &evt, 0);
                    }

                } else if (mdb_payload_rx[0] == 0x04 /*Session Cancel Request*/) {

                    len += uart_read_bytes(UART_NUM_2, mdb_payload_rx + len, 1, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 2) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    if (g_active_cashless_source == CASHLESS_SRC_EXTERNAL) {
                        cashless_to_target_evt_t evt = { CASHLESS_SRC_EXTERNAL, CSHL_EVT_SESSION_CANCEL, 0 };
                        xQueueSend(cashless_to_target_queue, &evt, 0);
                    }

                } else if (mdb_payload_rx[0] == 0x03 /*Begin Session*/) {

                    len += uart_read_bytes(UART_NUM_2, mdb_payload_rx + len, 3, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 4) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    uint16_t funds_available = (mdb_payload_rx[1] << 8) | mdb_payload_rx[2];

                    reader0x10.machine_state = IDLE_STATE;

                    if (g_active_cashless_source == CASHLESS_SRC_NONE) {
                        g_active_cashless_source = CASHLESS_SRC_EXTERNAL;
                        cashless_to_target_evt_t evt = { CASHLESS_SRC_EXTERNAL, CSHL_EVT_BEGIN_SESSION, funds_available };
                        xQueueSend(cashless_to_target_queue, &evt, 0);
                        ESP_LOGI(TAG, "External cashless: Begin Session funds=%u", funds_available);
                    } else {
                        ESP_LOGW(TAG, "External cashless: Begin Session parked (internal session active)");
                    }

                } else if (mdb_payload_rx[0] == 0x0b /*Command Out of Sequence*/) {

                    len += uart_read_bytes(UART_NUM_2, mdb_payload_rx + len, 1, pdMS_TO_TICKS(await)); // CHK*
                    if (len != 2) { reader0x10.machine_state = INACTIVE_STATE; continue; }

                    write_controller_9(ACK | BIT_MODE_SET);

                    reader0x10.machine_state = INACTIVE_STATE;
                }

            } else {
                if (++reader0x10.poll_fail_count >= 10) reader0x10.machine_state = INACTIVE_STATE; // 10: tolerate transient bus glitches
            }
        }

        uart_flush(UART_NUM_2);

        //--------------------------------------------------------------//
        // 0x08 Coin Changer
        //--------------------------------------------------------------//
        if (reader0x08.machine_state == INACTIVE_STATE) {

            mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_RESET & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*

            if (len == 1) {

                mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_POLL & BIT_CMD_SET);
                write_payload_controller_9(mdb_payload_tx, 1);

                len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 2, pdMS_TO_TICKS(await)); // Just Reset + CHK*

                if (len == 2) {
                    write_controller_9(ACK | BIT_MODE_SET);

                    if (mdb_payload_rx[0] == 0x00 /*Just Reset*/) {
                        reader0x08.machine_state = DISABLED_STATE;
                        ESP_LOGI(TAG, "Changer - Just Reset");
                    }
                }
            }

        } else if (reader0x08.machine_state == DISABLED_STATE) {

            mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_SETUP & BIT_CMD_SET);
            mdb_payload_tx[1] = 0x00; // Sub
            mdb_payload_tx[2] = 0x01; // VMC Feature Level
            mdb_payload_tx[3] = CONFIG_MDB_CURRENCY_CODE >> 8;   // Country Code High
            mdb_payload_tx[4] = CONFIG_MDB_CURRENCY_CODE & 0xFF; // Country Code Low
            mdb_payload_tx[5] = 0x01; // Scale Factor
            mdb_payload_tx[6] = 0x02; // Decimal Places

            write_payload_controller_9(mdb_payload_tx, 7);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 24, pdMS_TO_TICKS(await)); // 23 + CHK*

            if (len == 24) {
                write_controller_9(ACK | BIT_MODE_SET);

                reader0x08.feature_level     = mdb_payload_rx[0];
                reader0x08.country_code      = (mdb_payload_rx[1] << 8) | mdb_payload_rx[2];
                reader0x08.scale_factor      = mdb_payload_rx[3];
                reader0x08.decimal_places    = mdb_payload_rx[4];
                reader0x08.coin_type_routing = (mdb_payload_rx[5] << 8) | mdb_payload_rx[6];
                for (uint8_t i = 0; i < 16; i++)
                    reader0x08.coin_credit[i] = mdb_payload_rx[7 + i];

                ESP_LOGI(TAG, "Changer Setup: feature=%d scale=%d dec=%d routing=0x%04X",
                        reader0x08.feature_level, reader0x08.scale_factor,
                        reader0x08.decimal_places, reader0x08.coin_type_routing);

                mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_TUBE_STATUS & BIT_CMD_SET);
                write_payload_controller_9(mdb_payload_tx, 1);

                len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 19, pdMS_TO_TICKS(await)); // 18 + CHK*

                if (len == 19) {
                    write_controller_9(ACK | BIT_MODE_SET);

                    reader0x08.tube_full_status = (mdb_payload_rx[0] << 8) | mdb_payload_rx[1];
                    for (uint8_t i = 0; i < 16; i++)
                        reader0x08.tube_counts[i] = mdb_payload_rx[2 + i];

                    ESP_LOGI(TAG, "Changer Tube Status: full=0x%04X", reader0x08.tube_full_status);
                }

                // Enable all coins by default; the real VMC (via target bridge)
                // can narrow this down later through CHGR_COIN_TYPE.
                mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_COIN_TYPE & BIT_CMD_SET);
                mdb_payload_tx[1] = 0xFF;
                mdb_payload_tx[2] = 0xFF;
                mdb_payload_tx[3] = 0x00;
                mdb_payload_tx[4] = 0x00;

                write_payload_controller_9(mdb_payload_tx, 5);

                len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 5, pdMS_TO_TICKS(await)); // 4 + CHK*

                if (len == 5)
                    write_controller_9(ACK | BIT_MODE_SET);

                reader0x08.machine_state = ENABLED_STATE;
                ESP_LOGI(TAG, "Changer Enabled");
            }

        } else {
            // ENABLED_STATE

            coin_from_target_evt_t coin_req;
            if (xQueueReceive(coin_from_target_queue, &coin_req, 0)) {

                if (coin_req.type == COIN_REQ_TYPE_ENABLE) {
                    mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_COIN_TYPE & BIT_CMD_SET);
                    mdb_payload_tx[1] = coin_req.coin_enable >> 8;
                    mdb_payload_tx[2] = coin_req.coin_enable;
                    mdb_payload_tx[3] = coin_req.dispense_enable >> 8;
                    mdb_payload_tx[4] = coin_req.dispense_enable;
                    write_payload_controller_9(mdb_payload_tx, 5);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 5, pdMS_TO_TICKS(await));

                } else if (coin_req.type == COIN_REQ_DISPENSE) {
                    mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_DISPENSE & BIT_CMD_SET);
                    mdb_payload_tx[1] = coin_req.dispense_value;
                    write_payload_controller_9(mdb_payload_tx, 2);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await));
                }

                uart_flush(UART_NUM_2);
            }

            mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_POLL & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            // POLL blocks the full window (reads up to 17 bytes, usually only ACK arrives).
            // 60ms over 30ms: real changers can spread a multi-byte event reply past 30ms,
            // which truncated the read and lost the coin event. Costs ~30ms more per idle poll.
            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 17, pdMS_TO_TICKS(60)); // events + CHK*

            if (len == 1) {
                reader0x08.poll_fail_count = 0;

            } else if (len > 1) {
                reader0x08.poll_fail_count = 0;
                write_controller_9(ACK | BIT_MODE_SET);

                if (len == 2 && mdb_payload_rx[0] == 0x00) {
                    reader0x08.machine_state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Changer reset detected");
                } else {
                    uint8_t deposited_types[16];
                    uint8_t deposited_count = 0;

                    for (uint8_t i = 0; i + 1 < len; i++) {
                        uint8_t ev = mdb_payload_rx[i];
                        if (ev & 0x80) { // coin deposited
                            uint8_t coin_type = ev & 0x0F;
                            reader0x08.credit += reader0x08.coin_credit[coin_type];
                            if (deposited_count < 16) deposited_types[deposited_count++] = coin_type;

                            ESP_LOGI(TAG, "Coin inserted: type=%d credit+=%d total=%d",
                                    coin_type, reader0x08.coin_credit[coin_type], reader0x08.credit);
                        }
                    }

                    if (deposited_count > 0) {
                        // Refresh the real tube counts (stale since SETUP/last refresh)
                        // so both the forwarded poll event and the "safe" RPC report
                        // accurate figures.
                        mdb_payload_tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_TUBE_STATUS & BIT_CMD_SET);
                        write_payload_controller_9(mdb_payload_tx, 1);

                        size_t tube_len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 19, pdMS_TO_TICKS(await)); // 18 + CHK*
                        if (tube_len == 19) {
                            write_controller_9(ACK | BIT_MODE_SET);

                            reader0x08.tube_full_status = (mdb_payload_rx[0] << 8) | mdb_payload_rx[1];
                            for (uint8_t i = 0; i < 16; i++)
                                reader0x08.tube_counts[i] = mdb_payload_rx[2 + i];
                        }

                        for (uint8_t i = 0; i < deposited_count; i++) {
                            coin_deposit_evt_t deposit_evt = { deposited_types[i], reader0x08.tube_counts[deposited_types[i]] };
                            xQueueSend(coin_to_target_queue, &deposit_evt, 0);
                        }
                    }
                }

            } else {
                if (++reader0x08.poll_fail_count >= 10) { // 10: tolerate transient bus glitches
                    ESP_LOGW(TAG, "Changer: Poll timeout - resetting");
                    reader0x08.machine_state = INACTIVE_STATE;
                }
            }
        }

        uart_flush(UART_NUM_2);

        //--------------------------------------------------------------//
        // 0x30 Bill Validator
        //--------------------------------------------------------------//
        if (reader0x30.machine_state == INACTIVE_STATE) {

            mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_RESET & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*

            if (len == 1) {
                mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_POLL & BIT_CMD_SET);
                write_payload_controller_9(mdb_payload_tx, 1);

                len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 2, pdMS_TO_TICKS(await)); // Just Reset + CHK*

                if (len == 2) {
                    write_controller_9(ACK | BIT_MODE_SET);

                    if (mdb_payload_rx[0] == 0x06 /*Just Reset*/) {
                        reader0x30.machine_state = DISABLED_STATE;
                        ESP_LOGI(TAG, "Validator - Just Reset");
                    }
                }
            }

        } else if (reader0x30.machine_state == DISABLED_STATE) {

            mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_SETUP & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 28, pdMS_TO_TICKS(await)); // 27 + CHK*

            if (len == 28) {
                write_controller_9(ACK | BIT_MODE_SET);

                reader0x30.feature_level          = mdb_payload_rx[0];
                reader0x30.country_code           = (mdb_payload_rx[1] << 8) | mdb_payload_rx[2];
                reader0x30.scale_factor           = (mdb_payload_rx[3] << 8) | mdb_payload_rx[4];
                reader0x30.decimal_places         = mdb_payload_rx[5];
                reader0x30.bill_stacker_capacity  = (mdb_payload_rx[6] << 8) | mdb_payload_rx[7];
                reader0x30.bill_security_levels   = (mdb_payload_rx[8] << 8) | mdb_payload_rx[9];
                reader0x30.escrow_capability      = mdb_payload_rx[10];
                for (uint8_t i = 0; i < 16; i++)
                    reader0x30.bill_credit[i] = mdb_payload_rx[11 + i];

                ESP_LOGI(TAG, "Validator Setup: feature=%d scale=%d dec=%d capacity=%d escrow=%d",
                        reader0x30.feature_level, reader0x30.scale_factor,
                        reader0x30.decimal_places, reader0x30.bill_stacker_capacity,
                        reader0x30.escrow_capability);

                // Enable all bills + escrow by default; the real VMC (via
                // target bridge) can narrow this down later.
                mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_BILL_TYPE & BIT_CMD_SET);
                mdb_payload_tx[1] = 0xFF;
                mdb_payload_tx[2] = 0xFF;
                mdb_payload_tx[3] = 0xFF;
                mdb_payload_tx[4] = 0xFF;

                write_payload_controller_9(mdb_payload_tx, 5);

                len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*

                if (len == 1) {
                    reader0x30.machine_state = ENABLED_STATE;
                    ESP_LOGI(TAG, "Validator Enabled");

                    // Baseline stacker count for the "safe" RPC.
                    mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_STACKER & BIT_CMD_SET);
                    write_payload_controller_9(mdb_payload_tx, 1);

                    len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 3, pdMS_TO_TICKS(await)); // 2 + CHK*
                    if (len == 3) {
                        write_controller_9(ACK | BIT_MODE_SET);

                        uint16_t val = (mdb_payload_rx[0] << 8) | mdb_payload_rx[1];
                        reader0x30.stacker_full = (val & 0x8000) != 0;
                        reader0x30.stacker_count = val & 0x7FFF;
                    }
                }
            }

        } else {
            // ENABLED_STATE

            bill_from_target_evt_t bill_req;
            if (xQueueReceive(bill_from_target_queue, &bill_req, 0)) {

                if (bill_req.type == BILL_REQ_TYPE_ENABLE) {
                    mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_BILL_TYPE & BIT_CMD_SET);
                    mdb_payload_tx[1] = bill_req.bill_enable >> 8;
                    mdb_payload_tx[2] = bill_req.bill_enable;
                    mdb_payload_tx[3] = bill_req.bill_escrow_enable >> 8;
                    mdb_payload_tx[4] = bill_req.bill_escrow_enable;
                    write_payload_controller_9(mdb_payload_tx, 5);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await));

                } else if (bill_req.type == BILL_REQ_ESCROW) {
                    mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_ESCROW & BIT_CMD_SET);
                    mdb_payload_tx[1] = bill_req.escrow_command;
                    write_payload_controller_9(mdb_payload_tx, 2);
                    uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await));
                }

                uart_flush(UART_NUM_2);
            }

            mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_POLL & BIT_CMD_SET);
            write_payload_controller_9(mdb_payload_tx, 1);

            // 60ms over 30ms: a real validator can spread its multi-byte bill event past 30ms,
            // which truncated the read and lost the bill event. Costs ~30ms more per idle poll.
            len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 17, pdMS_TO_TICKS(60)); // events + CHK*

            if (len == 1) {
                reader0x30.poll_fail_count = 0;

            } else if (len > 1) {
                reader0x30.poll_fail_count = 0;
                write_controller_9(ACK | BIT_MODE_SET);

                if (len == 2 && mdb_payload_rx[0] == 0x06) {
                    reader0x30.machine_state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Validator reset detected");
                } else {
                    bool bill_stacked = false;

                    for (uint8_t i = 0; i + 1 < len; i++) {
                        uint8_t ev = mdb_payload_rx[i];

                        if ((ev & 0x80) && !(ev & 0x40)) { // 1000xxxx - bill stacked
                            uint8_t bill_type = ev & 0x0F;
                            reader0x30.credit += reader0x30.bill_credit[bill_type];
                            bill_stacked = true;

                            bill_stack_evt_t stack_evt = { bill_type };
                            xQueueSend(bill_to_target_queue, &stack_evt, 0);

                            ESP_LOGI(TAG, "Bill stacked: type=%d credit+=%d total=%d",
                                    bill_type, reader0x30.bill_credit[bill_type], reader0x30.credit);
                        }
                        else if ((ev & 0x90) == 0x90) { // 1001xxxx - bill in escrow
                            uint8_t bill_type = ev & 0x0F;
                            ESP_LOGI(TAG, "Bill in escrow: type=%d. Validating (Stacking)...", bill_type);

                            mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_ESCROW & BIT_CMD_SET);
                            mdb_payload_tx[1] = 0x01; // Stack the bill
                            write_payload_controller_9(mdb_payload_tx, 2);

                            uart_read_bytes(UART_NUM_2, mdb_payload_rx, 1, pdMS_TO_TICKS(await)); // ACK*
                        }
                        else if (ev == 0x01) ESP_LOGW(TAG, "Validator: Defective Motor");
                        else if (ev == 0x02) ESP_LOGW(TAG, "Validator: Sensor Problem");
                        else if (ev == 0x05) ESP_LOGW(TAG, "Validator: Jammed");
                        else if (ev == 0x08) ESP_LOGW(TAG, "Validator: Cash Box out of position");
                        else if (ev == 0x09) ESP_LOGW(TAG, "Validator: Disabled");
                    }

                    if (bill_stacked) {
                        // Refresh the real stacker count for the "safe" RPC.
                        mdb_payload_tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_STACKER & BIT_CMD_SET);
                        write_payload_controller_9(mdb_payload_tx, 1);

                        size_t stacker_len = uart_read_bytes(UART_NUM_2, mdb_payload_rx, 3, pdMS_TO_TICKS(await)); // 2 + CHK*
                        if (stacker_len == 3) {
                            write_controller_9(ACK | BIT_MODE_SET);

                            uint16_t val = (mdb_payload_rx[0] << 8) | mdb_payload_rx[1];
                            reader0x30.stacker_full = (val & 0x8000) != 0;
                            reader0x30.stacker_count = val & 0x7FFF;
                        }
                    }
                }

            } else {
                if (++reader0x30.poll_fail_count >= 10) { // 10: tolerate transient bus glitches
                    ESP_LOGW(TAG, "Validator: Poll timeout - resetting");
                    reader0x30.machine_state = INACTIVE_STATE;
                }
            }
        }
    }
}

//------------------------------------------------------------------------//
// mdb_target_task - slave: emulates cashless (0x10), coin changer (0x08)
// and bill validator (0x30) toward the vending machine's real VMC, on a
// single bit-banged bus shared by all three addresses.
//------------------------------------------------------------------------//

#define COIN_POLL_QSIZE 32
#define BILL_POLL_QSIZE 16

void mdb_target_task(void *pvParameters) {
    // Install the RX edge ISR from this task so the GPIO interrupt is allocated
    // on this task's core (APP_CPU/core 1), isolated from core-0 network ISRs
    // that would otherwise skew the bit-sampling timing.
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_mdb_target_rx, mdb_rx_falling_isr, NULL);

    uint8_t mdb_payload[36];

    // ---- Cashless (0x10) state ----
    machine_state_t cashless_state = INACTIVE_STATE;
    time_t cashless_session_begin_time = 0;
    uint16_t cashless_item_price = 0;
    uint16_t cashless_item_number = 0;

    // ---- Coin changer (0x08) state ----
    bool coin_reset_todo = false;
    uint16_t coin_enable = 0x0000;
    uint16_t coin_dispense_enable = 0x0000;
    uint8_t coin_poll_q[COIN_POLL_QSIZE];
    uint8_t coin_poll_q_head = 0, coin_poll_q_tail = 0;

    // ---- Bill validator (0x30) state ----
    bool bill_reset_todo = false;
    uint16_t bill_enable = 0x0000;
    uint16_t bill_escrow_enable = 0x0000;
    uint8_t bill_poll_q[BILL_POLL_QSIZE];
    uint8_t bill_poll_q_head = 0, bill_poll_q_tail = 0;

    for (;;) {
        uint8_t checksum = 0x00;
        uint8_t available_tx = 0;

        uint16_t coming_read = read_9(&checksum);

        if (!(coming_read & BIT_MODE_SET)) continue;

        if ((uint8_t) coming_read == ACK) continue;
        if ((uint8_t) coming_read == RET) continue;
        if ((uint8_t) coming_read == NAK) continue;

        uint8_t addr = coming_read & BIT_ADD_SET;
        uint8_t cmd  = coming_read & BIT_CMD_SET;

        //================================================================//
        // 0x10 Cashless
        //================================================================//
        if (addr == ADDR_CASHLESS) {

            switch (cmd) {
            case CSHL_RESET: {
                if (read_9(NULL) != checksum) continue;

                cashless_state = DISABLED_STATE;
                ESP_LOGI(TAG, "Cashless: RESET");
                break; // ACK
            }
            case CSHL_SETUP: {
                switch (read_9(&checksum)) {
                case 0x00: { // Config Data
                    (void) read_9(&checksum); // vmc_feature_level
                    (void) read_9(&checksum); // vmc_columns_on_display
                    (void) read_9(&checksum); // vmc_rows_on_display
                    (void) read_9(&checksum); // vmc_display_info

                    if (read_9(NULL) != checksum) continue;

                    bool external_ready = (reader0x10.machine_state != INACTIVE_STATE);

                    mdb_payload[0] = 0x01;
                    mdb_payload[1] = 1; // Feature Level
                    mdb_payload[2] = external_ready ? (reader0x10.country_code >> 8) : (CONFIG_MDB_CURRENCY_CODE >> 8);
                    mdb_payload[3] = external_ready ? (reader0x10.country_code & 0xFF) : (CONFIG_MDB_CURRENCY_CODE & 0xFF);
                    mdb_payload[4] = external_ready ? reader0x10.scale_factor : CASHLESS_DEFAULT_SCALE_FACTOR;
                    mdb_payload[5] = external_ready ? reader0x10.decimal_places : CASHLESS_DEFAULT_DECIMAL_PLACES;
                    mdb_payload[6] = 3;    // Response Time
                    mdb_payload[7] = 0b00001001; // Miscellaneous Options
                    available_tx = 8;

                    ESP_LOGI(TAG, "Cashless: CONFIG_DATA");
                    break;
                }
                case 0x01: { // Max/Min Prices
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);

                    if (read_9(NULL) != checksum) continue;

                    ESP_LOGI(TAG, "Cashless: MAX_MIN_PRICES");
                    break; // ACK
                }
                }
                break;
            }
            case CSHL_POLL: {
                if (read_9(NULL) != checksum) continue;

                cashless_to_target_evt_t evt;
                if (xQueueReceive(cashless_to_target_queue, &evt, 0)) {

                    bool accept = (evt.type == CSHL_EVT_BEGIN_SESSION && g_active_cashless_source == CASHLESS_SRC_NONE)
                               || (evt.source == g_active_cashless_source && g_active_cashless_source != CASHLESS_SRC_NONE);

                    if (accept) {
                        switch (evt.type) {
                        case CSHL_EVT_BEGIN_SESSION: {
                            g_active_cashless_source = evt.source;
                            cashless_state = IDLE_STATE;
                            mdb_payload[0] = 0x03;
                            mdb_payload[1] = evt.value >> 8;
                            mdb_payload[2] = evt.value;
                            available_tx = 3;
                            time(&cashless_session_begin_time);
                            ESP_LOGI(TAG, "Cashless: Begin Session (src=%d) funds=%u", evt.source, evt.value);
                            break;
                        }
                        case CSHL_EVT_VEND_APPROVED: {
                            cashless_item_price = evt.value;
                            mdb_payload[0] = 0x05;
                            mdb_payload[1] = evt.value >> 8;
                            mdb_payload[2] = evt.value;
                            available_tx = 3;
                            break;
                        }
                        case CSHL_EVT_VEND_DENIED: {
                            mdb_payload[0] = 0x06;
                            available_tx = 1;
                            cashless_state = IDLE_STATE;
                            break;
                        }
                        case CSHL_EVT_SESSION_END: {
                            mdb_payload[0] = 0x07;
                            available_tx = 1;
                            cashless_state = ENABLED_STATE;
                            g_active_cashless_source = CASHLESS_SRC_NONE;
                            break;
                        }
                        case CSHL_EVT_SESSION_CANCEL: {
                            mdb_payload[0] = 0x04;
                            available_tx = 1;
                            break;
                        }
                        }
                    }
                    // Event not for the currently active source: drop it (its
                    // owner keeps polling and will retry).
                } else if (cashless_state >= IDLE_STATE) {
                    time_t now = time(NULL);
                    if ((now - cashless_session_begin_time) > 60) {
                        // Local safety timeout: give up the arbitration lock so
                        // the other source can proceed even if this session's
                        // owner never sent an explicit End Session.
                        mdb_payload[0] = 0x04; // Session Cancel Request
                        available_tx = 1;
                        cashless_state = IDLE_STATE;
                        g_active_cashless_source = CASHLESS_SRC_NONE;
                        ESP_LOGW(TAG, "Cashless: session idle timeout, releasing arbitration");
                    }
                }
                break;
            }
            case CSHL_VEND: {
                switch (read_9(&checksum)) {
                case 0x00: { // Vend Request
                    cashless_item_price = (read_9(&checksum) << 8) | read_9(&checksum);
                    cashless_item_number = (read_9(&checksum) << 8) | read_9(&checksum);

                    if (read_9(NULL) != checksum) continue;

                    cashless_state = VEND_STATE;

                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_REQUEST, cashless_item_price, cashless_item_number };
                    QueueHandle_t dest = (g_active_cashless_source == CASHLESS_SRC_EXTERNAL)
                        ? cashless_from_target_to_controller_queue
                        : cashless_from_target_to_internal_queue;
                    xQueueSend(dest, &out, 0);

                    ESP_LOGI(TAG, "Cashless: VEND_REQUEST price=%u item=%u", cashless_item_price, cashless_item_number);
                    break; // ACK
                }
                case 0x01: { // Vend Cancel
                    if (read_9(NULL) != checksum) continue;

                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_CANCEL, 0, 0 };
                    QueueHandle_t dest = (g_active_cashless_source == CASHLESS_SRC_EXTERNAL)
                        ? cashless_from_target_to_controller_queue
                        : cashless_from_target_to_internal_queue;
                    xQueueSend(dest, &out, 0);
                    break; // ACK
                }
                case 0x02: { // Vend Success
                    cashless_item_number = (read_9(&checksum) << 8) | read_9(&checksum);

                    if (read_9(NULL) != checksum) continue;

                    cashless_state = IDLE_STATE;

                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_SUCCESS, cashless_item_price, cashless_item_number };
                    QueueHandle_t dest = (g_active_cashless_source == CASHLESS_SRC_EXTERNAL)
                        ? cashless_from_target_to_controller_queue
                        : cashless_from_target_to_internal_queue;
                    xQueueSend(dest, &out, 0);

                    ESP_LOGI(TAG, "Cashless: VEND_SUCCESS item=%u", cashless_item_number);
                    break; // ACK
                }
                case 0x03: { // Vend Failure
                    if (read_9(NULL) != checksum) continue;

                    cashless_state = IDLE_STATE;

                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_FAILURE, cashless_item_price, cashless_item_number };
                    QueueHandle_t dest = (g_active_cashless_source == CASHLESS_SRC_EXTERNAL)
                        ? cashless_from_target_to_controller_queue
                        : cashless_from_target_to_internal_queue;
                    xQueueSend(dest, &out, 0);

                    ESP_LOGI(TAG, "Cashless: VEND_FAILURE");
                    break; // ACK
                }
                case 0x04: { // Session Complete
                    if (read_9(NULL) != checksum) continue;

                    cashless_from_target_evt_t out = { CSHL_REQ_SESSION_COMPLETE, 0, 0 };
                    QueueHandle_t dest = (g_active_cashless_source == CASHLESS_SRC_EXTERNAL)
                        ? cashless_from_target_to_controller_queue
                        : cashless_from_target_to_internal_queue;
                    xQueueSend(dest, &out, 0);

                    ESP_LOGI(TAG, "Cashless: SESSION_COMPLETE");
                    break; // ACK
                }
                case 0x05: { // Cash Sale (informational, no physical cash path here)
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);

                    if (read_9(NULL) != checksum) continue;

                    break; // ACK
                }
                }
                break;
            }
            case CSHL_READER: {
                switch (read_9(&checksum)) {
                case 0x00: { // Reader Disable
                    if (read_9(NULL) != checksum) continue;
                    cashless_state = DISABLED_STATE;
                    break; // ACK
                }
                case 0x01: { // Reader Enable
                    if (read_9(NULL) != checksum) continue;
                    cashless_state = ENABLED_STATE;
                    break; // ACK
                }
                case 0x02: { // Reader Cancel
                    if (read_9(NULL) != checksum) continue;
                    mdb_payload[0] = 0x08;
                    available_tx = 1;
                    break;
                }
                }
                break;
            }
            case CSHL_EXPANSION: {
                switch (read_9(&checksum)) {
                case 0x00: { // Request ID
                    for (uint8_t x = 0; x < 29; x++) read_9(&checksum);

                    if (read_9(NULL) != checksum) continue;

                    mdb_payload[0] = 0x09;
                    memcpy(&mdb_payload[1], "VMF", 3);
                    memset(&mdb_payload[4], ' ', 12);
                    memset(&mdb_payload[16], ' ', 12);
                    mdb_payload[28] = 0x00;
                    mdb_payload[29] = 0x03;
                    available_tx = 30;

                    ESP_LOGI(TAG, "Cashless: REQUEST_ID");
                    break;
                }
                }
                break;
            }
            }

        //================================================================//
        // 0x08 Coin Changer
        //================================================================//
        } else if (addr == ADDR_CHANGER) {

            switch (cmd) {
            case CHGR_RESET: {
                if (read_9(NULL) != checksum) continue;

                coin_reset_todo = true;
                coin_enable = 0x0000;
                coin_dispense_enable = 0x0000;

                ESP_LOGI(TAG, "Changer: RESET");
                break; // ACK
            }
            case CHGR_SETUP: {
                if (read_9(NULL) != checksum) continue;

                bool physical_ready = (reader0x08.machine_state != INACTIVE_STATE);

                mdb_payload[0] = physical_ready ? reader0x08.feature_level : 0x01;
                mdb_payload[1] = physical_ready ? (reader0x08.country_code >> 8) : (CONFIG_MDB_CURRENCY_CODE >> 8);
                mdb_payload[2] = physical_ready ? (reader0x08.country_code & 0xFF) : (CONFIG_MDB_CURRENCY_CODE & 0xFF);
                mdb_payload[3] = physical_ready ? reader0x08.scale_factor : 1;
                mdb_payload[4] = physical_ready ? reader0x08.decimal_places : 2;
                mdb_payload[5] = physical_ready ? (reader0x08.coin_type_routing >> 8) : 0;
                mdb_payload[6] = physical_ready ? (reader0x08.coin_type_routing & 0xFF) : 0;
                for (int i = 0; i < 16; i++)
                    mdb_payload[7 + i] = physical_ready ? reader0x08.coin_credit[i] : 0;
                available_tx = 23;

                ESP_LOGI(TAG, "Changer: SETUP -> DISABLED (physical=%d)", physical_ready);
                break;
            }
            case CHGR_TUBE_STATUS: {
                if (read_9(NULL) != checksum) continue;

                bool physical_ready = (reader0x08.machine_state != INACTIVE_STATE);

                mdb_payload[0] = physical_ready ? (reader0x08.tube_full_status >> 8) : 0;
                mdb_payload[1] = physical_ready ? (reader0x08.tube_full_status & 0xFF) : 0;
                for (int i = 0; i < 16; i++)
                    mdb_payload[2 + i] = physical_ready ? reader0x08.tube_counts[i] : 0;
                available_tx = 18;

                break;
            }
            case CHGR_POLL: {
                if (read_9(NULL) != checksum) continue;

                coin_deposit_evt_t deposit_evt;
                while (xQueueReceive(coin_to_target_queue, &deposit_evt, 0)) {
                    uint8_t next = (coin_poll_q_head + 2) % COIN_POLL_QSIZE;
                    if (next != coin_poll_q_tail) {
                        coin_poll_q[coin_poll_q_head] = 0x40 | deposit_evt.coin_type; // routed to tube
                        coin_poll_q_head = (coin_poll_q_head + 1) % COIN_POLL_QSIZE;
                        coin_poll_q[coin_poll_q_head] = deposit_evt.tube_count;
                        coin_poll_q_head = (coin_poll_q_head + 1) % COIN_POLL_QSIZE;
                    } else {
                        ESP_LOGW(TAG, "Changer: poll queue full, coin event dropped");
                    }
                }

                if (coin_reset_todo) {
                    coin_reset_todo = false;
                    mdb_payload[0] = 0x00;
                    available_tx = 1;
                } else {
                    while (coin_poll_q_head != coin_poll_q_tail && available_tx < 16) {
                        mdb_payload[available_tx++] = coin_poll_q[coin_poll_q_tail];
                        coin_poll_q_tail = (coin_poll_q_tail + 1) % COIN_POLL_QSIZE;
                    }
                }
                break;
            }
            case CHGR_COIN_TYPE: {
                uint8_t en_high   = (uint8_t) read_9(&checksum);
                uint8_t en_low    = (uint8_t) read_9(&checksum);
                uint8_t disp_high = (uint8_t) read_9(&checksum);
                uint8_t disp_low  = (uint8_t) read_9(&checksum);
                if (read_9(NULL) != checksum) continue;

                coin_enable = ((uint16_t) en_high << 8) | en_low;
                coin_dispense_enable = ((uint16_t) disp_high << 8) | disp_low;

                coin_from_target_evt_t req = { COIN_REQ_TYPE_ENABLE, coin_enable, coin_dispense_enable, 0 };
                xQueueSend(coin_from_target_queue, &req, 0);

                ESP_LOGI(TAG, "Changer: COIN_TYPE enable=0x%04X dispense=0x%04X", coin_enable, coin_dispense_enable);
                break; // ACK
            }
            case CHGR_DISPENSE: {
                uint8_t disp_val = (uint8_t) read_9(&checksum);
                if (read_9(NULL) != checksum) continue;

                coin_from_target_evt_t req = { COIN_REQ_DISPENSE, 0, 0, disp_val };
                xQueueSend(coin_from_target_queue, &req, 0);

                ESP_LOGI(TAG, "Changer: DISPENSE 0x%02X", disp_val);
                break; // ACK
            }
            case CHGR_EXPANSION: {
                uint8_t sub = (uint8_t) read_9(&checksum);

                if (sub == 0x00) { // Request ID
                    if (read_9(NULL) != checksum) continue;

                    memcpy(&mdb_payload[0], "MDB", 3);
                    memset(&mdb_payload[3], ' ', 12);
                    memset(&mdb_payload[15], ' ', 12);
                    mdb_payload[27] = 0x00;
                    mdb_payload[28] = 0x01;
                    mdb_payload[29] = 0x00;
                    mdb_payload[30] = 0x00;
                    mdb_payload[31] = 0x00;
                    mdb_payload[32] = 0x00;
                    available_tx = 33;

                } else if (sub == 0x01) { // Feature Enable
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    (void) read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                }
                break;
            }
            }

        //================================================================//
        // 0x30 Bill Validator
        //================================================================//
        } else if (addr == ADDR_VALIDATOR) {

            switch (cmd) {
            case VLD_RESET: {
                if (read_9(NULL) != checksum) continue;

                bill_reset_todo = true;
                bill_enable = 0x0000;
                bill_escrow_enable = 0x0000;

                ESP_LOGI(TAG, "Validator: RESET");
                break; // ACK
            }
            case VLD_SETUP: {
                if (read_9(NULL) != checksum) continue;

                bool physical_ready = (reader0x30.machine_state != INACTIVE_STATE);

                mdb_payload[0] = physical_ready ? reader0x30.feature_level : 0x01;
                mdb_payload[1] = physical_ready ? (reader0x30.country_code >> 8) : (CONFIG_MDB_CURRENCY_CODE >> 8);
                mdb_payload[2] = physical_ready ? (reader0x30.country_code & 0xFF) : (CONFIG_MDB_CURRENCY_CODE & 0xFF);
                mdb_payload[3] = physical_ready ? (reader0x30.scale_factor >> 8) : 0x00;
                mdb_payload[4] = physical_ready ? (reader0x30.scale_factor & 0xFF) : 0x64;
                mdb_payload[5] = physical_ready ? reader0x30.decimal_places : 2;
                mdb_payload[6] = physical_ready ? (reader0x30.bill_stacker_capacity >> 8) : 0;
                mdb_payload[7] = physical_ready ? (reader0x30.bill_stacker_capacity & 0xFF) : 0;
                mdb_payload[8] = 0x00; mdb_payload[9] = 0x00; // Security Levels
                mdb_payload[10] = 0x00; // Escrow (bridge does not expose escrow upstream)
                for (int i = 0; i < 16; i++)
                    mdb_payload[11 + i] = physical_ready ? reader0x30.bill_credit[i] : 0;
                available_tx = 27;

                ESP_LOGI(TAG, "Validator: SETUP -> DISABLED (physical=%d)", physical_ready);
                break;
            }
            case VLD_SECURITY: {
                (void) read_9(&checksum);
                (void) read_9(&checksum);
                if (read_9(NULL) != checksum) continue;
                break; // ACK
            }
            case VLD_POLL: {
                if (read_9(NULL) != checksum) continue;

                bill_stack_evt_t stack_evt;
                while (xQueueReceive(bill_to_target_queue, &stack_evt, 0)) {
                    uint8_t next = (bill_poll_q_head + 1) % BILL_POLL_QSIZE;
                    if (next != bill_poll_q_tail) {
                        bill_poll_q[bill_poll_q_head] = 0x80 | stack_evt.bill_type; // stacked
                        bill_poll_q_head = next;
                    } else {
                        ESP_LOGW(TAG, "Validator: poll queue full, bill event dropped");
                    }
                }

                if (bill_reset_todo) {
                    bill_reset_todo = false;
                    mdb_payload[0] = 0x06; // Just Reset
                    available_tx = 1;
                } else {
                    while (bill_poll_q_head != bill_poll_q_tail && available_tx < 16) {
                        mdb_payload[available_tx++] = bill_poll_q[bill_poll_q_tail];
                        bill_poll_q_tail = (bill_poll_q_tail + 1) % BILL_POLL_QSIZE;
                    }
                }
                break;
            }
            case VLD_BILL_TYPE: {
                uint8_t en_high  = (uint8_t) read_9(&checksum);
                uint8_t en_low   = (uint8_t) read_9(&checksum);
                uint8_t esc_high = (uint8_t) read_9(&checksum);
                uint8_t esc_low  = (uint8_t) read_9(&checksum);
                if (read_9(NULL) != checksum) continue;

                bill_enable = ((uint16_t) en_high << 8) | en_low;
                bill_escrow_enable = ((uint16_t) esc_high << 8) | esc_low;

                bill_from_target_evt_t req = { BILL_REQ_TYPE_ENABLE, bill_enable, bill_escrow_enable, 0 };
                xQueueSend(bill_from_target_queue, &req, 0);

                ESP_LOGI(TAG, "Validator: BILL_TYPE enable=0x%04X escrow=0x%04X", bill_enable, bill_escrow_enable);
                break; // ACK
            }
            case VLD_ESCROW: {
                uint8_t escrow_cmd = (uint8_t) read_9(&checksum);
                if (read_9(NULL) != checksum) continue;

                bill_from_target_evt_t req = { BILL_REQ_ESCROW, 0, 0, escrow_cmd };
                xQueueSend(bill_from_target_queue, &req, 0);
                break; // ACK
            }
            case VLD_STACKER: {
                if (read_9(NULL) != checksum) continue;

                bool physical_ready = (reader0x30.machine_state != INACTIVE_STATE);
                uint16_t val = physical_ready ? ((reader0x30.stacker_full ? 0x8000 : 0) | (reader0x30.stacker_count & 0x7FFF)) : 0;
                mdb_payload[0] = val >> 8;
                mdb_payload[1] = val & 0xFF;
                available_tx = 2;
                break;
            }
            case VLD_EXPANSION: {
                uint8_t sub = (uint8_t) read_9(&checksum);

                if (sub == 0x00) { // Request ID
                    if (read_9(NULL) != checksum) continue;

                    memcpy(&mdb_payload[0], "VMF", 3);
                    memset(&mdb_payload[3], ' ', 12);
                    memset(&mdb_payload[15], ' ', 12);
                    mdb_payload[27] = 0x00;
                    mdb_payload[28] = 0x01;
                    available_tx = 29;
                }
                break;
            }
            }

        } else {
            continue; // not one of our addresses
        }

        write_payload_9(mdb_payload, available_tx);
    }
}

void app_main(void) {

    xLedEventGroup = xEventGroupCreate();
    xWifiEventGroup = xEventGroupCreate();

    xTaskCreate(led_status_task, "led_status_task", 2048, NULL, 1, NULL);
    xEventGroupSetBits(xLedEventGroup, BIT_STATUS_TRIGGER);

    //--------------- Bridge queues ---------------//
    cashless_to_target_queue = xQueueCreate(4, sizeof(cashless_to_target_evt_t));
    cashless_from_target_to_controller_queue = xQueueCreate(4, sizeof(cashless_from_target_evt_t));
    cashless_from_target_to_internal_queue = xQueueCreate(4, sizeof(cashless_from_target_evt_t));

    coin_to_target_queue = xQueueCreate(16, sizeof(coin_deposit_evt_t));
    coin_from_target_queue = xQueueCreate(4, sizeof(coin_from_target_evt_t));

    bill_to_target_queue = xQueueCreate(16, sizeof(bill_stack_evt_t));
    bill_from_target_queue = xQueueCreate(4, sizeof(bill_from_target_evt_t));

    mdb_rx_queue = xQueueCreate(64, sizeof(uint16_t));

    //--------------- Controller port (UART2, master) ---------------//
    uart_config_t uart_config_2 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, pin_mdb_controller_tx, pin_mdb_controller_rx, -1, -1);
    uart_driver_install(UART_NUM_2, 256, 256, 0, (void*) 0, 0);

    //--------------- Target port (bit-banged GPIO, slave) ---------------//
    gpio_config_t target_rx_cfg = {
        .pin_bit_mask = 1ULL << pin_mdb_target_rx,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&target_rx_cfg);

    gpio_config_t target_tx_cfg = {
        .pin_bit_mask = 1ULL << pin_mdb_target_tx,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&target_tx_cfg);
    gpio_set_level(pin_mdb_target_tx, 1);

    // Bit-banged target port + UART-driven controller port both pinned to
    // core 1, isolated from the WiFi/BLE stacks (core 0) so their ISRs and
    // scheduling jitter never skew MDB bit timing.
    xTaskCreatePinnedToCore(mdb_controller_task, "controller_task", 6765, (void*) 0, 1, (void*) 0, 1);
    xTaskCreatePinnedToCore(mdb_target_task, "target_task", 6765, (void*) 0, 1, (void*) 0, 1);

    //--------------- NVS + network stack ---------------//
    nvs_flash_init();

    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_t *wifi_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_route_prio(wifi_netif, 200);

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, NULL, NULL);

    const esp_timer_create_args_t wifi_timer_args = {
        .callback = wifi_retry_cb,
        .name = "wifi_retry",
    };
    esp_timer_create(&wifi_timer_args, &wifi_retry_timer);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    //--------------- SNTP ---------------//
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    //--------------- BLE (internal cashless engine) ---------------//
    char myhost[64];
    strcpy(myhost, "0.vmflow.xyz");

    nvs_handle_t handle;
    if (nvs_open("vmflow", NVS_READONLY, &handle) == ESP_OK) {
        size_t s_len = 0;
        if (nvs_get_str(handle, "passkey", NULL, &s_len) == ESP_OK) {
            nvs_get_str(handle, "passkey", my_passkey, &s_len);

            if (nvs_get_str(handle, "domain", NULL, &s_len) == ESP_OK) {
                nvs_get_str(handle, "domain", my_subdomain, &s_len);

                snprintf(myhost, sizeof(myhost), "%s.vmflow.xyz", my_subdomain);

                xEventGroupSetBits(xLedEventGroup, BIT_STATUS_PASSKEY | BIT_STATUS_DOMAIN | BIT_STATUS_TRIGGER);
            }
        }
        nvs_close(handle);
    }

    // HMAC key tracks the passkey buffer by reference; later BLE provisioning
    // writes into the same buffer and takes effect without re-registering.
    rpc_auth_set_key(my_passkey);

    ble_init(myhost, ble_event_handler, ble_pax_event_handler);

    esp_timer_handle_t periodic_pax_timer;
    const esp_timer_create_args_t periodic_pax_timer_args = {
        .callback = &ble_scan_start,
        .arg      = (void*) (uintptr_t) PAX_SCAN_DURATION_SEC,
        .name     = "task_paxcounter",
    };
    esp_timer_create(&periodic_pax_timer_args, &periodic_pax_timer);
    esp_timer_start_periodic(periodic_pax_timer, PAX_SCAN_INTERVAL_US);

    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);
    xTaskCreate(cashless_internal_task, "cashless_internal_task", 4096, NULL, 2, NULL);
}
