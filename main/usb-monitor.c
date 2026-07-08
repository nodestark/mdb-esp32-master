/*
 * VMflow.xyz
 *
 * usb-monitor.c - USB CDC read-only ASCII monitor protocol
 *
 * The board has a single native USB port. This module turns it into a
 * TinyUSB composite device exposing two virtual CDC-ACM ports over that
 * same cable:
 *  - CDC0: console (ESP_LOGx/printf), replacing the USB_SERIAL_JTAG
 *    console the board used before.
 *  - CDC1: a line-based ASCII protocol modeled on qibixx's documented
 *    MDB-USB Interface API (V/H general commands, plus a status query
 *    group) so a PC-side terminal/app can watch the bridge over USB.
 *
 * CDC1 is strictly read-only: it reports mdb_bridge_get_snapshot() state
 * and nothing else. It does not accept credit-injection or any other
 * command that mutates bridge state - that stays on the BLE/MQTT paths.
 */

#include <string.h>
#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_app_desc.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"

#include "mdb-master-esp32s3.h"
#include "usb-monitor.h"

#define TAG "usb_monitor"

#define VMFLOW_HW_REV      "1.0"   // placeholder - no hardware-revision constant exists yet
#define VMFLOW_HW_CAPFLAGS  0x0000 // reserved, 0 = read-only monitor, no write capability

static char s_line_buf[96];
static size_t s_line_len;

static void usb_monitor_reply(const char *resp) {
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_1, (const uint8_t *) resp, strlen(resp));
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_1, pdMS_TO_TICKS(20));
}

static void usb_monitor_dispatch(const char *line) {
    char resp[160];

    if (strcmp(line, "V") == 0) {
        const esp_app_desc_t *app = esp_app_get_description();
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(resp, sizeof(resp), "v,%s,%02x%02x%02x%02x%02x%02x\n",
            app->version, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    } else if (strcmp(line, "H") == 0) {
        snprintf(resp, sizeof(resp), "h,%s,%04x\n", VMFLOW_HW_REV, VMFLOW_HW_CAPFLAGS);

    } else if (strcmp(line, "S") == 0) {
        mdb_bridge_snapshot_t snap;
        mdb_bridge_get_snapshot(&snap);
        snprintf(resp, sizeof(resp), "s,%d,%d,%d\n",
            snap.cashless_source, snap.changer_state, snap.validator_state);

    } else if (strcmp(line, "C") == 0) {
        mdb_bridge_snapshot_t snap;
        mdb_bridge_get_snapshot(&snap);
        snprintf(resp, sizeof(resp), "c,%d,%.0f,%d\n",
            snap.coin_present ? 1 : 0, snap.coin_tube_total_cents, snap.coin_tube_full ? 1 : 0);

    } else if (strcmp(line, "B") == 0) {
        mdb_bridge_snapshot_t snap;
        mdb_bridge_get_snapshot(&snap);
        snprintf(resp, sizeof(resp), "b,%d,%u,%d,%.0f\n",
            snap.bill_present ? 1 : 0, snap.bill_stacker_count, snap.bill_stacker_full ? 1 : 0,
            snap.bill_stacker_value_cents_since_reset);

    } else if (strcmp(line, "L") == 0) {
        mdb_bridge_snapshot_t snap;
        mdb_bridge_get_snapshot(&snap);
        snprintf(resp, sizeof(resp), "l,%u,%u,%lld\n",
            snap.last_sale_price, snap.last_sale_item, (long long) snap.last_vend_success_time);

    } else if (strcmp(line, "U") == 0) {
        mdb_bridge_snapshot_t snap;
        mdb_bridge_get_snapshot(&snap);
        snprintf(resp, sizeof(resp), "u,%llu,%lu,%lu,%s\n",
            (unsigned long long) snap.uptime_s, (unsigned long) snap.free_heap, (unsigned long) snap.min_free_heap, snap.ip_wifi);

    } else if (strcmp(line, "?") == 0) {
        snprintf(resp, sizeof(resp), "?,V,H,S,C,B,L,U\n");

    } else {
        snprintf(resp, sizeof(resp), "e,unknown_command\n");
    }

    usb_monitor_reply(resp);
}

static void usb_monitor_cdc1_rx_cb(int itf, cdcacm_event_t *event) {
    if (event->type != CDC_EVENT_RX)
        return;

    uint8_t buf[64];
    size_t rx_size = 0;
    if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_1, buf, sizeof(buf), &rx_size) != ESP_OK)
        return;

    for (size_t i = 0; i < rx_size; i++) {
        char c = (char) buf[i];

        if (c == '\n' || c == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                usb_monitor_dispatch(s_line_buf);
                s_line_len = 0;
            }
            continue;
        }

        if (s_line_len >= sizeof(s_line_buf) - 1) {
            // Overlong/garbage line - drop it and resync on the next newline.
            s_line_len = 0;
            continue;
        }

        s_line_buf[s_line_len++] = c;
    }
}

void usb_monitor_init(void) {
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t cdc0_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc0_cfg));
    ESP_ERROR_CHECK(tinyusb_console_init(TINYUSB_CDC_ACM_0));

    tinyusb_config_cdcacm_t cdc1_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_1,
        .callback_rx = usb_monitor_cdc1_rx_cb,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc1_cfg));

    ESP_LOGI(TAG, "USB monitor ready: CDC0=console, CDC1=read-only ASCII protocol");
}
