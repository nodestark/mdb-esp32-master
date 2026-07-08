#ifndef MDB_MASTER_ESP32S3_H
#define MDB_MASTER_ESP32S3_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

// Read-only snapshot of bridge state, for external monitor consumers
// (USB CDC1 monitor protocol, usb-monitor.c). Mirrors the same fields
// already published via rpc_publish_info()/rpc_publish_safe() over MQTT.
typedef struct {
    uint64_t uptime_s;
    uint32_t free_heap;
    uint32_t min_free_heap;
    int      cashless_source;      // cashless_src_t: 0=NONE,1=INTERNAL,2=EXTERNAL
    int      changer_state;        // machine_state_t (reader0x08)
    int      validator_state;      // machine_state_t (reader0x30)
    uint16_t last_sale_price;
    uint16_t last_sale_item;
    int64_t  last_vend_success_time;
    char     ip_wifi[16];

    bool     coin_present;
    double   coin_tube_total_cents;
    bool     coin_tube_full;

    bool     bill_present;
    uint16_t bill_stacker_count;
    bool     bill_stacker_full;
    double   bill_stacker_value_cents_since_reset;
} mdb_bridge_snapshot_t;

// Fills *out with the current bridge state. Safe to call from any task:
// every field it reads is already read cross-task without locking by
// rpc_publish_info()/rpc_publish_safe() today (mqtt_task) - this follows
// the same lock-free, single-writer/multi-reader-of-scalars pattern.
void mdb_bridge_get_snapshot(mdb_bridge_snapshot_t *out);

#endif
