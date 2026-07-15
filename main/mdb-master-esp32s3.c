/*
 * VMflow.xyz
 *
 * mdb-master-esp32s3.c - MDB controller <-> target bridge (cashless + coin + bill)
 *
 * Two independent MDB ports:
 *  - controller port (UART2, master): drives physical cashless (0x10), coin
 *    changer (0x08) and bill validator (0x30). Plug-and-play: each device
 *    starts INACTIVE and is probed every cycle until it responds.
 *  - target port (bit-banged GPIO, slave): emulates cashless (0x10), coin
 *    changer (0x08) and bill validator (0x30) toward the vending machine VMC.
 *    Always online: responds with defaults so the VMC never sees a missing
 *    device even before a physical device is detected on the controller port.
 *
 * Events bridge via queues in both directions:
 *  - cashless_to_target_queue  : session/vend events → target (exposed extern
 *    so future BLE/MQTT payment engines can push sessions without arbitration
 *    logic living here).
 *  - cashless_from_target_queue: vend requests/outcomes ← target (extern, same
 *    reason — consumer decides what to do with them).
 *  - coin_to_target_queue / coin_from_target_queue : coin changer bridge.
 *  - bill_to_target_queue / bill_from_target_queue : bill validator bridge.
 */

#include <string.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <rom/ets_sys.h>
#include <soc/gpio_struct.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define TAG "mdb_bridge"

#define PIN_TARGET_RX     GPIO_NUM_4
#define PIN_TARGET_TX     GPIO_NUM_5
#define PIN_CONTROLLER_RX GPIO_NUM_1
#define PIN_CONTROLLER_TX GPIO_NUM_2

#define ACK          0x00
#define RET          0xAA
#define NAK          0xFF

#define BIT_MODE_SET 0b100000000
#define BIT_ADD_SET  0b011111000
#define BIT_CMD_SET  0b000000111

#define ADDR_CASHLESS  0x10
#define ADDR_CHANGER   0x08
#define ADDR_VALIDATOR 0x30

// Cashless Device (0x10) command set
enum CASHLESS_CMD {
    CSHL_RESET     = 0x00,
    CSHL_SETUP     = 0x01,
    CSHL_POLL      = 0x02,
    CSHL_VEND      = 0x03,
    CSHL_READER    = 0x04,
    CSHL_EXPANSION = 0x07,
};

// Coin Changer (0x08) command set
enum CHANGER_CMD {
    CHGR_RESET       = 0x00,
    CHGR_SETUP       = 0x01,
    CHGR_TUBE_STATUS = 0x02,
    CHGR_POLL        = 0x03,
    CHGR_COIN_TYPE   = 0x04,
    CHGR_DISPENSE    = 0x05,
    CHGR_EXPANSION   = 0x07,
};

// Bill Validator (0x30) command set
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

typedef enum {
    INACTIVE_STATE,
    DISABLED_STATE,
    ENABLED_STATE,
    IDLE_STATE,   // cashless: session open, waiting for vend request
    VEND_STATE,   // cashless: vend request sent, waiting for outcome
} device_state_t;

// Physical device snapshots — written only by the controller task, read by
// the target task at SETUP time to mirror the real device's configuration.
typedef struct {
    uint8_t        feature_level;
    uint16_t       country_code;
    uint8_t        scale_factor;
    uint8_t        decimal_places;
    uint8_t        poll_fail_count;
    device_state_t state;
} cashless_t;

typedef struct {
    uint8_t        feature_level;
    uint16_t       country_code;
    uint8_t        scale_factor;
    uint8_t        decimal_places;
    uint16_t       coin_type_routing;
    uint16_t       tube_full_status;
    uint8_t        tube_counts[16];
    uint8_t        coin_credit[16];
    uint8_t        poll_fail_count;
    device_state_t state;
} changer_t;

typedef struct {
    uint8_t        feature_level;
    uint16_t       country_code;
    uint16_t       scale_factor;
    uint8_t        decimal_places;
    uint16_t       bill_stacker_capacity;
    uint8_t        bill_credit[16];
    uint16_t       stacker_count;
    bool           stacker_full;
    uint8_t        poll_fail_count;
    device_state_t state;
} validator_t;

static cashless_t  phys_cashless  = { .state = INACTIVE_STATE };
static changer_t   phys_changer   = { .state = INACTIVE_STATE };
static validator_t phys_validator = { .state = INACTIVE_STATE };

// Defaults used by the target emulation when no physical device is present.
#define CASHLESS_DEFAULT_SCALE_FACTOR   1
#define CASHLESS_DEFAULT_DECIMAL_PLACES 2

static const uint8_t CHANGER_DEFAULT_COIN_CREDIT[16]  = {5, 10, 25, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define CHANGER_DEFAULT_FEATURE_LEVEL  0x03
#define CHANGER_DEFAULT_COIN_ROUTING   0x000F

static const uint8_t VALIDATOR_DEFAULT_BILL_CREDIT[16] = {1, 2, 5, 10, 20, 50, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define VALIDATOR_DEFAULT_SCALE_FACTOR 0x0064

//------------------------------------------------------------------------//
// Bridge queues
//
// cashless_to_target_queue and cashless_from_target_queue are exposed as
// non-static so future payment engines (BLE, MQTT) added in other
// compilation units can produce/consume session events without changing the
// bridge logic here.
//------------------------------------------------------------------------//

typedef enum {
    CSHL_EVT_BEGIN_SESSION,  // value = funds_available
    CSHL_EVT_VEND_APPROVED,  // value = item_price
    CSHL_EVT_VEND_DENIED,
    CSHL_EVT_SESSION_END,
    CSHL_EVT_SESSION_CANCEL,
} cashless_to_target_type_t;

typedef struct {
    cashless_to_target_type_t type;
    uint16_t value;
} cashless_to_target_evt_t;

QueueHandle_t cashless_to_target_queue;   // → target (extern: payment engines push here)

typedef enum {
    CSHL_REQ_VEND_REQUEST,
    CSHL_REQ_VEND_CANCEL,
    CSHL_REQ_VEND_SUCCESS,
    CSHL_REQ_VEND_FAILURE,
    CSHL_REQ_SESSION_COMPLETE,
} cashless_from_target_type_t;

typedef struct {
    cashless_from_target_type_t type;
    uint16_t item_price;
    uint16_t item_number;
} cashless_from_target_evt_t;

QueueHandle_t cashless_from_target_queue; // ← target (extern: payment engines consume)

typedef struct {
    uint8_t coin_type;
    uint8_t tube_count;
    uint8_t routing;
} coin_deposit_evt_t;
static QueueHandle_t coin_to_target_queue;

typedef struct {
    enum { COIN_REQ_TYPE_ENABLE, COIN_REQ_DISPENSE } type;
    uint16_t coin_enable;
    uint16_t dispense_enable;
    uint8_t  dispense_value;
} coin_from_target_evt_t;
static QueueHandle_t coin_from_target_queue;

typedef struct { uint8_t bill_type; } bill_stack_evt_t;
static QueueHandle_t bill_to_target_queue;

typedef struct {
    enum { BILL_REQ_TYPE_ENABLE, BILL_REQ_ESCROW } type;
    uint16_t bill_enable;
    uint16_t bill_escrow_enable;
    uint8_t  escrow_command;
} bill_from_target_evt_t;
static QueueHandle_t bill_from_target_queue;

//------------------------------------------------------------------------//
// Target port: bit-banged 9-bit MDB I/O (slave toward VMC)
//------------------------------------------------------------------------//

static QueueHandle_t mdb_rx_queue;

static void IRAM_ATTR mdb_rx_falling_isr(void *arg) {
    gpio_intr_disable(PIN_TARGET_RX);

    uint16_t v = 0;
    ets_delay_us(156); // skip start bit

    for (int x = 0; x < 9; x++) {
        v |= (gpio_get_level(PIN_TARGET_RX) << x);
        ets_delay_us(104);
    }
    xQueueSendFromISR(mdb_rx_queue, &v, NULL);

    gpio_intr_enable(PIN_TARGET_RX);
}

static uint16_t read_9(uint8_t *checksum) {
    uint16_t v = 0;
    xQueueReceive(mdb_rx_queue, &v, portMAX_DELAY);
    if (checksum) *checksum += (uint8_t)v;
    return v;
}

static void write_9(uint16_t nth9) {
    gpio_set_level(PIN_TARGET_TX, 0);
    ets_delay_us(104);
    for (uint8_t x = 0; x < 9; x++) {
        gpio_set_level(PIN_TARGET_TX, (nth9 >> x) & 1);
        ets_delay_us(104);
    }
    gpio_set_level(PIN_TARGET_TX, 1);
    ets_delay_us(104);
}

static void write_payload_9(uint8_t *payload, uint8_t length) {
    uint8_t checksum = 0x00;

    // Half-duplex: TX (GPIO5) couples into adjacent RX pin (GPIO4); disable
    // RX ISR while transmitting to avoid spurious frames desync-ing reception.
    gpio_intr_disable(PIN_TARGET_RX);

    for (int x = 0; x < length; x++) {
        checksum += payload[x];
        write_9(payload[x]);
    }
    write_9(BIT_MODE_SET | checksum);

    ets_delay_us(200);
    xQueueReset(mdb_rx_queue);
    GPIO.status_w1tc = (1U << PIN_TARGET_RX); // clear latched RX edge
    gpio_intr_enable(PIN_TARGET_RX);
}

//------------------------------------------------------------------------//
// Controller port: hardware UART2 9-bit MDB I/O (master toward peripherals)
//------------------------------------------------------------------------//

static void write_controller_9(uint16_t nth9) {
    uint8_t ones = __builtin_popcount((uint8_t)nth9);

    // The MDB 9th (mode) bit is emulated via UART parity. uart_set_parity()
    // must settle before the byte is shifted out, so wait for TX to drain
    // first; otherwise the mode bit of commands following a long read arrives
    // with stale parity and the peripheral drops them.
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(250));

    if ((nth9 >> 8) & 1)
        uart_set_parity(UART_NUM_2, ones % 2 ? UART_PARITY_EVEN : UART_PARITY_ODD);
    else
        uart_set_parity(UART_NUM_2, ones % 2 ? UART_PARITY_ODD : UART_PARITY_EVEN);

    uart_write_bytes(UART_NUM_2, (uint8_t *)&nth9, 1);
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(250));
}

static void write_payload_controller_9(uint8_t *payload, uint8_t length) {
    uint8_t checksum = 0;
    write_controller_9((checksum = payload[0]) | BIT_MODE_SET);
    for (uint8_t x = 1; x < length; x++) {
        write_controller_9(payload[x]);
        checksum += payload[x];
    }
    write_controller_9(checksum);
}

//------------------------------------------------------------------------//
// mdb_controller_task - drives physical cashless, coin changer and bill
// validator; relays events and commands to/from the target task via queues.
//------------------------------------------------------------------------//

void mdb_controller_task(void *pvParameters) {
    uint8_t tx[36], rx[36];
    size_t  len;
    const uint8_t await = 125; // ms

    for (;;) {
        uart_flush(UART_NUM_2);

        //------------------------------------------------------------------//
        // 0x10 Cashless (plug and play)
        //------------------------------------------------------------------//
        if (phys_cashless.state == INACTIVE_STATE) {

            tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_RESET & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));

            if (len == 1) {
                tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_POLL & BIT_CMD_SET);
                write_payload_controller_9(tx, 1);
                len = uart_read_bytes(UART_NUM_2, rx, 2, pdMS_TO_TICKS(await));

                if (len == 2 && rx[0] == 0x00 /*Just Reset*/) {
                    write_controller_9(ACK | BIT_MODE_SET);

                    // SETUP Config Data
                    tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_SETUP & BIT_CMD_SET);
                    tx[1] = 0x00; tx[2] = 1; tx[3] = 0; tx[4] = 0; tx[5] = 0b00000001;
                    write_payload_controller_9(tx, 6);
                    len = uart_read_bytes(UART_NUM_2, rx, 9, pdMS_TO_TICKS(await));

                    if (len == 9) {
                        write_controller_9(ACK | BIT_MODE_SET);
                        phys_cashless.feature_level   = rx[1];
                        phys_cashless.country_code    = ((uint16_t)rx[2] << 8) | rx[3];
                        phys_cashless.scale_factor    = rx[4];
                        phys_cashless.decimal_places  = rx[5];
                        phys_cashless.state           = DISABLED_STATE;
                        ESP_LOGI(TAG, "Cashless: Config scale=%d dec=%d",
                                 phys_cashless.scale_factor, phys_cashless.decimal_places);
                    }
                }
            }

        } else if (phys_cashless.state == DISABLED_STATE) {

            // SETUP Max/Min Prices
            tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_SETUP & BIT_CMD_SET);
            tx[1] = 0x01; tx[2] = 0xFF; tx[3] = 0xFF; tx[4] = 0x00; tx[5] = 0x00;
            write_payload_controller_9(tx, 6);
            len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
            if (len != 1) { phys_cashless.state = INACTIVE_STATE; goto next_changer; }

            // EXPANSION Request ID
            tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_EXPANSION & BIT_CMD_SET);
            tx[1] = 0x00;
            memcpy(&tx[2], "VMF", 3);
            memset(&tx[5], ' ', 12);
            memset(&tx[17], ' ', 12);
            tx[29] = '0'; tx[30] = '1';
            write_payload_controller_9(tx, 31);
            len = uart_read_bytes(UART_NUM_2, rx, 31, pdMS_TO_TICKS(await));
            if (len == 31) write_controller_9(ACK | BIT_MODE_SET);

            // Reader Enable
            tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_READER & BIT_CMD_SET);
            tx[1] = 0x01;
            write_payload_controller_9(tx, 2);
            len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
            if (len != 1) { phys_cashless.state = INACTIVE_STATE; goto next_changer; }

            phys_cashless.state = ENABLED_STATE;
            ESP_LOGI(TAG, "Cashless: Reader Enabled");

        } else { // ENABLED / IDLE / VEND

            // Relay pending commands from the target to the physical device.
            cashless_from_target_evt_t req;
            if (xQueueReceive(cashless_from_target_queue, &req, 0)) {
                switch (req.type) {
                case CSHL_REQ_VEND_REQUEST:
                    if (phys_cashless.state == IDLE_STATE) {
                        tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                        tx[1] = 0x00;
                        tx[2] = req.item_price >> 8; tx[3] = req.item_price;
                        tx[4] = req.item_number >> 8; tx[5] = req.item_number;
                        write_payload_controller_9(tx, 6);
                        len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                        if (len == 1) phys_cashless.state = VEND_STATE;
                    }
                    break;
                case CSHL_REQ_VEND_CANCEL:
                    tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    tx[1] = 0x01;
                    write_payload_controller_9(tx, 2);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                    break;
                case CSHL_REQ_VEND_SUCCESS:
                    tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    tx[1] = 0x02;
                    tx[2] = req.item_number >> 8; tx[3] = req.item_number;
                    write_payload_controller_9(tx, 4);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                    phys_cashless.state = IDLE_STATE;
                    break;
                case CSHL_REQ_VEND_FAILURE:
                    tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    tx[1] = 0x03;
                    write_payload_controller_9(tx, 2);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                    phys_cashless.state = IDLE_STATE;
                    break;
                case CSHL_REQ_SESSION_COMPLETE:
                    tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_VEND & BIT_CMD_SET);
                    tx[1] = 0x04;
                    write_payload_controller_9(tx, 2);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                    break;
                }
                uart_flush(UART_NUM_2);
            }

            // Poll the physical cashless device.
            tx[0] = (ADDR_CASHLESS & BIT_ADD_SET) | (CSHL_POLL & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 4, pdMS_TO_TICKS(await));

            if (len >= 1) {
                phys_cashless.poll_fail_count = 0;

                if (rx[0] == 0x03 && len >= 4) { // Begin Session
                    write_controller_9(ACK | BIT_MODE_SET);
                    uint16_t funds = ((uint16_t)rx[1] << 8) | rx[2];
                    phys_cashless.state = IDLE_STATE;
                    cashless_to_target_evt_t evt = { CSHL_EVT_BEGIN_SESSION, funds };
                    xQueueSend(cashless_to_target_queue, &evt, 0);
                    ESP_LOGI(TAG, "Cashless: Begin Session funds=%u", funds);

                } else if (rx[0] == 0x04 && len >= 2) { // Session Cancel Request
                    write_controller_9(ACK | BIT_MODE_SET);
                    cashless_to_target_evt_t evt = { CSHL_EVT_SESSION_CANCEL, 0 };
                    xQueueSend(cashless_to_target_queue, &evt, 0);

                } else if (rx[0] == 0x05 && len >= 4) { // Vend Approved
                    write_controller_9(ACK | BIT_MODE_SET);
                    uint16_t price = ((uint16_t)rx[1] << 8) | rx[2];
                    cashless_to_target_evt_t evt = { CSHL_EVT_VEND_APPROVED, price };
                    xQueueSend(cashless_to_target_queue, &evt, 0);

                } else if (rx[0] == 0x06 && len >= 2) { // Vend Denied
                    write_controller_9(ACK | BIT_MODE_SET);
                    phys_cashless.state = IDLE_STATE;
                    cashless_to_target_evt_t evt = { CSHL_EVT_VEND_DENIED, 0 };
                    xQueueSend(cashless_to_target_queue, &evt, 0);

                } else if (rx[0] == 0x07 && len >= 2) { // End Session
                    write_controller_9(ACK | BIT_MODE_SET);
                    phys_cashless.state = ENABLED_STATE;
                    cashless_to_target_evt_t evt = { CSHL_EVT_SESSION_END, 0 };
                    xQueueSend(cashless_to_target_queue, &evt, 0);

                } else if (rx[0] == 0x0B && len >= 2) { // Out of Sequence
                    write_controller_9(ACK | BIT_MODE_SET);
                    phys_cashless.state = INACTIVE_STATE;
                }
                // 0x00 ACK (nothing to report) — no action needed

            } else {
                if (++phys_cashless.poll_fail_count >= 10) {
                    phys_cashless.state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Cashless: poll timeout, resetting");
                }
            }
        }

        next_changer:
        uart_flush(UART_NUM_2);

        //------------------------------------------------------------------//
        // 0x08 Coin Changer (plug and play)
        //------------------------------------------------------------------//
        if (phys_changer.state == INACTIVE_STATE) {

            tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_RESET & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));

            if (len == 1) {
                tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_POLL & BIT_CMD_SET);
                write_payload_controller_9(tx, 1);
                len = uart_read_bytes(UART_NUM_2, rx, 2, pdMS_TO_TICKS(await));

                if (len == 2 && rx[0] == 0x0B) {
                    write_controller_9(ACK | BIT_MODE_SET);
                    phys_changer.state = DISABLED_STATE;
                    ESP_LOGI(TAG, "Changer: Just Reset");
                }
            }

        } else if (phys_changer.state == DISABLED_STATE) {

            tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_SETUP & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 24, pdMS_TO_TICKS(await));

            if (len == 24) {
                uart_flush_input(UART_NUM_2);
                write_controller_9(ACK | BIT_MODE_SET);

                phys_changer.feature_level     = rx[0];
                phys_changer.country_code      = ((uint16_t)rx[1] << 8) | rx[2];
                phys_changer.scale_factor      = rx[3];
                phys_changer.decimal_places    = rx[4];
                phys_changer.coin_type_routing = ((uint16_t)rx[5] << 8) | rx[6];
                for (uint8_t i = 0; i < 16; i++) phys_changer.coin_credit[i] = rx[7 + i];

                ESP_LOGI(TAG, "Changer Setup: feature=%d scale=%d dec=%d routing=0x%04X",
                         phys_changer.feature_level, phys_changer.scale_factor,
                         phys_changer.decimal_places, phys_changer.coin_type_routing);

                vTaskDelay(pdMS_TO_TICKS(5));

                tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_TUBE_STATUS & BIT_CMD_SET);
                write_payload_controller_9(tx, 1);
                len = uart_read_bytes(UART_NUM_2, rx, 19, pdMS_TO_TICKS(await));
                if (len == 19) {
                    uart_flush_input(UART_NUM_2);
                    write_controller_9(ACK | BIT_MODE_SET);
                    phys_changer.tube_full_status = ((uint16_t)rx[0] << 8) | rx[1];
                    for (uint8_t i = 0; i < 16; i++) phys_changer.tube_counts[i] = rx[2 + i];
                }

                vTaskDelay(pdMS_TO_TICKS(5));

                tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_COIN_TYPE & BIT_CMD_SET);
                tx[1] = 0xFF; tx[2] = 0xFF; tx[3] = 0x00; tx[4] = 0x00;
                write_payload_controller_9(tx, 5);
                len = uart_read_bytes(UART_NUM_2, rx, 5, pdMS_TO_TICKS(await));
                if (len == 5) write_controller_9(ACK | BIT_MODE_SET);

                phys_changer.state = ENABLED_STATE;
                ESP_LOGI(TAG, "Changer Enabled");
            }

        } else {

            coin_from_target_evt_t req;
            if (xQueueReceive(coin_from_target_queue, &req, 0)) {
                if (req.type == COIN_REQ_TYPE_ENABLE) {
                    tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_COIN_TYPE & BIT_CMD_SET);
                    tx[1] = req.coin_enable >> 8;     tx[2] = req.coin_enable;
                    tx[3] = req.dispense_enable >> 8; tx[4] = req.dispense_enable;
                    write_payload_controller_9(tx, 5);
                    uart_read_bytes(UART_NUM_2, rx, 5, pdMS_TO_TICKS(await));
                } else if (req.type == COIN_REQ_DISPENSE) {
                    tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_DISPENSE & BIT_CMD_SET);
                    tx[1] = req.dispense_value;
                    write_payload_controller_9(tx, 2);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                }
                uart_flush(UART_NUM_2);
            }

            tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_POLL & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 17, pdMS_TO_TICKS(60));

            if (len == 1) {
                phys_changer.poll_fail_count = 0;

            } else if (len > 1) {
                phys_changer.poll_fail_count = 0;
                write_controller_9(ACK | BIT_MODE_SET);

                if (len == 2 && rx[0] == 0x0B) {
                    phys_changer.state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Changer reset detected");
                } else {
                    uint8_t dep_types[16], dep_routing[16], dep_count = 0;

                    for (uint8_t i = 0; i + 1 < len; i++) {
                        uint8_t ev = rx[i];
                        if ((ev & 0xC0) == 0x40) {
                            uint8_t type = ev & 0x0F;
                            if (dep_count < 16) {
                                dep_types[dep_count]   = type;
                                dep_routing[dep_count] = (ev >> 4) & 0x03;
                                dep_count++;
                            }
                            ESP_LOGI(TAG, "Coin: type=%d routing=%d", type, (ev >> 4) & 0x03);
                            i++;
                        }
                    }

                    if (dep_count > 0) {
                        tx[0] = (ADDR_CHANGER & BIT_ADD_SET) | (CHGR_TUBE_STATUS & BIT_CMD_SET);
                        write_payload_controller_9(tx, 1);
                        size_t tl = uart_read_bytes(UART_NUM_2, rx, 19, pdMS_TO_TICKS(await));
                        if (tl == 19) {
                            write_controller_9(ACK | BIT_MODE_SET);
                            phys_changer.tube_full_status = ((uint16_t)rx[0] << 8) | rx[1];
                            for (uint8_t i = 0; i < 16; i++) phys_changer.tube_counts[i] = rx[2 + i];
                        }
                        for (uint8_t i = 0; i < dep_count; i++) {
                            coin_deposit_evt_t evt = {
                                dep_types[i],
                                phys_changer.tube_counts[dep_types[i]],
                                dep_routing[i],
                            };
                            xQueueSend(coin_to_target_queue, &evt, 0);
                        }
                    }
                }

            } else {
                if (++phys_changer.poll_fail_count >= 10) {
                    phys_changer.state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Changer: poll timeout, resetting");
                }
            }
        }

        uart_flush(UART_NUM_2);

        //------------------------------------------------------------------//
        // 0x30 Bill Validator (plug and play)
        //------------------------------------------------------------------//
        if (phys_validator.state == INACTIVE_STATE) {

            tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_RESET & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));

            if (len == 1) {
                tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_POLL & BIT_CMD_SET);
                write_payload_controller_9(tx, 1);
                len = uart_read_bytes(UART_NUM_2, rx, 2, pdMS_TO_TICKS(await));

                if (len == 2 && rx[0] == 0x06) {
                    write_controller_9(ACK | BIT_MODE_SET);
                    phys_validator.state = DISABLED_STATE;
                    ESP_LOGI(TAG, "Validator: Just Reset");
                }
            }

        } else if (phys_validator.state == DISABLED_STATE) {

            tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_SETUP & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 28, pdMS_TO_TICKS(await));

            if (len == 28) {
                uart_flush_input(UART_NUM_2);
                write_controller_9(ACK | BIT_MODE_SET);

                phys_validator.feature_level         = rx[0];
                phys_validator.country_code          = ((uint16_t)rx[1] << 8) | rx[2];
                phys_validator.scale_factor          = ((uint16_t)rx[3] << 8) | rx[4];
                phys_validator.decimal_places        = rx[5];
                phys_validator.bill_stacker_capacity = ((uint16_t)rx[6] << 8) | rx[7];
                for (uint8_t i = 0; i < 16; i++) phys_validator.bill_credit[i] = rx[11 + i];

                ESP_LOGI(TAG, "Validator Setup: feature=%d scale=%d dec=%d capacity=%d",
                         phys_validator.feature_level, phys_validator.scale_factor,
                         phys_validator.decimal_places, phys_validator.bill_stacker_capacity);

                tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_BILL_TYPE & BIT_CMD_SET);
                tx[1] = 0xFF; tx[2] = 0xFF; tx[3] = 0xFF; tx[4] = 0xFF;
                write_payload_controller_9(tx, 5);
                len = uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));

                if (len == 1) {
                    phys_validator.state = ENABLED_STATE;
                    ESP_LOGI(TAG, "Validator Enabled");

                    tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_STACKER & BIT_CMD_SET);
                    write_payload_controller_9(tx, 1);
                    size_t sl = uart_read_bytes(UART_NUM_2, rx, 3, pdMS_TO_TICKS(await));
                    if (sl == 3) {
                        write_controller_9(ACK | BIT_MODE_SET);
                        uint16_t val = ((uint16_t)rx[0] << 8) | rx[1];
                        phys_validator.stacker_full  = (val & 0x8000) != 0;
                        phys_validator.stacker_count = val & 0x7FFF;
                    }
                }
            }

        } else {

            bill_from_target_evt_t req;
            if (xQueueReceive(bill_from_target_queue, &req, 0)) {
                if (req.type == BILL_REQ_TYPE_ENABLE) {
                    tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_BILL_TYPE & BIT_CMD_SET);
                    tx[1] = req.bill_enable >> 8;        tx[2] = req.bill_enable;
                    tx[3] = req.bill_escrow_enable >> 8; tx[4] = req.bill_escrow_enable;
                    write_payload_controller_9(tx, 5);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                } else if (req.type == BILL_REQ_ESCROW) {
                    tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_ESCROW & BIT_CMD_SET);
                    tx[1] = req.escrow_command;
                    write_payload_controller_9(tx, 2);
                    uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                }
                uart_flush(UART_NUM_2);
            }

            tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_POLL & BIT_CMD_SET);
            write_payload_controller_9(tx, 1);
            len = uart_read_bytes(UART_NUM_2, rx, 17, pdMS_TO_TICKS(60));

            if (len == 1) {
                phys_validator.poll_fail_count = 0;

            } else if (len > 1) {
                phys_validator.poll_fail_count = 0;
                write_controller_9(ACK | BIT_MODE_SET);

                if (len == 2 && rx[0] == 0x06) {
                    phys_validator.state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Validator reset detected");
                } else {
                    bool bill_stacked = false;
                    for (uint8_t i = 0; i + 1 < len; i++) {
                        uint8_t ev = rx[i];
                        if ((ev & 0x80) && !(ev & 0x40)) {
                            bill_stacked = true;
                            bill_stack_evt_t evt = { ev & 0x0F };
                            xQueueSend(bill_to_target_queue, &evt, 0);
                            ESP_LOGI(TAG, "Bill stacked: type=%d", ev & 0x0F);
                        } else if ((ev & 0x90) == 0x90) {
                            tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_ESCROW & BIT_CMD_SET);
                            tx[1] = 0x01;
                            write_payload_controller_9(tx, 2);
                            uart_read_bytes(UART_NUM_2, rx, 1, pdMS_TO_TICKS(await));
                        }
                    }

                    if (bill_stacked) {
                        tx[0] = (ADDR_VALIDATOR & BIT_ADD_SET) | (VLD_STACKER & BIT_CMD_SET);
                        write_payload_controller_9(tx, 1);
                        size_t sl = uart_read_bytes(UART_NUM_2, rx, 3, pdMS_TO_TICKS(await));
                        if (sl == 3) {
                            write_controller_9(ACK | BIT_MODE_SET);
                            uint16_t val = ((uint16_t)rx[0] << 8) | rx[1];
                            phys_validator.stacker_full  = (val & 0x8000) != 0;
                            phys_validator.stacker_count = val & 0x7FFF;
                        }
                    }
                }

            } else {
                if (++phys_validator.poll_fail_count >= 10) {
                    phys_validator.state = INACTIVE_STATE;
                    ESP_LOGW(TAG, "Validator: poll timeout, resetting");
                }
            }
        }
    }
}

//------------------------------------------------------------------------//
// mdb_target_task - emulates cashless (0x10), coin changer (0x08) and bill
// validator (0x30) toward the VMC. All three devices are always online;
// the VMC drives the init sequence (RESET → SETUP → POLL → READER_ENABLE).
// Physical device config is mirrored when available; defaults otherwise.
//------------------------------------------------------------------------//

#define COIN_POLL_QSIZE 32
#define BILL_POLL_QSIZE 16

void mdb_target_task(void *pvParameters) {
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_TARGET_RX, mdb_rx_falling_isr, NULL);

    uint8_t payload[36];

    // ---- Cashless (0x10) state ----
    device_state_t cshl_state       = INACTIVE_STATE;
    bool     cshl_reset_todo        = false;
    int64_t  cshl_session_start_us  = 0;
    uint16_t cshl_item_price        = 0;
    uint16_t cshl_item_number       = 0;

    // ---- Coin changer (0x08) state ----
    bool     coin_reset_todo        = false;
    uint16_t coin_enable            = 0x0000;
    uint16_t coin_dispense_enable   = 0x0000;
    uint8_t  coin_poll_q[COIN_POLL_QSIZE];
    uint8_t  coin_poll_q_head = 0, coin_poll_q_tail = 0;

    // ---- Bill validator (0x30) state ----
    bool     bill_reset_todo        = false;
    uint16_t bill_enable            = 0x0000;
    uint16_t bill_escrow_enable     = 0x0000;
    uint8_t  bill_poll_q[BILL_POLL_QSIZE];
    uint8_t  bill_poll_q_head = 0, bill_poll_q_tail = 0;

    for (;;) {
        uint8_t  checksum     = 0x00;
        uint8_t  available_tx = 0;

        uint16_t incoming = read_9(&checksum);

        if (!(incoming & BIT_MODE_SET)) continue;
        if ((uint8_t)incoming == ACK)   continue;
        if ((uint8_t)incoming == RET)   continue;
        if ((uint8_t)incoming == NAK)   continue;

        uint8_t addr = incoming & BIT_ADD_SET;
        uint8_t cmd  = incoming & BIT_CMD_SET;

        //================================================================//
        // 0x10 Cashless (emulated, always online)
        //================================================================//
        if (addr == ADDR_CASHLESS) {

            switch (cmd) {
            case CSHL_RESET: {
                if (read_9(NULL) != checksum) continue;
                cshl_reset_todo = true;
                cshl_state = INACTIVE_STATE;
                ESP_LOGI(TAG, "Target Cashless: RESET");
                break; // ACK
            }
            case CSHL_SETUP: {
                switch (read_9(&checksum)) {
                case 0x00: { // Config Data
                    read_9(&checksum); // vmc_feature_level
                    read_9(&checksum); // columns
                    read_9(&checksum); // rows
                    read_9(&checksum); // display_info
                    if (read_9(NULL) != checksum) continue;

                    bool phys = (phys_cashless.state != INACTIVE_STATE);
                    payload[0] = 0x01; // Reader Config
                    payload[1] = 1;    // Feature Level
                    payload[2] = 0xFF; payload[3] = 0xFF; // country code
                    payload[4] = phys ? phys_cashless.scale_factor   : CASHLESS_DEFAULT_SCALE_FACTOR;
                    payload[5] = phys ? phys_cashless.decimal_places : CASHLESS_DEFAULT_DECIMAL_PLACES;
                    payload[6] = 3;          // Response Time
                    payload[7] = 0b00001001; // Miscellaneous Options
                    available_tx = 8;
                    cshl_state = DISABLED_STATE;
                    ESP_LOGI(TAG, "Target Cashless: SETUP Config (phys=%d)", phys);
                    break;
                }
                case 0x01: { // Max/Min Prices
                    read_9(&checksum); read_9(&checksum);
                    read_9(&checksum); read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                    ESP_LOGI(TAG, "Target Cashless: SETUP MaxMin");
                    break; // ACK
                }
                }
                break;
            }
            case CSHL_POLL: {
                if (read_9(NULL) != checksum) continue;

                if (cshl_reset_todo) {
                    cshl_reset_todo = false;
                    payload[0]     = 0x00; // Just Reset
                    available_tx   = 1;
                    cshl_state     = DISABLED_STATE;

                } else {
                    cashless_to_target_evt_t evt;
                    if (xQueueReceive(cashless_to_target_queue, &evt, 0)) {
                        switch (evt.type) {
                        case CSHL_EVT_BEGIN_SESSION:
                            cshl_state = IDLE_STATE;
                            cshl_session_start_us = esp_timer_get_time();
                            payload[0] = 0x03;
                            payload[1] = evt.value >> 8;
                            payload[2] = evt.value;
                            available_tx = 3;
                            ESP_LOGI(TAG, "Target Cashless: Begin Session funds=%u", evt.value);
                            break;
                        case CSHL_EVT_VEND_APPROVED:
                            payload[0] = 0x05;
                            payload[1] = evt.value >> 8;
                            payload[2] = evt.value;
                            available_tx = 3;
                            break;
                        case CSHL_EVT_VEND_DENIED:
                            payload[0] = 0x06;
                            available_tx = 1;
                            cshl_state = IDLE_STATE;
                            break;
                        case CSHL_EVT_SESSION_END:
                            payload[0] = 0x07;
                            available_tx = 1;
                            cshl_state = ENABLED_STATE;
                            break;
                        case CSHL_EVT_SESSION_CANCEL:
                            payload[0] = 0x04;
                            available_tx = 1;
                            break;
                        }
                    } else if (cshl_state >= IDLE_STATE) {
                        // Safety timeout: release session if idle for 60 s.
                        if (esp_timer_get_time() - cshl_session_start_us > 60LL * 1000000LL) {
                            payload[0] = 0x04; // Session Cancel Request
                            available_tx = 1;
                            cshl_state = IDLE_STATE;
                            cshl_session_start_us = esp_timer_get_time();
                            ESP_LOGW(TAG, "Target Cashless: session idle timeout");
                        }
                    }
                }
                break;
            }
            case CSHL_VEND: {
                switch (read_9(&checksum)) {
                case 0x00: { // Vend Request
                    cshl_item_price  = ((uint16_t)read_9(&checksum) << 8) | read_9(&checksum);
                    cshl_item_number = ((uint16_t)read_9(&checksum) << 8) | read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                    cshl_state = VEND_STATE;
                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_REQUEST, cshl_item_price, cshl_item_number };
                    xQueueSend(cashless_from_target_queue, &out, 0);
                    ESP_LOGI(TAG, "Target Cashless: Vend Request price=%u item=%u", cshl_item_price, cshl_item_number);
                    break; // ACK
                }
                case 0x01: { // Vend Cancel
                    if (read_9(NULL) != checksum) continue;
                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_CANCEL, 0, 0 };
                    xQueueSend(cashless_from_target_queue, &out, 0);
                    break; // ACK
                }
                case 0x02: { // Vend Success
                    cshl_item_number = ((uint16_t)read_9(&checksum) << 8) | read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                    cshl_state = IDLE_STATE;
                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_SUCCESS, cshl_item_price, cshl_item_number };
                    xQueueSend(cashless_from_target_queue, &out, 0);
                    ESP_LOGI(TAG, "Target Cashless: Vend Success item=%u", cshl_item_number);
                    break; // ACK
                }
                case 0x03: { // Vend Failure
                    if (read_9(NULL) != checksum) continue;
                    cshl_state = IDLE_STATE;
                    cashless_from_target_evt_t out = { CSHL_REQ_VEND_FAILURE, cshl_item_price, cshl_item_number };
                    xQueueSend(cashless_from_target_queue, &out, 0);
                    break; // ACK
                }
                case 0x04: { // Session Complete
                    if (read_9(NULL) != checksum) continue;
                    cashless_from_target_evt_t out = { CSHL_REQ_SESSION_COMPLETE, 0, 0 };
                    xQueueSend(cashless_from_target_queue, &out, 0);
                    ESP_LOGI(TAG, "Target Cashless: Session Complete");
                    break; // ACK
                }
                case 0x05: { // Cash Sale (informational)
                    read_9(&checksum); read_9(&checksum);
                    read_9(&checksum); read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                    break; // ACK
                }
                }
                break;
            }
            case CSHL_READER: {
                switch (read_9(&checksum)) {
                case 0x00: // Reader Disable
                    if (read_9(NULL) != checksum) continue;
                    cshl_state = DISABLED_STATE;
                    break; // ACK
                case 0x01: // Reader Enable
                    if (read_9(NULL) != checksum) continue;
                    cshl_state = ENABLED_STATE;
                    break; // ACK
                case 0x02: // Reader Cancel
                    if (read_9(NULL) != checksum) continue;
                    payload[0] = 0x08; // Cancelled
                    available_tx = 1;
                    break;
                }
                break;
            }
            case CSHL_EXPANSION: {
                uint8_t sub = (uint8_t)read_9(&checksum);
                if (sub == 0x00) { // Request ID
                    for (uint8_t x = 0; x < 29; x++) read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                    payload[0] = 0x09; // Peripheral ID
                    memcpy(&payload[1], "VMF", 3);
                    memset(&payload[4], ' ', 12);
                    memset(&payload[16], ' ', 12);
                    payload[28] = 0x00; payload[29] = 0x03;
                    available_tx = 30;
                }
                break;
            }
            default:
                continue;
            }

        //================================================================//
        // 0x08 Coin Changer (emulated, always online)
        //================================================================//
        } else if (addr == ADDR_CHANGER) {

            switch (cmd) {
            case CHGR_RESET: {
                if (read_9(NULL) != checksum) continue;
                coin_reset_todo      = true;
                coin_enable          = 0x0000;
                coin_dispense_enable = 0x0000;
                ESP_LOGI(TAG, "Target Changer: RESET");
                break;
            }
            case CHGR_SETUP: {
                if (read_9(NULL) != checksum) continue;
                bool phys = (phys_changer.state != INACTIVE_STATE);
                payload[0] = phys ? phys_changer.feature_level : CHANGER_DEFAULT_FEATURE_LEVEL;
                payload[1] = 0xFF; payload[2] = 0xFF;
                payload[3] = phys ? phys_changer.scale_factor : 1;
                payload[4] = phys ? phys_changer.decimal_places : 2;
                payload[5] = phys ? (phys_changer.coin_type_routing >> 8) : (CHANGER_DEFAULT_COIN_ROUTING >> 8);
                payload[6] = phys ? (phys_changer.coin_type_routing & 0xFF) : (CHANGER_DEFAULT_COIN_ROUTING & 0xFF);
                for (int i = 0; i < 16; i++)
                    payload[7 + i] = phys ? phys_changer.coin_credit[i] : CHANGER_DEFAULT_COIN_CREDIT[i];
                available_tx = 23;
                ESP_LOGI(TAG, "Target Changer: SETUP (phys=%d)", phys);
                break;
            }
            case CHGR_TUBE_STATUS: {
                if (read_9(NULL) != checksum) continue;
                bool phys = (phys_changer.state != INACTIVE_STATE);
                payload[0] = phys ? (phys_changer.tube_full_status >> 8) : 0;
                payload[1] = phys ? (phys_changer.tube_full_status & 0xFF) : 0;
                for (int i = 0; i < 16; i++)
                    payload[2 + i] = phys ? phys_changer.tube_counts[i] : 0;
                available_tx = 18;
                break;
            }
            case CHGR_POLL: {
                if (read_9(NULL) != checksum) continue;
                coin_deposit_evt_t dep;
                while (xQueueReceive(coin_to_target_queue, &dep, 0)) {
                    uint8_t next = (coin_poll_q_head + 2) % COIN_POLL_QSIZE;
                    if (next != coin_poll_q_tail) {
                        coin_poll_q[coin_poll_q_head] = 0x40 | (dep.routing << 4) | dep.coin_type;
                        coin_poll_q_head = (coin_poll_q_head + 1) % COIN_POLL_QSIZE;
                        coin_poll_q[coin_poll_q_head] = dep.tube_count;
                        coin_poll_q_head = (coin_poll_q_head + 1) % COIN_POLL_QSIZE;
                    }
                }
                if (coin_reset_todo) {
                    coin_reset_todo = false;
                    payload[0]     = 0x0B;
                    available_tx   = 1;
                } else {
                    while (coin_poll_q_head != coin_poll_q_tail && available_tx < 16) {
                        payload[available_tx++] = coin_poll_q[coin_poll_q_tail];
                        coin_poll_q_tail = (coin_poll_q_tail + 1) % COIN_POLL_QSIZE;
                    }
                }
                break;
            }
            case CHGR_COIN_TYPE: {
                uint8_t en_h = (uint8_t)read_9(&checksum), en_l = (uint8_t)read_9(&checksum);
                uint8_t di_h = (uint8_t)read_9(&checksum), di_l = (uint8_t)read_9(&checksum);
                if (read_9(NULL) != checksum) continue;
                coin_enable          = ((uint16_t)en_h << 8) | en_l;
                coin_dispense_enable = ((uint16_t)di_h << 8) | di_l;
                coin_from_target_evt_t req = { COIN_REQ_TYPE_ENABLE, coin_enable, coin_dispense_enable, 0 };
                xQueueSend(coin_from_target_queue, &req, 0);
                ESP_LOGI(TAG, "Target Changer: COIN_TYPE enable=0x%04X", coin_enable);
                break;
            }
            case CHGR_DISPENSE: {
                uint8_t val = (uint8_t)read_9(&checksum);
                if (read_9(NULL) != checksum) continue;
                coin_from_target_evt_t req = { COIN_REQ_DISPENSE, 0, 0, val };
                xQueueSend(coin_from_target_queue, &req, 0);
                break;
            }
            case CHGR_EXPANSION: {
                uint8_t sub = (uint8_t)read_9(&checksum);
                if (sub == 0x00) {
                    if (read_9(NULL) != checksum) continue;
                    memcpy(&payload[0], "MDB", 3); memset(&payload[3], ' ', 12);
                    memset(&payload[15], ' ', 12);
                    payload[27] = 0x00; payload[28] = 0x01;
                    payload[29] = payload[30] = payload[31] = payload[32] = 0x00;
                    available_tx = 33;
                } else if (sub == 0x01) {
                    read_9(&checksum); read_9(&checksum);
                    read_9(&checksum); read_9(&checksum);
                    if (read_9(NULL) != checksum) continue;
                }
                break;
            }
            default: continue;
            }

        //================================================================//
        // 0x30 Bill Validator (emulated, always online)
        //================================================================//
        } else if (addr == ADDR_VALIDATOR) {

            switch (cmd) {
            case VLD_RESET: {
                if (read_9(NULL) != checksum) continue;
                bill_reset_todo    = true;
                bill_enable        = 0x0000;
                bill_escrow_enable = 0x0000;
                ESP_LOGI(TAG, "Target Validator: RESET");
                break;
            }
            case VLD_SETUP: {
                if (read_9(NULL) != checksum) continue;
                bool phys = (phys_validator.state != INACTIVE_STATE);
                payload[0]  = phys ? phys_validator.feature_level : 0x01;
                payload[1]  = 0xFF; payload[2] = 0xFF;
                payload[3]  = phys ? (phys_validator.scale_factor >> 8)  : (VALIDATOR_DEFAULT_SCALE_FACTOR >> 8);
                payload[4]  = phys ? (phys_validator.scale_factor & 0xFF) : (VALIDATOR_DEFAULT_SCALE_FACTOR & 0xFF);
                payload[5]  = phys ? phys_validator.decimal_places : 2;
                payload[6]  = phys ? (phys_validator.bill_stacker_capacity >> 8)  : 0;
                payload[7]  = phys ? (phys_validator.bill_stacker_capacity & 0xFF) : 0;
                payload[8]  = 0x00; payload[9] = 0x00; payload[10] = 0x00;
                for (int i = 0; i < 16; i++)
                    payload[11 + i] = phys ? phys_validator.bill_credit[i] : VALIDATOR_DEFAULT_BILL_CREDIT[i];
                available_tx = 27;
                ESP_LOGI(TAG, "Target Validator: SETUP (phys=%d)", phys);
                break;
            }
            case VLD_SECURITY: {
                read_9(&checksum); read_9(&checksum);
                if (read_9(NULL) != checksum) continue;
                break;
            }
            case VLD_POLL: {
                if (read_9(NULL) != checksum) continue;
                bill_stack_evt_t stk;
                while (xQueueReceive(bill_to_target_queue, &stk, 0)) {
                    uint8_t next = (bill_poll_q_head + 1) % BILL_POLL_QSIZE;
                    if (next != bill_poll_q_tail) {
                        bill_poll_q[bill_poll_q_head] = 0x80 | stk.bill_type;
                        bill_poll_q_head = next;
                    }
                }
                if (bill_reset_todo) {
                    bill_reset_todo = false;
                    payload[0]     = 0x06;
                    available_tx   = 1;
                } else {
                    while (bill_poll_q_head != bill_poll_q_tail && available_tx < 16) {
                        payload[available_tx++] = bill_poll_q[bill_poll_q_tail];
                        bill_poll_q_tail = (bill_poll_q_tail + 1) % BILL_POLL_QSIZE;
                    }
                }
                break;
            }
            case VLD_BILL_TYPE: {
                uint8_t en_h  = (uint8_t)read_9(&checksum), en_l  = (uint8_t)read_9(&checksum);
                uint8_t esc_h = (uint8_t)read_9(&checksum), esc_l = (uint8_t)read_9(&checksum);
                if (read_9(NULL) != checksum) continue;
                bill_enable        = ((uint16_t)en_h  << 8) | en_l;
                bill_escrow_enable = ((uint16_t)esc_h << 8) | esc_l;
                bill_from_target_evt_t req = { BILL_REQ_TYPE_ENABLE, bill_enable, bill_escrow_enable, 0 };
                xQueueSend(bill_from_target_queue, &req, 0);
                ESP_LOGI(TAG, "Target Validator: BILL_TYPE enable=0x%04X", bill_enable);
                break;
            }
            case VLD_ESCROW: {
                uint8_t esc = (uint8_t)read_9(&checksum);
                if (read_9(NULL) != checksum) continue;
                bill_from_target_evt_t req = { BILL_REQ_ESCROW, 0, 0, esc };
                xQueueSend(bill_from_target_queue, &req, 0);
                break;
            }
            case VLD_STACKER: {
                if (read_9(NULL) != checksum) continue;
                bool phys = (phys_validator.state != INACTIVE_STATE);
                uint16_t val = phys
                    ? ((phys_validator.stacker_full ? 0x8000 : 0) | (phys_validator.stacker_count & 0x7FFF))
                    : 0;
                payload[0] = val >> 8; payload[1] = val & 0xFF;
                available_tx = 2;
                break;
            }
            case VLD_EXPANSION: {
                uint8_t sub = (uint8_t)read_9(&checksum);
                if (sub == 0x00) {
                    if (read_9(NULL) != checksum) continue;
                    memcpy(&payload[0], "VMF", 3); memset(&payload[3], ' ', 12);
                    memset(&payload[15], ' ', 12);
                    payload[27] = 0x00; payload[28] = 0x01;
                    available_tx = 29;
                }
                break;
            }
            default: continue;
            }

        } else {
            continue;
        }

        write_payload_9(payload, available_tx);
    }
}

void app_main(void) {
    cashless_to_target_queue   = xQueueCreate(4,  sizeof(cashless_to_target_evt_t));
    cashless_from_target_queue = xQueueCreate(4,  sizeof(cashless_from_target_evt_t));
    coin_to_target_queue       = xQueueCreate(16, sizeof(coin_deposit_evt_t));
    coin_from_target_queue     = xQueueCreate(4,  sizeof(coin_from_target_evt_t));
    bill_to_target_queue       = xQueueCreate(16, sizeof(bill_stack_evt_t));
    bill_from_target_queue     = xQueueCreate(4,  sizeof(bill_from_target_evt_t));
    mdb_rx_queue               = xQueueCreate(64, sizeof(uint16_t));

    // Controller port: UART2, 9600 baud, parity-emulated 9th (mode) bit.
    uart_config_t uart_cfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_2, &uart_cfg);
    uart_set_pin(UART_NUM_2, PIN_CONTROLLER_TX, PIN_CONTROLLER_RX, -1, -1);
    uart_driver_install(UART_NUM_2, 256, 256, 0, NULL, 0);

    // Target port: bit-banged GPIO.
    gpio_config_t rx_cfg = {
        .pin_bit_mask = 1ULL << PIN_TARGET_RX,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&rx_cfg);

    gpio_config_t tx_cfg = {
        .pin_bit_mask = 1ULL << PIN_TARGET_TX,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&tx_cfg);
    gpio_set_level(PIN_TARGET_TX, 1);

    // Target task on core 1: bit-bang ISR timing must be isolated from core-0
    // interrupts. Controller task on core 0: uses only UART hardware.
    xTaskCreatePinnedToCore(mdb_controller_task, "controller_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mdb_target_task,     "target_task",     4096, NULL, 1, NULL, 1);
}
