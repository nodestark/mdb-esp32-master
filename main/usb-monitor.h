#ifndef USB_MONITOR_H
#define USB_MONITOR_H

// Installs the TinyUSB composite device (2x CDC-ACM) on the board's single
// native USB port: CDC0 carries the console (ESP_LOGx/printf), replacing
// what CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG did before; CDC1 is a
// dedicated read-only ASCII line protocol (qibixx MDB-USB-style V/H/status
// commands) for a PC-side terminal/app to monitor live bridge state. No
// command on CDC1 can mutate bridge state - this is a monitor, not a
// second control channel alongside BLE/MQTT.
//
// Call once from app_main(), before WiFi/NVS/BLE init - this subsystem has
// no dependency on either.
void usb_monitor_init(void);

#endif
