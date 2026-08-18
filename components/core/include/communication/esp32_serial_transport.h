#ifndef _MICROROS_CLIENT_ESP32_SERIAL_TRANSPORT_H_
#define _MICROROS_CLIENT_ESP32_SERIAL_TRANSPORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32_t open_calls;
    uint32_t close_calls;
    uint32_t read_calls;
    uint32_t write_calls;

    uint32_t rx_bytes;
    uint32_t tx_bytes;

    int32_t last_read_result;
    int32_t last_write_result;

    /* 0=无错误，1=参数，2=uart_param_config，3=uart_set_pin，4=driver_install */
    int32_t last_open_stage;
    int32_t last_esp_error;

    uint32_t driver_preinstalled;
    bool driver_owned;
} esp32_serial_debug_stats_t;

bool esp32_serial_open(struct uxrCustomTransport *transport);
bool esp32_serial_close(struct uxrCustomTransport *transport);

size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *err);

size_t esp32_serial_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *err);

void esp32_serial_get_debug_stats(esp32_serial_debug_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif