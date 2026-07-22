#include "communication/esp32_serial_transport.h"

#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define UART_TXD CONFIG_MICROROS_UART_TXD
#define UART_RXD CONFIG_MICROROS_UART_RXD
#define UART_RTS CONFIG_MICROROS_UART_RTS
#define UART_CTS CONFIG_MICROROS_UART_CTS

#define UART_RX_BUFFER_SIZE 1024

static esp32_serial_debug_stats_t g_stats = {0};
static bool g_driver_owned = false;

static uart_port_t get_uart_port(struct uxrCustomTransport *transport)
{
    return *((uart_port_t *)transport->args);
}

void esp32_serial_get_debug_stats(esp32_serial_debug_stats_t *stats)
{
    if (stats != NULL) {
        g_stats.driver_owned = g_driver_owned;
        *stats = g_stats;
    }
}

bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    ++g_stats.open_calls;
    g_stats.last_open_stage = 0;
    g_stats.last_esp_error = ESP_OK;

    if (transport == NULL || transport->args == NULL) {
        g_stats.last_open_stage = 1;
        g_stats.last_esp_error = ESP_ERR_INVALID_ARG;
        return false;
    }

    const uart_port_t uart_port = get_uart_port(transport);

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    esp_err_t ret = uart_param_config(uart_port, &uart_config);
    if (ret != ESP_OK) {
        g_stats.last_open_stage = 2;
        g_stats.last_esp_error = ret;
        return false;
    }

    const int tx_pin = UART_TXD < 0 ? UART_PIN_NO_CHANGE : UART_TXD;
    const int rx_pin = UART_RXD < 0 ? UART_PIN_NO_CHANGE : UART_RXD;
    const int rts_pin = UART_RTS < 0 ? UART_PIN_NO_CHANGE : UART_RTS;
    const int cts_pin = UART_CTS < 0 ? UART_PIN_NO_CHANGE : UART_CTS;

    ret = uart_set_pin(uart_port, tx_pin, rx_pin, rts_pin, cts_pin);
    if (ret != ESP_OK) {
        g_stats.last_open_stage = 3;
        g_stats.last_esp_error = ret;
        return false;
    }

    if (uart_is_driver_installed(uart_port)) {
        ++g_stats.driver_preinstalled;
        g_stats.driver_owned = g_driver_owned;
        return true;
    }

    ret = uart_driver_install(
        uart_port,
        UART_RX_BUFFER_SIZE,
        0,
        0,
        NULL,
        0
    );

    if (ret != ESP_OK) {
        g_stats.last_open_stage = 4;
        g_stats.last_esp_error = ret;
        return false;
    }

    g_driver_owned = true;
    g_stats.driver_owned = true;

    /* 清除驱动安装前可能存在的残留噪声。 */
    uart_flush_input(uart_port);
    return true;
}

bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    ++g_stats.close_calls;

    if (transport == NULL || transport->args == NULL) {
        return false;
    }

    const uart_port_t uart_port = get_uart_port(transport);

    /* 不删除其他模块预先安装的 UART 驱动。 */
    if (!g_driver_owned) {
        return true;
    }

    const esp_err_t ret = uart_driver_delete(uart_port);
    if (ret == ESP_OK) {
        g_driver_owned = false;
        g_stats.driver_owned = false;
        return true;
    }

    g_stats.last_esp_error = ret;
    return false;
}

size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *err)
{
    ++g_stats.write_calls;

    if (err != NULL) {
        *err = 0;
    }

    if (transport == NULL || transport->args == NULL || buf == NULL) {
        g_stats.last_write_result = -1;
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    const uart_port_t uart_port = get_uart_port(transport);
    const int written = uart_write_bytes(uart_port, buf, len);
    g_stats.last_write_result = written;

    if (written < 0) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    g_stats.tx_bytes += (uint32_t)written;

    if ((size_t)written != len && err != NULL) {
        *err = 1;
    }

    return (size_t)written;
}

size_t esp32_serial_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *err)
{
    ++g_stats.read_calls;

    if (err != NULL) {
        *err = 0;
    }

    if (transport == NULL || transport->args == NULL || buf == NULL) {
        g_stats.last_read_result = -1;
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    const uart_port_t uart_port = get_uart_port(transport);

    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout);
    if (timeout > 0 && timeout_ticks == 0) {
        timeout_ticks = 1;
    }

    const int received = uart_read_bytes(
        uart_port,
        buf,
        len,
        timeout_ticks
    );

    g_stats.last_read_result = received;

    if (received < 0) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    if (received > 0) {
        g_stats.rx_bytes += (uint32_t)received;
    }

    return (size_t)received;
}