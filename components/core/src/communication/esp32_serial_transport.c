#include "communication/esp32_serial_transport.h"

#include "sdkconfig.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#define UART_TXD CONFIG_MICROROS_UART_TXD
#define UART_RXD CONFIG_MICROROS_UART_RXD
#define UART_RTS CONFIG_MICROROS_UART_RTS
#define UART_CTS CONFIG_MICROROS_UART_CTS

#define UART_RX_BUFFER_SIZE 1024

static bool g_driver_owned = false;

static uart_port_t get_uart_port(struct uxrCustomTransport *transport)
{
    return *((uart_port_t *)transport->args);
}

bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    if (transport == NULL || transport->args == NULL) {
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

    if (uart_param_config(uart_port, &uart_config) != ESP_OK) {
        return false;
    }

    const int tx_pin = UART_TXD < 0 ? UART_PIN_NO_CHANGE : UART_TXD;
    const int rx_pin = UART_RXD < 0 ? UART_PIN_NO_CHANGE : UART_RXD;
    const int rts_pin = UART_RTS < 0 ? UART_PIN_NO_CHANGE : UART_RTS;
    const int cts_pin = UART_CTS < 0 ? UART_PIN_NO_CHANGE : UART_CTS;

    if (uart_set_pin(uart_port, tx_pin, rx_pin, rts_pin, cts_pin) != ESP_OK) {
        return false;
    }

    if (uart_is_driver_installed(uart_port)) {
        return true;
    }

    if (uart_driver_install(uart_port, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0) != ESP_OK) {
        return false;
    }

    g_driver_owned = true;
    uart_flush_input(uart_port);
    return true;
}

bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    if (transport == NULL || transport->args == NULL) {
        return false;
    }

    const uart_port_t uart_port = get_uart_port(transport);

    /* 不删除其他模块预先安装的 UART 驱动 */
    if (!g_driver_owned) {
        return true;
    }

    if (uart_driver_delete(uart_port) != ESP_OK) {
        return false;
    }

    g_driver_owned = false;
    return true;
}

size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *err)
{
    if (err != NULL) {
        *err = 0;
    }

    if (transport == NULL || transport->args == NULL || buf == NULL) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    const uart_port_t uart_port = get_uart_port(transport);
    const int written = uart_write_bytes(uart_port, buf, len);

    if (written < 0) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

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
    if (err != NULL) {
        *err = 0;
    }

    if (transport == NULL || transport->args == NULL || buf == NULL) {
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

    const int received = uart_read_bytes(uart_port, buf, len, timeout_ticks);

    if (received < 0) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    return (size_t)received;
}
