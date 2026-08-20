#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

extern "C" {
#include "communication/esp32_serial_transport.h"
}

#include "application/ros_node.hpp"
#include "application/asv_app.hpp"

static const char *TAG = "MAIN";
static uart_port_t uart_port = UART_NUM_2;

extern "C" void app_main(void)
{
    ESP_LOGI(
        TAG,
        "starting: UART%d TX=%d RX=%d RTS=%d CTS=%d",
        (int)uart_port,
        CONFIG_MICROROS_UART_TXD,
        CONFIG_MICROROS_UART_RXD,
        CONFIG_MICROROS_UART_RTS,
        CONFIG_MICROROS_UART_CTS
    );

    g_app.init();
    ESP_LOGI(TAG, "ASVApp initialized");

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
    ESP_LOGI(TAG, "micro-ROS custom serial transport registered");
#else
#error micro-ROS custom serial transport is not enabled
#endif

    const BaseType_t task_result = xTaskCreate(
        esp_ros_task,
        "uros_task",
        8192,
        NULL,
        5,
        NULL
    );

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "failed to create esp_ros_task");
        return;
    }

    ESP_LOGI(TAG, "esp_ros_task created");
}