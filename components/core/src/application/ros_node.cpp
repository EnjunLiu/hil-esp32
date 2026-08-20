/**
 * @file ros_node.cpp
 * @brief micro-ROS 节点 —— ESP32 ASV 固件
 */

#include "application/ros_node.hpp"

#include <stdio.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <interfaces/msg/output.h>
#include <interfaces/msg/input.h>

#include "application/asv_app.hpp"

#ifndef UROS_MINIMAL_NODE_TEST
#define UROS_MINIMAL_NODE_TEST 0
#endif

#ifndef UROS_WAIT_FOR_AGENT
#define UROS_WAIT_FOR_AGENT 1
#endif

static const char *UROS_TAG = "MICRO_ROS";

#define RETURN_IF_RCL_ERROR(expression)                                       \
    do {                                                                      \
        const rcl_ret_t rc = (expression);                                    \
        if (rc != RCL_RET_OK) {                                               \
            return rc;                                                        \
        }                                                                     \
    } while (0)

#define RCSOFTCHECK(expression)                                               \
    do {                                                                      \
        const rcl_ret_t soft_rc = (expression);                               \
        if (soft_rc != RCL_RET_OK) {                                          \
            const rcl_error_string_t error = rcl_get_error_string();          \
            ESP_LOGW(UROS_TAG, "nonfatal ROS error: rc=%d, error=%s",         \
                     (int)soft_rc, error.str[0] != '\0' ? error.str : "<empty>"); \
            rcl_reset_error();                                                \
        }                                                                     \
    } while (0)

struct Param {
    const char *name;
    float Params::*field;
};

static constexpr Param kParams[] = {
    {"time_constant", &Params::time_constant},
    {"v_max", &Params::v_max},
    {"e_max", &Params::e_max},
    {"delta_t", &Params::delta_t},
    {"gamma_rl", &Params::gamma_rl},
    {"lambda_rls", &Params::lambda_rls},
    {"decay_force", &Params::decay_force},
    {"decay_moment", &Params::decay_moment},
    {"w_bound", &Params::w_bound},
    {"pid_gain_max", &Params::pid_gain_max},
    {"pid_gain_min", &Params::pid_gain_min},
    {"max_force", &Params::max_force},
    {"max_moment", &Params::max_moment},
    {"i_bound_force", &Params::i_bound_force},
    {"d_bound_force", &Params::d_bound_force},
    {"i_bound_moment", &Params::i_bound_moment},
    {"d_bound_moment", &Params::d_bound_moment},
    {"input_timeout_s", &Params::input_timeout_s},
};

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_parameter_server_t param_server;

static uint32_t applied_param_version = 0;
static bool agent_connected = true;

static rcl_subscription_t input_sub;
static rcl_publisher_t output_pub;
static rcl_timer_t step_timer;

static interfaces__msg__Input input_msg;
static interfaces__msg__Output output_msg;

static void stop_after_rcl_error(const char *stage, rcl_ret_t rc)
{
    const rcl_error_string_t error = rcl_get_error_string();
    ESP_LOGE(UROS_TAG, "FAILED: %s, rc=%d, rcl_error=%s",
             stage, (int)rc, error.str[0] != '\0' ? error.str : "<empty>");
    rcl_reset_error();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define RCCHECK_STAGE(stage, expression)                                      \
    do {                                                                      \
        ESP_LOGI(UROS_TAG, "BEGIN: %s", stage);                               \
        const rcl_ret_t stage_rc = (expression);                              \
        if (stage_rc != RCL_RET_OK) {                                         \
            stop_after_rcl_error(stage, stage_rc);                            \
        }                                                                     \
        ESP_LOGI(UROS_TAG, "SUCCESS: %s", stage);                             \
    } while (0)

static Input toInput(const interfaces__msg__Input &msg)
{
    return {
        msg.stamp_us, msg.desired_x, msg.desired_y,
        msg.surge_velocity, msg.yaw_rate, msg.valid
    };
}

static void fill_output_msg(const Output &output)
{
    output_msg.stamp_us = output.stamp_us;
    output_msg.force = output.force;
    output_msg.moment = output.moment;
    output_msg.valid = output.valid;
}

static void input_callback(const void *msgin)
{
    g_app.setInput(toInput(*static_cast<const interfaces__msg__Input *>(msgin)));
}

static bool on_parameter_changed(const Parameter *,
                                 const Parameter *new_param,
                                 void *)
{
    if (new_param == NULL) {
        return true;
    }
    if (new_param->value.type == RCLC_PARAMETER_INT &&
        std::strcmp(new_param->name.data, "param_version") == 0) {
        return new_param->value.integer_value >= 0;
    }
    return true;
}

static void read_params_from_server(Params *params)
{
    *params = Params{};

    int64_t iver = 0;
    rclc_parameter_get_int(&param_server, "param_version", &iver);
    params->version = (uint32_t)iver;

    double d;
    for (const auto &p : kParams) {
        if (rclc_parameter_get_double(&param_server, p.name, &d) == RCL_RET_OK) {
            params->*(p.field) = (float)d;
        }
    }
}

static void step_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    int64_t param_version = 0;
    rclc_parameter_get_int(&param_server, "param_version", &param_version);

    if ((uint32_t)param_version != applied_param_version) {
        Params params;
        read_params_from_server(&params);
        if (g_app.applyParams(params, false)) {
            applied_param_version = (uint32_t)param_version;
        }
    }

    bool reset_state = false;
    rclc_parameter_get_bool(&param_server, "reset_state", &reset_state);
    if (reset_state) {
        g_app.resetState();
        rclc_parameter_set_bool(&param_server, "reset_state", false);
    }

    fill_output_msg(g_app.step());
    RCSOFTCHECK(rcl_publish(&output_pub, &output_msg, NULL));
}

static rcl_ret_t declare_params()
{
    for (const auto &p : kParams) {
        RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, p.name, RCLC_PARAMETER_DOUBLE));
    }
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "param_version", RCLC_PARAMETER_INT));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "reset_state", RCLC_PARAMETER_BOOL));
    return RCL_RET_OK;
}

static rcl_ret_t set_default_params()
{
    const Params defaults{};
    for (const auto &p : kParams) {
        RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, p.name, defaults.*(p.field)));
    }
    RETURN_IF_RCL_ERROR(rclc_parameter_set_int(&param_server, "param_version", (int64_t)defaults.version));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_bool(&param_server, "reset_state", false));
    return RCL_RET_OK;
}

static void reset_agent_session(const char *reason)
{
    ESP_LOGW(UROS_TAG, "agent session reset: %s", reason);
    applied_param_version = 0;
    g_app.init();
    RCSOFTCHECK(set_default_params());
}

extern "C" void esp_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(UROS_TAG, "esp_ros_task entered; mode=%s",
             UROS_MINIMAL_NODE_TEST ? "MINIMAL_NODE_TEST" : "FULL_NODE");

#if UROS_WAIT_FOR_AGENT
    uint32_t ping_attempt = 0;
    while (true) {
        ping_attempt++;
        ESP_LOGI(UROS_TAG, "ping Agent, attempt=%lu", (unsigned long)ping_attempt);
        const rmw_ret_t ping_rc = rmw_uros_ping_agent(1000, 1);
        if (ping_rc == RMW_RET_OK) {
            ESP_LOGI(UROS_TAG, "Agent ping SUCCESS");
            break;
        }
        ESP_LOGW(UROS_TAG, "Agent ping FAILED: rc=%d", (int)ping_rc);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    allocator = rcl_get_default_allocator();

    RCCHECK_STAGE("rclc_support_init", rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK_STAGE("rclc_node_init_default",
                  rclc_node_init_default(&node, "esp32_node", "", &support));

#if UROS_MINIMAL_NODE_TEST
    ESP_LOGI(UROS_TAG, "MINIMAL NODE READY: /esp32_node");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
#else
    RCCHECK_STAGE("input subscription",
                  rclc_subscription_init_default(
                      &input_sub, &node,
                      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Input),
                      "/control/input"));
    RCCHECK_STAGE("output publisher",
                  rclc_publisher_init_default(
                      &output_pub, &node,
                      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Output),
                      "/control/output"));

    const rclc_parameter_options_t parameter_options = {
        .notify_changed_over_dds = true,
        .max_params = 20,
        .allow_undeclared_parameters = false,
        .low_mem_mode = false,
    };

    RCCHECK_STAGE("parameter server init",
                  rclc_parameter_server_init_with_option(&param_server, &node, &parameter_options));
    RCCHECK_STAGE("declare parameters", declare_params());
    RCCHECK_STAGE("set parameter defaults", set_default_params());

    const unsigned int timer_timeout_ms = 100;
    RCCHECK_STAGE("step timer init",
                  rclc_timer_init_default(
                      &step_timer, &support, RCL_MS_TO_NS(timer_timeout_ms), step_timer_callback));

    const size_t executor_handles = RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 2;
    RCCHECK_STAGE("executor init",
                  rclc_executor_init(&executor, &support.context, executor_handles, &allocator));
    RCCHECK_STAGE("executor add input subscription",
                  rclc_executor_add_subscription(
                      &executor, &input_sub, &input_msg,
                      &input_callback, ON_NEW_DATA));
    RCCHECK_STAGE("executor add parameter server",
                  rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));
    RCCHECK_STAGE("executor add step timer",
                  rclc_executor_add_timer(&executor, &step_timer));

    ESP_LOGI(UROS_TAG, "FULL NODE READY: /esp32_node");
    ESP_LOGI(UROS_TAG, "topics: /control/input, /control/output");

    uint32_t spin_error_count = 0;
    uint32_t ping_ticks = 0;

    while (true) {
        const rcl_ret_t spin_rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

        if (spin_rc != RCL_RET_OK && spin_rc != RCL_RET_TIMEOUT) {
            ++spin_error_count;
            if (spin_error_count == 1 || spin_error_count % 100 == 0) {
                const rcl_error_string_t error = rcl_get_error_string();
                ESP_LOGW(UROS_TAG, "executor spin error count=%lu rc=%d error=%s",
                         (unsigned long)spin_error_count, (int)spin_rc,
                         error.str[0] != '\0' ? error.str : "<empty>");
                rcl_reset_error();
            }
        }

        if (++ping_ticks >= 1000) {
            ping_ticks = 0;
            const bool was_connected = agent_connected;
            agent_connected = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);
            if (was_connected && !agent_connected) {
                reset_agent_session("disconnected");
            } else if (!was_connected && agent_connected) {
                reset_agent_session("reconnected");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
#endif
}
