/**
 * @file ros_node.cpp
 * @brief micro-ROS 节点实现 —— 运行在 ESP32 上的 ASV 控制器
 */

#include "application/ros_node.hpp"

#include <stdio.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

#include <rmw_microros/rmw_microros.h>

#include "communication/esp32_serial_transport.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <asv_interfaces/msg/asv_wrench.h>
#include <asv_interfaces/msg/control_input.h>
#include <asv_interfaces/msg/debug.h>

#include "application/asv_control_app.hpp"

/*
 * 1：最小节点测试
 * 0：完整节点
 */
#ifndef UROS_MINIMAL_NODE_TEST
#define UROS_MINIMAL_NODE_TEST 0
#endif

/* 是否在创建 support 之前持续 ping micro ros agent*/
#ifndef UROS_WAIT_FOR_AGENT
#define UROS_WAIT_FOR_AGENT 1
#endif

static const char *UROS_TAG = "MICRO_ROS";

static void print_transport_stats(const char *stage)
{
    esp32_serial_debug_stats_t stats = {};
    esp32_serial_get_debug_stats(&stats);

    ESP_LOGI(
        UROS_TAG,
        "[%s] UART open=%lu close=%lu read_calls=%lu write_calls=%lu "
        "rx_bytes=%lu tx_bytes=%lu last_read=%ld last_write=%ld "
        "open_stage=%ld esp_err=0x%lx preinstalled=%lu owned=%u",
        stage,
        (unsigned long)stats.open_calls,
        (unsigned long)stats.close_calls,
        (unsigned long)stats.read_calls,
        (unsigned long)stats.write_calls,
        (unsigned long)stats.rx_bytes,
        (unsigned long)stats.tx_bytes,
        (long)stats.last_read_result,
        (long)stats.last_write_result,
        (long)stats.last_open_stage,
        (unsigned long)stats.last_esp_error,
        (unsigned long)stats.driver_preinstalled,
        stats.driver_owned ? 1U : 0U
    );
}

static void print_runtime_state(const char *stage)
{
    ESP_LOGI(
        UROS_TAG,
        "[%s] free_heap=%lu minimum_free_heap=%lu stack_high_water=%lu",
        stage,
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
        (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
        (unsigned long)uxTaskGetStackHighWaterMark(NULL)
    );
}

static void stop_after_rcl_error(const char *stage, rcl_ret_t rc)
{
    const rcl_error_string_t error = rcl_get_error_string();

    ESP_LOGE(
        UROS_TAG,
        "FAILED: %s, rc=%d, rcl_error=%s",
        stage,
        (int)rc,
        error.str[0] != '\0' ? error.str : "<empty>"
    );

    print_transport_stats("failure");
    print_runtime_state("failure");
    rcl_reset_error();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define RCCHECK_STAGE(stage, expression)                                      \
    do {                                                                      \
        ESP_LOGI(UROS_TAG, "BEGIN: %s", stage);                               \
        print_runtime_state(stage);                                           \
        const rcl_ret_t stage_rc = (expression);                              \
        if (stage_rc != RCL_RET_OK) {                                         \
            stop_after_rcl_error(stage, stage_rc);                            \
        }                                                                     \
        ESP_LOGI(UROS_TAG, "SUCCESS: %s", stage);                             \
        print_transport_stats(stage);                                         \
    } while (0)

#define RCSOFTCHECK(expression)                                               \
    do {                                                                      \
        const rcl_ret_t soft_rc = (expression);                               \
        if (soft_rc != RCL_RET_OK) {                                          \
            const rcl_error_string_t error = rcl_get_error_string();          \
            ESP_LOGW(                                                         \
                UROS_TAG,                                                     \
                "nonfatal ROS error at line %d: rc=%d, error=%s",             \
                __LINE__,                                                     \
                (int)soft_rc,                                                 \
                error.str[0] != '\0' ? error.str : "<empty>"                  \
            );                                                                \
            rcl_reset_error();                                                \
        }                                                                     \
    } while (0)

/**
 * ROS2 内存分配器
 */
static rcl_allocator_t allocator;

/**
 * rclc 支持对象
 *
 * rclc_support_t 保存 micro-ROS 客户端运行所需的基础环境，包括：
 *
 *   - rcl_context_t:
 *       ROS 2 client library 的运行上下文，
 *       管理节点、通信中间件等底层状态。
 *
 *   - rcl_allocator_t:
 *       ROS 2 使用的内存分配器。
 *
 *   - rcl_clock_t:
 *       ROS 2 时间源，用于时间戳和定时操作。
 *
 * 在创建 node、publisher、subscriber 等 ROS 实体之前，
 * 必须先初始化 rclc_support_t。
 */
static rclc_support_t support;

/**
 * ROS2 节点
 */
static rcl_node_t node;

/**
 * ROS2 Executor
 *
 * rclc_executor 用于管理 ROS 实体的事件调度。
 * Executor 内部维护一个 handle 数组，每个 handle 对应一个
 * 需要处理的事件源，例如 subscription、timer、service 等。
 *
 * 执行流程：
 *
 *   1. 将所有注册的 handle 加入 ROS wait set；
 *   2. 调用 rcl_wait() 等待任意事件源 ready；
 *   3. 检查 ready 的 handle；
 *   4. 调用对应的 callback；
 *
 * rclc_executor_spin() 会持续执行上述流程；
 * rclc_executor_spin_some() 只执行一次后返回。
 *
 * 本节点注册：
 *
 *   - 1 个 subscription:
 *       control_input_sub
 *
 *   - 1 个 timer:
 *       control_timer
 *
 *   - parameter server:
 *       RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES 个内部句柄
 *
 * 因此 executor 至少需要 7 个 handle 槽位。
 */
static rclc_executor_t executor;

/**
 * ROS2 参数服务
 *
 * 运行在节点内部，对外暴露参数读写接口。
 *
 * 参数变更时触发 on_parameter_changed() 回调，可以进行验证或拒绝更改。
 */
static rclc_parameter_server_t param_server;

// =========================================================================
// ROS2 实体：订阅者、发布者、定时器
// =========================================================================

/**
 * 订阅者
 *
 * 订阅者监听指定话题，当有新消息到达时，executor 调用注册的回调函数
 */
static rcl_subscription_t control_input_sub; // 订阅话题 "/control/control_input"

/**
 * 发布者
 *
 * 发布者向指定话题发送消息。每次调用 rcl_publish() 就发送一条消息
 */
static rcl_publisher_t wrench_pub;   // 发布力/力矩到  "/control/asv_wrench"
static rcl_publisher_t debug_pub;    // 发布调试数据到 "/control/debug"

/**
 * 定时器
 *
 * 定时器按固定周期触发回调。
 */
static rcl_timer_t control_timer;

// =========================================================================
// 消息实例 —— 预分配的发送/接收缓冲区
// =========================================================================
// 为了减少动态内存分配，消息实例静态分配在 BSS 段
// 每次收到新消息时，ROS2 将数据反序列化到这个结构体中
// 每次发布消息时，将数据序列化并发送

static asv_interfaces__msg__ControlInput control_input_msg;  // 接收缓冲区
static asv_interfaces__msg__ASVWrench wrench_msg;            // 发送缓冲区
static asv_interfaces__msg__Debug debug_msg;                 // 发送缓冲区


/**
 * 订阅者回调函数
 *
 * @param msgin  指向收到的消息数据的 void 指针（由 executor 传入）
 *
 * 工作流程：
 *   1. 将 void* 转换为具体的消息类型指针（static_cast）
 *   2. 将 ROS 消息字段拷贝到"平台无关"的 Plain 结构体
 *      （解耦 ROS 消息格式与控制器内部数据格式）
 *   3. 将控制指令传递给全局应用实例 g_app
 *
 * 为什么用 Plain 结构体？
 *   ROS 消息类型（asv_interfaces__msg__ControlInput）的字段布局由
 *   代码生成器决定，可能与控制器的内部表示不完全一致。使用中间层
 *   (ControlInputPlain) 可以：
 *     - 隔离 ROS 消息格式变化对控制器代码的影响
 *     - 方便在非 ROS 环境下进行单元测试
 */
static void control_input_callback(const void *msgin)
{
    // 【类型转换】executor 传入 void*，需要显式转换为实际消息类型
    const auto *msg = static_cast<const asv_interfaces__msg__ControlInput *>(msgin);

    // 将 ROS 消息字段逐字段拷贝到业务结构体
    // （即使内存布局一致也不应 memcpy，保持消息类型与业务类型解耦）
    ControlInputPlain input;
    input.seq            = msg->seq;
    input.stamp_us       = msg->stamp_us;
    input.desired_x      = msg->desired_x;
    input.desired_y      = msg->desired_y;
    input.surge_velocity = msg->surge_velocity;
    input.yaw_rate       = msg->yaw_rate;
    input.valid          = msg->valid;

    g_app.setControlInput(input);
}

/**
 * 生成控制器参数的默认值
 */
static ControllerParamsPlain make_default_controller_params()
{
    return ControllerParamsPlain{};
}

/**
 * 参数变更回调
 *
 * 单个参数的合法性由 ASVControlApp::validateParams_() 统一校验，
 * 此处只做类型级过滤，不做值域检查，避免边界值重复定义。
 *
 * @param old_param  变更前的参数值（首次设置时为 NULL）
 * @param new_param  变更后的参数值（删除时为 NULL，这里允许删除）
 * @param context    用户自定义上下文（本代码未使用）
 * @return           true 接受变更，false 拒绝变更
 */
static bool on_parameter_changed(const Parameter *old_param,
                                 const Parameter *new_param,
                                 void *context)
{
    (void)old_param;  // 此代码不比较新旧值
    (void)context;    // 未使用用户上下文

    // new_param == NULL表示删除参数，始终允许删除
    if (new_param == NULL) return true;

    // double 参数的值域校验统一由 ASVControlApp::validateParams_() 负责，
    // 回调层只做类型级过滤（int/param_version）
    if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
        return true;
    }

    // RCLC_PARAMETER_INT：整数型参数
    if (new_param->value.type == RCLC_PARAMETER_INT) {
        // param_version 用于版本号管理，必须 ≥ 0
        if (std::strcmp(new_param->name.data, "param_version") == 0) {
            return new_param->value.integer_value >= 0;
        }
    }

    // bool 类型和其他 int 参数总是接受（无需验证）
    return true;
}

// =========================================================================
// 从参数服务器读取所有控制器参数
// =========================================================================

/**
 * 【批量读取参数】—— 从参数服务器读取所有参数并构建配置结构体
 *
 * 设计模式：
 *   1. 先用默认值填充结构体（安全兜底）
 *   2. 逐个参数尝试读取，读取成功则覆盖默认值
 *   3. 读取失败的参数保持默认值（不会导致未定义行为）
 *
 * 这样即使参数服务器中部分参数缺失（比如刚启动时还没收到上位机的配置），
 * 控制器仍能以默认参数安全运行。
 *
 * @param params  [out] 输出参数：读取到的参数值
 */
static void read_params_from_server(ControllerParamsPlain *params)
{
    // 以安全默认值初始化
    *params = make_default_controller_params();

    // 读取 param_version，用于检测参数是否有更新
    int64_t iver = 0;
    rclc_parameter_get_int(&param_server, "param_version", &iver);
    params->version = (uint32_t)iver;

    // 逐个读取 double 型参数
    // rclc_parameter_get_double() 返回 RCL_RET_OK 表示读取成功
    // 存储为 float 以节省内存
    double d;
    if (rclc_parameter_get_double(&param_server, "time_constant", &d) == RCL_RET_OK)
        params->time_constant = (float)d;
    if (rclc_parameter_get_double(&param_server, "v_max", &d) == RCL_RET_OK)
        params->v_max = (float)d;
    if (rclc_parameter_get_double(&param_server, "e_max", &d) == RCL_RET_OK)
        params->e_max = (float)d;
    if (rclc_parameter_get_double(&param_server, "delta_t", &d) == RCL_RET_OK)
        params->delta_t = (float)d;
    if (rclc_parameter_get_double(&param_server, "gamma_rl", &d) == RCL_RET_OK)
        params->gamma_rl = (float)d;
    if (rclc_parameter_get_double(&param_server, "lambda_rls", &d) == RCL_RET_OK)
        params->lambda_rls = (float)d;
    if (rclc_parameter_get_double(&param_server, "max_force", &d) == RCL_RET_OK)
        params->max_force = (float)d;
    if (rclc_parameter_get_double(&param_server, "max_moment", &d) == RCL_RET_OK)
        params->max_moment = (float)d;
    if (rclc_parameter_get_double(&param_server, "i_bound_force", &d) == RCL_RET_OK)
        params->i_bound_force = (float)d;
    if (rclc_parameter_get_double(&param_server, "d_bound_force", &d) == RCL_RET_OK)
        params->d_bound_force = (float)d;
    if (rclc_parameter_get_double(&param_server, "i_bound_moment", &d) == RCL_RET_OK)
        params->i_bound_moment = (float)d;
    if (rclc_parameter_get_double(&param_server, "d_bound_moment", &d) == RCL_RET_OK)
        params->d_bound_moment = (float)d;
}

/**
 * 定时器回调
 *
 * @param timer          触发此回调的定时器对象
 * @param last_call_time 上次调用时的系统时间（纳秒），本代码未使用
 */
static void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;  // 未使用时间戳参数

    if (timer == NULL) {
        return;
    }

    // 步骤1：先应用完整的新参数集
    static uint32_t applied_version = 0;
    int64_t param_version = 0;
    rclc_parameter_get_int(&param_server, "param_version", &param_version);

    if ((uint32_t)param_version != applied_version) {
        ControllerParamsPlain params;
        read_params_from_server(&params);

        if (g_app.applyParams(params, false)) {
            applied_version = (uint32_t)param_version;  // 记录已应用版本
        }
    }

    // 步骤2：在新参数生效后复位控制器
    bool reset_ctrl = false;
    rclc_parameter_get_bool(&param_server, "reset_controller", &reset_ctrl);
    if (reset_ctrl) {
        g_app.resetController();
        rclc_parameter_set_bool(&param_server, "reset_controller", false);
    }

    // 步骤3：运行控制算法
    g_app.step();

    // 步骤4：发布推力/力矩消息
    const WrenchPlain wrench = g_app.getWrench();
    wrench_msg.seq      = wrench.seq;
    wrench_msg.stamp_us = wrench.stamp_us;
    wrench_msg.force    = wrench.force;
    wrench_msg.moment   = wrench.moment;
    wrench_msg.valid    = wrench.valid;
    RCSOFTCHECK(rcl_publish(&wrench_pub, &wrench_msg, NULL));

    // 步骤5：发布调试数据
    const DebugPlain debug = g_app.getDebug();
    debug_msg.seq        = debug.seq;
    debug_msg.stamp_us   = debug.stamp_us;
    debug_msg.x_hat      = debug.x_hat;
    debug_msg.y_hat      = debug.y_hat;
    debug_msg.v_hat_x    = debug.v_hat_x;
    debug_msg.v_hat_y    = debug.v_hat_y;
    debug_msg.v_hat      = debug.v_hat;
    debug_msg.theta      = debug.theta;
    debug_msg.e_track_x  = debug.e_track_x;
    debug_msg.e_track_y  = debug.e_track_y;
    debug_msg.a          = debug.a;
    debug_msg.varepsilon = debug.varepsilon;
    debug_msg.delta_v    = debug.delta_v;
    debug_msg.delta_psi  = debug.delta_psi;
    debug_msg.p_f        = debug.p_f;
    debug_msg.i_f        = debug.i_f;
    debug_msg.d_f        = debug.d_f;
    debug_msg.p_m        = debug.p_m;
    debug_msg.i_m        = debug.i_m;
    debug_msg.d_m        = debug.d_m;
    RCSOFTCHECK(rcl_publish(&debug_pub, &debug_msg, NULL));
}

/**
 * 向参数服务器注册参数
 */
static rcl_ret_t declare_controller_params()
{
#define RETURN_IF_RCL_ERROR(expression)                                       \
    do {                                                                      \
        const rcl_ret_t rc = (expression);                                    \
        if (rc != RCL_RET_OK) {                                               \
            return rc;                                                        \
        }                                                                     \
    } while (0)

    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "time_constant", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "v_max", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "e_max", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "delta_t", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "gamma_rl", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "lambda_rls", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "max_force", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "max_moment", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "i_bound_force", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "d_bound_force", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "i_bound_moment", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "d_bound_moment", RCLC_PARAMETER_DOUBLE));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "param_version", RCLC_PARAMETER_INT));
    RETURN_IF_RCL_ERROR(rclc_add_parameter(&param_server, "reset_controller", RCLC_PARAMETER_BOOL));

#undef RETURN_IF_RCL_ERROR
    return RCL_RET_OK;
}

/**
 * 设置参数默认值。返回第一个失败的 rcl 返回码，便于 RCCHECK_STAGE 精确定位。
 */
static rcl_ret_t set_default_controller_params()
{
#define RETURN_IF_RCL_ERROR(expression)                                       \
    do {                                                                       \
        const rcl_ret_t rc = (expression);                                     \
        if (rc != RCL_RET_OK) {                                                \
            return rc;                                                         \
        }                                                                      \
    } while (0)

    const ControllerParamsPlain d = make_default_controller_params();

    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "time_constant", d.time_constant));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "v_max", d.v_max));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "e_max", d.e_max));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "delta_t", d.delta_t));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "gamma_rl", d.gamma_rl));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "lambda_rls", d.lambda_rls));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "max_force", d.max_force));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "max_moment", d.max_moment));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "i_bound_force", d.i_bound_force));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "d_bound_force", d.d_bound_force));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "i_bound_moment", d.i_bound_moment));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_double(&param_server, "d_bound_moment", d.d_bound_moment));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_int(&param_server, "param_version", (int64_t)d.version));
    RETURN_IF_RCL_ERROR(rclc_parameter_set_bool(&param_server, "reset_controller", false));

#undef RETURN_IF_RCL_ERROR
    return RCL_RET_OK;
}

// micro-ROS 任务入口 —— 整个节点的启动与运行
extern "C" void esp_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(
        UROS_TAG,
        "esp_ros_task entered; mode=%s",
        UROS_MINIMAL_NODE_TEST ? "MINIMAL_NODE_TEST" : "FULL_NODE"
    );
    print_runtime_state("task start");

#if UROS_WAIT_FOR_AGENT
    uint32_t ping_attempt = 0;

    while (true) {
        ping_attempt++;
        ESP_LOGI(UROS_TAG, "ping Agent, attempt=%lu", (unsigned long)ping_attempt);

        const rmw_ret_t ping_rc = rmw_uros_ping_agent(1000, 1);
        print_transport_stats("after ping");

        if (ping_rc == RMW_RET_OK) {
            ESP_LOGI(UROS_TAG, "Agent ping SUCCESS");
            break;
        }

        ESP_LOGW(UROS_TAG, "Agent ping FAILED: rc=%d", (int)ping_rc);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    allocator = rcl_get_default_allocator();

    RCCHECK_STAGE(
        "rclc_support_init",
        rclc_support_init(&support, 0, NULL, &allocator)
    );

    RCCHECK_STAGE(
        "rclc_node_init_default",
        rclc_node_init_default(
            &node,
            "esp32_node",
            "",
            &support
        )
    );

#if UROS_MINIMAL_NODE_TEST
    /*
     * 最小节点测试
     */
    ESP_LOGI(UROS_TAG, "MINIMAL NODE READY: /esp32_node");
    ESP_LOGI(UROS_TAG, "Now run on Jetson: ros2 node list --no-daemon");

    while (true) {
        print_transport_stats("minimal node heartbeat");
        print_runtime_state("minimal node heartbeat");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
#else
    RCCHECK_STAGE(
        "control_input subscription",
        rclc_subscription_init_default(
            &control_input_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(asv_interfaces, msg, ControlInput),
            "/control/control_input"
        )
    );

    RCCHECK_STAGE(
        "wrench publisher",
        rclc_publisher_init_default(
            &wrench_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(asv_interfaces, msg, ASVWrench),
            "/control/asv_wrench"
        )
    );

    RCCHECK_STAGE(
        "debug publisher",
        rclc_publisher_init_default(
            &debug_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(asv_interfaces, msg, Debug),
            "/control/debug"
        )
    );



    const rclc_parameter_options_t parameter_options = {
    .notify_changed_over_dds = true,
    .max_params = 16,
    .allow_undeclared_parameters = false,
    .low_mem_mode = false,
    };

    RCCHECK_STAGE(
        "parameter server init",
        rclc_parameter_server_init_with_option(
            &param_server,
            &node,
            &parameter_options
        )
    );

    RCCHECK_STAGE(
        "declare controller parameters",
        declare_controller_params()
    );

    RCCHECK_STAGE(
        "set controller parameter defaults",
        set_default_controller_params()
    );

    const unsigned int timer_timeout_ms = 100;
    RCCHECK_STAGE(
        "control timer init",
        rclc_timer_init_default(
            &control_timer,
            &support,
            RCL_MS_TO_NS(timer_timeout_ms),
            control_timer_callback
        )
    );

    const size_t executor_handles =
        RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 2;

    RCCHECK_STAGE(
        "executor init",
        rclc_executor_init(
            &executor,
            &support.context,
            executor_handles,
            &allocator
        )
    );

    RCCHECK_STAGE(
        "executor add control_input subscription",
        rclc_executor_add_subscription(
            &executor,
            &control_input_sub,
            &control_input_msg,
            &control_input_callback,
            ON_NEW_DATA
        )
    );

    RCCHECK_STAGE(
        "executor add parameter server",
        rclc_executor_add_parameter_server(
            &executor,
            &param_server,
            on_parameter_changed
        )
    );

    RCCHECK_STAGE(
        "executor add control timer",
        rclc_executor_add_timer(
            &executor,
            &control_timer
        )
    );

    ESP_LOGI(UROS_TAG, "FULL NODE READY: /esp32_node");
    ESP_LOGI(
        UROS_TAG,
        "topics: /control/control_input, /control/asv_wrench, /control/debug"
    );

    uint32_t spin_error_count = 0;
    uint32_t heartbeat_count = 0;

    while (true) {
        const rcl_ret_t spin_rc =
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

        if (spin_rc != RCL_RET_OK && spin_rc != RCL_RET_TIMEOUT) {
            ++spin_error_count;

            /* 限流：第一次和每 100 次错误打印一次，避免断线时刷屏。 */
            if (spin_error_count == 1 || spin_error_count % 100 == 0) {
                const rcl_error_string_t error = rcl_get_error_string();
                ESP_LOGW(
                    UROS_TAG,
                    "executor spin error count=%lu rc=%d error=%s",
                    (unsigned long)spin_error_count,
                    (int)spin_rc,
                    error.str[0] != '\0' ? error.str : "<empty>"
                );
                print_transport_stats("spin error");
                rcl_reset_error();
            }
        }

        heartbeat_count++;
        if (heartbeat_count >= 1000) {
            heartbeat_count = 0;
            print_transport_stats("full node heartbeat");
            print_runtime_state("full node heartbeat");
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
#endif
}
