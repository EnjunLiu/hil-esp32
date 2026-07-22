/**
 * @file    asv_control_app.hpp
 * @brief   ASV 控制应用层声明
 */

#pragma once

#include "IQmathLib.h"

#include "asv_types.hpp"

#include "estimator/estimator.hpp" // 观测器&导引律
#include "control/RLPID_controller.hpp" // 控制器

class ASVControlApp {
public:
    /**
     * @brief 在静态初始化期间可以安全运行的默认构造函数。
     *
     * 用默认值初始化 params_，然后用 params_ 构造所有子对象
     * 
     *  @note 不要在这里访问硬件或 RTOS 服务
     */
    ASVControlApp();

    /**
     * @brief 在 RTOS 和外设就绪后调用的初始化函数
     *
     * 重置所有 I/O 缓冲区、错误追踪信息和控制器内部状态，并将默认 params_ 重新同步到每个子对象
     *
     * @note 可多次调用，用于复位
     */
    void init();

    /**
     * @brief 将最新输入数据送入控制流水线
     */
    void setControlInput(const ControlInputPlain &input);

    /**
     * @brief 更新控制参数 params_
     *
     * @param params           新控制参数，其 `version` 必须严格大于当前的版本
     * @param reset_controller 若为 true，则在下次控制器更新前将控制器复位
     * @return true：参数已接受并应用
     *         false：版本过旧或校验失败，调用 `getLastErrorCode()` 获取原因
     */
    bool applyParams(const ControllerParamsPlain &params, bool reset_controller);

    /**
     * @brief 执行一次控制器更新。
     *
     * 读取 input_，更新 RL-PID 控制器参数，并将计算出的力/力矩和调试数据存入 wrench_ 和 debug_
     */
    void step();

    /**
     * @brief 重置控制器内部状态
     */
    void resetController();

    /// @return 力/力矩
    WrenchPlain getWrench() const;

    /// @return 调试数据
    DebugPlain getDebug() const;

    /**
     * @return 最近一次 `applyParams()` 的错误码
     *         0 = 成功，2 = 版本过旧，3 = 参数越界
     */
    int32_t getLastErrorCode() const;

private:
    /**
     * @brief 将 params_ 中的每个字段同步到相关对象
     */
    void syncSubObjectParams_();

    /**
     * @brief 重置控制器内部状态（内部实现）
     */
    void resetController_();

    /**
     * @brief 对候选 parames_ 做范围检查
     * @return true 表示候选 parames_ 有效
     */
    bool validateParams_(const ControllerParamsPlain &params) const;

    ControlInputPlain input_;           // 控制输入
    ControllerParamsPlain params_;      // 控制参数

    WrenchPlain wrench_;                // 力/力矩
    DebugPlain debug_;                  // 调试数据

    /* 与参考工程的 F、tau 对应 */
    _iq force_iq_  = 0;
    _iq moment_iq_ = 0;

    static constexpr int64_t kControlInputTimeoutUs = 500000; // 500 ms
    int64_t last_control_input_rx_us_ = 0;
    bool input_unavailable_ = true;

    uint32_t last_param_version_ = 0;   // 上次接受的 params_ 版本号
    int32_t  last_error_code_    = 0;   // 上次 applyParams() 的错误码

    Observer       observer_x_;      // X 方向观测器
    Observer       observer_y_;      // Y 方向观测器
    GuidanceLaw    guidancelaw_;     // 导引律
    RLPIDController controller_v_;   // 纵向速度控制器
    RLPIDController controller_psi_; // 航向角控制器
};

/** 全局单例 —— 在此声明，在对应的 .cpp 中定义
 * */
extern ASVControlApp g_app;
