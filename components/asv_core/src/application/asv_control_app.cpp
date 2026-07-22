/**
 * @file    asv_control_app.cpp
 * @brief   ASV控制应用层实现
 */

#include "application/asv_control_app.hpp"
#include "iq_math_extension/iq_scalar.hpp"
#include <esp_timer.h>

/* --------------------------------------------------------------------------
 * 全局实例 —— 阶段一构造在此处完成（静态初始化，先于 RTOS）
 * -------------------------------------------------------------------------- */
ASVControlApp g_app;

ASVControlApp::ASVControlApp()
    : params_{}       // 使用 ControllerParamsPlain 的默认成员初始值
    , observer_x_(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t)
    , observer_y_(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t)
    , guidancelaw_(params_.time_constant, params_.delta_t)
    , controller_v_(params_.delta_t, params_.i_bound_force, params_.d_bound_force, params_.max_force)
    , controller_psi_(params_.delta_t, params_.i_bound_moment, params_.d_bound_moment, params_.max_moment) {}

void ASVControlApp::init()
{
    /* ---- 1. I/O 缓冲区复位 ---- */
    input_  = ControlInputPlain{};
    wrench_ = WrenchPlain{};
    debug_  = DebugPlain{};

    force_iq_  = _IQ(0.0);
    moment_iq_ = _IQ(0.0);

    last_control_input_rx_us_ = 0;
    input_unavailable_ = true;

    /* ---- 2. 错误追踪信息复位 ---- */
    last_param_version_ = 0;
    last_error_code_    = 0;

    /* ---- 3. 恢复默认参数并同步到所有对象 ----*/
    params_ = ControllerParamsPlain{};
    syncSubObjectParams_();

    /* ---- 4. 控制器复位---- */
    resetController_();
}

/* =========================================================================
 * 运行时接口
 * ========================================================================= */

void ASVControlApp::setControlInput(const ControlInputPlain &input)
{
    input_ = input;
    last_control_input_rx_us_ = esp_timer_get_time();
}

bool ASVControlApp::applyParams(const ControllerParamsPlain &params, bool reset_controller)
{
    /* 拒绝旧版本或相同版本 */
    if (params.version <= last_param_version_) {
        last_error_code_ = 2;
        return false;
    }

    /* 范围校验 */
    if (!validateParams_(params)) {
        last_error_code_ = 3;
        return false;
    }

    /* 接受新参数集 */
    params_ = params;
    last_param_version_ = params.version;

    /* 同步到所有子对象 */
    syncSubObjectParams_();

    /* 可选：控制器复位 */
    if (reset_controller) {
        resetController_();
    }

    last_error_code_ = 0;
    return true;
}

void ASVControlApp::step()
{
    const int64_t now_us = input_.stamp_us;

    wrench_.seq++;                       // 自增序号，首次调用由 0 → 1
    wrench_.stamp_us = now_us;

    debug_.seq      = wrench_.seq;
    debug_.stamp_us = now_us;

    const bool input_fresh =
        last_control_input_rx_us_ > 0 &&
        now_us >= last_control_input_rx_us_ &&
        (now_us - last_control_input_rx_us_) <= kControlInputTimeoutUs;

    if (!input_.valid || !input_fresh) {
        force_iq_  = _IQ(0.0);
        moment_iq_ = _IQ(0.0);

        wrench_.force  = 0.0f;
        wrench_.moment = 0.0f;
        wrench_.valid  = false;

        if (!input_unavailable_) {
            resetController_();
        }

        input_unavailable_ = true;
        return;
    }

    input_unavailable_ = false;

    /* ---- 控制流水线 ---- */

    // 1. 观测器
    observer_x_.update(_IQ(input_.desired_x));
    observer_y_.update(_IQ(input_.desired_y));

    // 2. 导引律
    guidancelaw_.update(observer_x_.v_state_hat, observer_y_.v_state_hat,
                        observer_x_.state_hat,    observer_y_.state_hat);



    // 3. 控制器

    const _iq surge_velocity_iq = _IQ(input_.surge_velocity);
    const _iq yaw_rate_iq       = _IQ(input_.yaw_rate);
    const _iq gradsign_force =
    (_IQmpy(force_iq_, surge_velocity_iq) < _IQ(0.0))
        ? _IQ(-1.0)
        : _IQ(1.0);

    const _iq gradsign_moment =
        (_IQmpy(moment_iq_, yaw_rate_iq) < _IQ(0.0))
            ? _IQ(-1.0)
            : _IQ(1.0);
    /* 参考工程：Delta_F 是增量，F 对它进行累加 */
    const _iq delta_force_iq = controller_v_.output(
        guidancelaw_.desired_velocity,
        gradsign_force);

    force_iq_ += delta_force_iq;

    /* 参考工程：tau 直接使用航向控制器输出 */
    moment_iq_ = controller_psi_.output(
        guidancelaw_.desired_angle,
        gradsign_moment);

    /* 只在发布边界转换成浮点数 */
    wrench_.force  = _IQtoF(force_iq_);
    wrench_.moment = _IQtoF(moment_iq_);
    wrench_.valid  = true;

    /* ---- 维护调试数据 ---- */
    debug_.x_hat   = _IQtoF(observer_x_.state_hat);
    debug_.y_hat   = _IQtoF(observer_y_.state_hat);
    debug_.v_hat_x = _IQtoF(observer_x_.v_state_hat);
    debug_.v_hat_y = _IQtoF(observer_y_.v_state_hat);
    debug_.v_hat   = _IQtoF(_IQhypot(observer_x_.v_state_hat, observer_y_.v_state_hat));

    debug_.theta      = _IQtoF(guidancelaw_.theta);
    debug_.e_track_x  = _IQtoF(guidancelaw_.e_track_x);
    debug_.e_track_y  = _IQtoF(guidancelaw_.e_track_y);
    debug_.a          = _IQtoF(guidancelaw_.A);
    debug_.varepsilon = _IQtoF(guidancelaw_.varepsilon);
    debug_.delta_v    = _IQtoF(guidancelaw_.desired_velocity);
    debug_.delta_psi  = _IQtoF(guidancelaw_.desired_angle);

    debug_.p_f = _IQtoF(controller_v_.P);
    debug_.i_f = _IQtoF(controller_v_.I);
    debug_.d_f = _IQtoF(controller_v_.D);
    debug_.p_m = _IQtoF(controller_psi_.P);
    debug_.i_m = _IQtoF(controller_psi_.I);
    debug_.d_m = _IQtoF(controller_psi_.D);
}

WrenchPlain ASVControlApp::getWrench() const
{
    return wrench_;
}

DebugPlain ASVControlApp::getDebug() const
{
    return debug_;
}

int32_t ASVControlApp::getLastErrorCode() const
{
    return last_error_code_;
}

void ASVControlApp::resetController()
{
    resetController_();
}


/**
 * @brief 将 params_ 中的每个字段推送到相关对象
 */
void ASVControlApp::syncSubObjectParams_()
{
    /* ---- 观测器 ---- */
    observer_x_.T       = _IQ(params_.time_constant);
    observer_x_.v_max   = _IQ(params_.v_max);
    observer_x_.e_max   = _IQ(params_.e_max);
    observer_x_.Delta_t = _IQ(params_.delta_t);

    observer_y_.T       = _IQ(params_.time_constant);
    observer_y_.v_max   = _IQ(params_.v_max);
    observer_y_.e_max   = _IQ(params_.e_max);
    observer_y_.Delta_t = _IQ(params_.delta_t);

    /* ---- 导引律 ---- */
    guidancelaw_.T       = _IQ(params_.time_constant);
    guidancelaw_.Delta_t = _IQ(params_.delta_t);

    /* ---- 控制器 ---- */
    controller_v_.Delta_t      = _IQ(params_.delta_t);
    controller_v_.gamma        = _IQ(params_.gamma_rl);
    controller_v_.lambda_rls   = _IQ(params_.lambda_rls);
    controller_v_.decay        = _IQ(0.999);
    controller_v_.I_bound      = _IQ(params_.i_bound_force);
    controller_v_.D_bound      = _IQ(params_.d_bound_force);
    controller_v_.output_bound = _IQ(params_.max_force);

    controller_psi_.Delta_t      = _IQ(params_.delta_t);
    controller_psi_.gamma        = _IQ(params_.gamma_rl);
    controller_psi_.lambda_rls   = _IQ(params_.lambda_rls);
    controller_psi_.decay        = _IQ(0.99);
    controller_psi_.I_bound      = _IQ(params_.i_bound_moment);
    controller_psi_.D_bound      = _IQ(params_.d_bound_moment);
    controller_psi_.output_bound = _IQ(params_.max_moment);
}

/**
 * @brief 重置控制器的内部状态
 */
void ASVControlApp::resetController_()
{
    controller_v_ = RLPIDController(
        params_.delta_t,
        params_.i_bound_force,
        params_.d_bound_force,
        params_.max_force);

    controller_psi_ = RLPIDController(
        params_.delta_t,
        params_.i_bound_moment,
        params_.d_bound_moment,
        params_.max_moment);

    force_iq_  = _IQ(0.0);
    moment_iq_ = _IQ(0.0);

    wrench_.force  = 0.0f;
    wrench_.moment = 0.0f;
    wrench_.valid  = false;

    syncSubObjectParams_();
}

bool ASVControlApp::validateParams_(const ControllerParamsPlain &params) const
{
    /* 时间常数：必须为正且在合理范围内 */
    if (params.time_constant <= 0.001f || params.time_constant > 100.0f) return false;

    /* 速度 / 误差限幅：速度非负，误差为正 */
    if (params.v_max < 0.0f   || params.v_max > 100.0f)  return false;
    if (params.e_max <= 0.001f || params.e_max > 100.0f)  return false;

    /* 时间步长 */
    if (params.delta_t < 0.001f || params.delta_t > 1.0f) return false;

    /* RL 折扣因子 & RLS 遗忘因子：[0, 1] */
    if (params.gamma_rl  < 0.0f || params.gamma_rl  > 1.0f) return false;
    if (params.lambda_rls < 0.0f || params.lambda_rls > 1.0f) return false;

    /* 执行器饱和限幅：非负且有界 */
    if (params.max_force  < 0.0f || params.max_force  > 10000.0f) return false;
    if (params.max_moment < 0.0f || params.max_moment > 10000.0f) return false;

    /* PID 积分/微分限幅：非负且有界 */
    if (params.i_bound_force  < 0.0f || params.i_bound_force  > 1000.0f) return false;
    if (params.d_bound_force  < 0.0f || params.d_bound_force  > 1000.0f) return false;
    if (params.i_bound_moment < 0.0f || params.i_bound_moment > 1000.0f) return false;
    if (params.d_bound_moment < 0.0f || params.d_bound_moment > 1000.0f) return false;

    return true;
}
