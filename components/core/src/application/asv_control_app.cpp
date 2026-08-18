/**
 * @file    asv_control_app.cpp
 * @brief   ASV控制应用层实现
 */

#include "application/asv_control_app.hpp"
#include <esp_timer.h>

ASVControlApp g_app;

ASVControlApp::ASVControlApp()
    : params_{}
    , observer_x_(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t)
    , observer_y_(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t)
    , guidancelaw_(params_.time_constant, params_.delta_t)
    , controller_v_(params_.delta_t, params_.i_bound_force, params_.d_bound_force, params_.max_force)
    , controller_psi_(params_.delta_t, params_.i_bound_moment, params_.d_bound_moment, params_.max_moment) {}

void ASVControlApp::init()
{
    input_ = ControlInputPlain{};
    wrench_ = WrenchPlain{};

    force_iq_ = _IQ(0.0);
    moment_iq_ = _IQ(0.0);

    last_control_input_rx_us_ = 0;
    input_unavailable_ = true;
    last_param_version_ = 0;

    params_ = ControllerParamsPlain{};
    observer_x_ = Observer(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t);
    observer_y_ = Observer(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t);
    guidancelaw_ = GuidanceLaw(params_.time_constant, params_.delta_t);
    resetControllerState_();
}

void ASVControlApp::setControlInput(const ControlInputPlain &input)
{
    input_ = input;
    last_control_input_rx_us_ = esp_timer_get_time();
}

bool ASVControlApp::applyParams(const ControllerParamsPlain &params, bool reset_controller)
{
    if (params.version <= last_param_version_ || !validateParams_(params)) {
        return false;
    }

    params_ = params;
    last_param_version_ = params.version;
    syncSubObjectParams_();

    if (reset_controller) {
        resetControllerState_();
    }

    return true;
}

void ASVControlApp::step()
{
    const int64_t now_us = esp_timer_get_time();

    wrench_.seq++;
    wrench_.stamp_us = input_.stamp_us;

    const bool input_fresh =
        last_control_input_rx_us_ > 0 &&
        now_us >= last_control_input_rx_us_ &&
        (now_us - last_control_input_rx_us_) <= kControlInputTimeoutUs;

    if (!input_.valid || !input_fresh) {
        force_iq_ = _IQ(0.0);
        moment_iq_ = _IQ(0.0);
        wrench_.force = 0.0f;
        wrench_.moment = 0.0f;
        wrench_.valid = false;

        if (!input_unavailable_) {
            resetControllerState_();
        }

        input_unavailable_ = true;
        return;
    }

    input_unavailable_ = false;

    observer_x_.update(_IQ(input_.desired_x));
    observer_y_.update(_IQ(input_.desired_y));
    guidancelaw_.update(observer_x_.v_state_hat, observer_y_.v_state_hat,
                        observer_x_.state_hat, observer_y_.state_hat);

    const _iq surge_velocity_iq = _IQ(input_.surge_velocity);
    const _iq yaw_rate_iq = _IQ(input_.yaw_rate);
    const _iq gradsign_force =
        (_IQmpy(force_iq_, surge_velocity_iq) < _IQ(0.0)) ? _IQ(-1.0) : _IQ(1.0);
    const _iq gradsign_moment =
        (_IQmpy(moment_iq_, yaw_rate_iq) < _IQ(0.0)) ? _IQ(-1.0) : _IQ(1.0);

    force_iq_ += controller_v_.output(guidancelaw_.desired_velocity, gradsign_force);
    moment_iq_ = controller_psi_.output(guidancelaw_.desired_angle, gradsign_moment);

    wrench_.force = _IQtoF(force_iq_);
    wrench_.moment = _IQtoF(moment_iq_);
    wrench_.valid = true;
}

void ASVControlApp::resetController()
{
    resetControllerState_();
}

void ASVControlApp::syncObserver_(Observer &obs, const ControllerParamsPlain &params)
{
    obs.T = _IQ(params.time_constant);
    obs.v_max = _IQ(params.v_max);
    obs.e_max = _IQ(params.e_max);
    obs.Delta_t = _IQ(params.delta_t);
}

void ASVControlApp::syncRLPID_(RLPIDController &ctrl, const ControllerParamsPlain &params,
                               float decay, float i_bound, float d_bound, float output_bound)
{
    ctrl.Delta_t = _IQ(params.delta_t);
    ctrl.gamma = _IQ(params.gamma_rl);
    ctrl.lambda_rls = _IQ(params.lambda_rls);
    ctrl.decay = _IQ(decay);
    ctrl.I_bound = _IQ(i_bound);
    ctrl.D_bound = _IQ(d_bound);
    ctrl.output_bound = _IQ(output_bound);
}

void ASVControlApp::syncSubObjectParams_()
{
    syncObserver_(observer_x_, params_);
    syncObserver_(observer_y_, params_);

    guidancelaw_.T = _IQ(params_.time_constant);
    guidancelaw_.Delta_t = _IQ(params_.delta_t);

    syncRLPID_(controller_v_, params_, 0.999f,
               params_.i_bound_force, params_.d_bound_force, params_.max_force);
    syncRLPID_(controller_psi_, params_, 0.99f,
               params_.i_bound_moment, params_.d_bound_moment, params_.max_moment);
}

void ASVControlApp::resetControllerState_()
{
    controller_v_ = RLPIDController(
        params_.delta_t, params_.i_bound_force, params_.d_bound_force, params_.max_force);
    controller_psi_ = RLPIDController(
        params_.delta_t, params_.i_bound_moment, params_.d_bound_moment, params_.max_moment);

    force_iq_ = _IQ(0.0);
    moment_iq_ = _IQ(0.0);
    wrench_.force = 0.0f;
    wrench_.moment = 0.0f;
    wrench_.valid = false;

    syncSubObjectParams_();
}

bool ASVControlApp::validateParams_(const ControllerParamsPlain &params) const
{
    if (params.time_constant <= 0.001f || params.time_constant > 100.0f) return false;
    if (params.v_max < 0.0f || params.v_max > 100.0f) return false;
    if (params.e_max <= 0.001f || params.e_max > 100.0f) return false;
    if (params.delta_t < 0.001f || params.delta_t > 1.0f) return false;
    if (params.gamma_rl < 0.0f || params.gamma_rl > 1.0f) return false;
    if (params.lambda_rls < 0.0f || params.lambda_rls > 1.0f) return false;
    if (params.max_force < 0.0f || params.max_force > 10000.0f) return false;
    if (params.max_moment < 0.0f || params.max_moment > 10000.0f) return false;
    if (params.i_bound_force < 0.0f || params.i_bound_force > 1000.0f) return false;
    if (params.d_bound_force < 0.0f || params.d_bound_force > 1000.0f) return false;
    if (params.i_bound_moment < 0.0f || params.i_bound_moment > 1000.0f) return false;
    if (params.d_bound_moment < 0.0f || params.d_bound_moment > 1000.0f) return false;
    return true;
}
