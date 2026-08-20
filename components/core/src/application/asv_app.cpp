#include "application/asv_app.hpp"
#include <esp_timer.h>

ASVApp g_app;

ASVApp::ASVApp()
    : guidancelaw_()
    , controller_v_()
    , controller_psi_() {}

void ASVApp::init()
{
    input_ = Input{};
    force_ = 0.0f;
    moment_ = 0.0f;

    last_input_rx_us_ = 0;
    input_active_ = false;

    params_ = Params{};
    resetState_();
}

void ASVApp::setInput(const Input &input)
{
    input_ = input;
    last_input_rx_us_ = esp_timer_get_time();
}

bool ASVApp::applyParams(const Params &params, bool reset_state)
{
    if (params.version <= params_.version || !validateParams_(params)) {
        return false;
    }

    params_ = params;
    syncSubObjectParams_();

    if (reset_state) {
        resetState_();
    }

    return true;
}

Output ASVApp::step()
{
    const int64_t now_us = esp_timer_get_time();

    if (!inputActive_(now_us)) {
        if (input_active_) {
            resetState_();
        }
        input_active_ = false;
        return Output{ input_.stamp_us, 0.0f, 0.0f, false };
    }

    input_active_ = true;

    const float desired[ASV_OBSERVER_COUNT] = { input_.desired_x, input_.desired_y };
    for (int i = 0; i < ASV_OBSERVER_COUNT; ++i) {
        observers_[i].update(_IQ(desired[i]));
    }
    guidancelaw_.update(observers_[ASV_OBSERVER_X].v_state_hat, observers_[ASV_OBSERVER_Y].v_state_hat,
                        observers_[ASV_OBSERVER_X].state_hat, observers_[ASV_OBSERVER_Y].state_hat);

    const _iq surge_velocity_iq = _IQ(input_.surge_velocity);
    const _iq yaw_rate_iq = _IQ(input_.yaw_rate);
    _iq force_iq = _IQ(force_);
    _iq moment_iq = _IQ(moment_);
    const _iq gradsign_force =
        (_IQmpy(force_iq, surge_velocity_iq) < _IQ(0.0)) ? _IQ(-1.0) : _IQ(1.0);
    const _iq gradsign_moment =
        (_IQmpy(moment_iq, yaw_rate_iq) < _IQ(0.0)) ? _IQ(-1.0) : _IQ(1.0);

    force_iq += controller_v_.output(guidancelaw_.desired_velocity, gradsign_force);
    moment_iq = controller_psi_.output(guidancelaw_.desired_angle, gradsign_moment);

    force_ = _IQtoF(force_iq);
    moment_ = _IQtoF(moment_iq);
    return Output{ input_.stamp_us, force_, moment_, true };
}

bool ASVApp::inputActive_(int64_t now_us) const
{
    const int64_t timeout_us = (int64_t)(params_.input_timeout_s * 1000000.0f);
    return input_.valid &&
           last_input_rx_us_ != 0 &&
           (now_us - last_input_rx_us_) <= timeout_us;
}

void ASVApp::resetState()
{
    resetState_();
}

void ASVApp::syncSubObjectParams_()
{
    const Params &p = params_;

    for (int i = 0; i < ASV_OBSERVER_COUNT; ++i) {
        observers_[i].configure(p.time_constant, p.v_max, p.e_max, p.delta_t);
    }
    guidancelaw_.configure(p.time_constant, p.delta_t);

    controller_v_.configure(
        p.delta_t, p.gamma_rl, p.lambda_rls, p.decay_force,
        p.i_bound_force, p.d_bound_force, p.max_force,
        p.w_bound, p.pid_gain_max, p.pid_gain_min);
    controller_psi_.configure(
        p.delta_t, p.gamma_rl, p.lambda_rls, p.decay_moment,
        p.i_bound_moment, p.d_bound_moment, p.max_moment,
        p.w_bound, p.pid_gain_max, p.pid_gain_min);
}

void ASVApp::resetState_()
{
    for (int i = 0; i < ASV_OBSERVER_COUNT; ++i) {
        observers_[i].resetState();
    }
    guidancelaw_.resetState();
    controller_v_.resetAdaptiveState();
    controller_psi_.resetAdaptiveState();

    force_ = 0.0f;
    moment_ = 0.0f;

    syncSubObjectParams_();
}

bool ASVApp::validateParams_(const Params &params) const
{
    if (params.time_constant <= 0.001f || params.time_constant > 100.0f) return false;
    if (params.v_max < 0.0f || params.v_max > 100.0f) return false;
    if (params.e_max <= 0.001f || params.e_max > 100.0f) return false;
    if (params.delta_t < 0.001f || params.delta_t > 1.0f) return false;
    if (params.gamma_rl < 0.0f || params.gamma_rl > 1.0f) return false;
    if (params.lambda_rls < 0.0f || params.lambda_rls > 1.0f) return false;
    if (params.decay_force < 0.0f || params.decay_force > 1.0f) return false;
    if (params.decay_moment < 0.0f || params.decay_moment > 1.0f) return false;
    if (params.w_bound <= 0.0f || params.w_bound > 1000.0f) return false;
    if (params.pid_gain_max <= params.pid_gain_min) return false;
    if (params.pid_gain_max > 10000.0f) return false;
    if (params.pid_gain_min <= 0.0f) return false;
    if (params.max_force < 0.0f || params.max_force > 10000.0f) return false;
    if (params.max_moment < 0.0f || params.max_moment > 10000.0f) return false;
    if (params.i_bound_force < 0.0f || params.i_bound_force > 1000.0f) return false;
    if (params.d_bound_force < 0.0f || params.d_bound_force > 1000.0f) return false;
    if (params.i_bound_moment < 0.0f || params.i_bound_moment > 1000.0f) return false;
    if (params.d_bound_moment < 0.0f || params.d_bound_moment > 1000.0f) return false;
    if (params.input_timeout_s <= 0.001f || params.input_timeout_s > 10.0f) return false;
    return true;
}
