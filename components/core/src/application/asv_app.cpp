#include "application/asv_app.hpp"
#include <esp_timer.h>

ASVApp g_app;

ASVApp::ASVApp()
    : params_{}
    , observer_x_(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t)
    , observer_y_(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t)
    , guidancelaw_(params_.time_constant, params_.delta_t)
    , controller_v_(params_.delta_t, params_.i_bound_force, params_.d_bound_force, params_.max_force)
    , controller_psi_(params_.delta_t, params_.i_bound_moment, params_.d_bound_moment, params_.max_moment) {}

void ASVApp::init()
{
    input_ = InputPlain{};
    force_ = 0.0f;
    moment_ = 0.0f;

    last_input_rx_us_ = 0;
    input_unavailable_ = true;
    last_param_version_ = 0;

    params_ = ParamsPlain{};
    observer_x_ = Observer(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t);
    observer_y_ = Observer(params_.time_constant, params_.v_max, params_.e_max, params_.delta_t);
    guidancelaw_ = GuidanceLaw(params_.time_constant, params_.delta_t);
    resetState_();
}

void ASVApp::setInput(const InputPlain &input)
{
    input_ = input;
    last_input_rx_us_ = esp_timer_get_time();
}

bool ASVApp::applyParams(const ParamsPlain &params, bool reset_state)
{
    if (params.version <= last_param_version_ || !validateParams_(params)) {
        return false;
    }

    params_ = params;
    last_param_version_ = params.version;
    syncSubObjectParams_();

    if (reset_state) {
        resetState_();
    }

    return true;
}

WrenchPlain ASVApp::step()
{
    const int64_t now_us = esp_timer_get_time();

    WrenchPlain wrench;
    wrench.stamp_us = input_.stamp_us;

    const bool input_fresh =
        last_input_rx_us_ > 0 &&
        now_us >= last_input_rx_us_ &&
        (now_us - last_input_rx_us_) <= kInputTimeoutUs;

    if (!input_.valid || !input_fresh) {
        force_ = 0.0f;
        moment_ = 0.0f;
        wrench.force = force_;
        wrench.moment = moment_;
        wrench.valid = false;

        if (!input_unavailable_) {
            resetState_();
        }

        input_unavailable_ = true;
        return wrench;
    }

    input_unavailable_ = false;

    observer_x_.update(_IQ(input_.desired_x));
    observer_y_.update(_IQ(input_.desired_y));
    guidancelaw_.update(observer_x_.v_state_hat, observer_y_.v_state_hat,
                        observer_x_.state_hat, observer_y_.state_hat);

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
    wrench.force = force_;
    wrench.moment = moment_;
    wrench.valid = true;
    return wrench;
}

void ASVApp::resetState()
{
    resetState_();
}

void ASVApp::syncObserver_(Observer &obs, const ParamsPlain &params)
{
    obs.T = _IQ(params.time_constant);
    obs.v_max = _IQ(params.v_max);
    obs.e_max = _IQ(params.e_max);
    obs.Delta_t = _IQ(params.delta_t);
}

void ASVApp::syncController_(Controller &ctrl, const ParamsPlain &params,
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

void ASVApp::syncSubObjectParams_()
{
    syncObserver_(observer_x_, params_);
    syncObserver_(observer_y_, params_);

    guidancelaw_.T = _IQ(params_.time_constant);
    guidancelaw_.Delta_t = _IQ(params_.delta_t);

    syncController_(controller_v_, params_, 0.999f,
                    params_.i_bound_force, params_.d_bound_force, params_.max_force);
    syncController_(controller_psi_, params_, 0.99f,
                    params_.i_bound_moment, params_.d_bound_moment, params_.max_moment);
}

void ASVApp::resetState_()
{
    controller_v_ = Controller(
        params_.delta_t, params_.i_bound_force, params_.d_bound_force, params_.max_force);
    controller_psi_ = Controller(
        params_.delta_t, params_.i_bound_moment, params_.d_bound_moment, params_.max_moment);

    force_ = 0.0f;
    moment_ = 0.0f;

    syncSubObjectParams_();
}

bool ASVApp::validateParams_(const ParamsPlain &params) const
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
