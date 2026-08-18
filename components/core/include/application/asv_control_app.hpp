/**
 * @file    asv_control_app.hpp
 * @brief   ASV 控制应用层
 */

#pragma once

#include "IQmathLib.h"

#include "asv_types.hpp"
#include "estimator/estimator.hpp"
#include "control/RLPID_controller.hpp"

class ASVControlApp {
public:
    ASVControlApp();

    void init();
    void setControlInput(const ControlInputPlain &input);
    bool applyParams(const ControllerParamsPlain &params, bool reset_controller);
    void step();
    void resetController();

    WrenchPlain getWrench() const { return wrench_; }

private:
    void syncSubObjectParams_();
    void resetControllerState_();
    bool validateParams_(const ControllerParamsPlain &params) const;

    static void syncObserver_(Observer &obs, const ControllerParamsPlain &params);
    static void syncRLPID_(RLPIDController &ctrl, const ControllerParamsPlain &params,
                           float decay, float i_bound, float d_bound, float output_bound);

    ControlInputPlain input_;
    ControllerParamsPlain params_;

    WrenchPlain wrench_;

    _iq force_iq_ = 0;
    _iq moment_iq_ = 0;

    static constexpr int64_t kControlInputTimeoutUs = 500000;
    int64_t last_control_input_rx_us_ = 0;
    bool input_unavailable_ = true;

    uint32_t last_param_version_ = 0;

    Observer observer_x_;
    Observer observer_y_;
    GuidanceLaw guidancelaw_;
    RLPIDController controller_v_;
    RLPIDController controller_psi_;
};

extern ASVControlApp g_app;
