/**
 * @file    asv_app.hpp
 * @brief   ASV 固件应用层
 */

#pragma once

#include "asv_types.hpp"
#include "estimator/estimator.hpp"
#include "control/controller.hpp"

class ASVApp {
public:
    ASVApp();

    void init(); // 初始化固件
    void setInput(const InputPlain &input); // 设置输入
    bool applyParams(const ParamsPlain &params, bool reset_state); // 应用参数
    WrenchPlain step(); // 步进并返回输出
    void resetState(); // 重置固件运行状态

private:
    void syncSubObjectParams_(); // 同步子对象参数
    void resetState_(); // 重置固件运行状态
    bool validateParams_(const ParamsPlain &params) const; // 验证参数

    static void syncObserver_(Observer &obs, const ParamsPlain &params);
    static void syncController_(Controller &ctrl, const ParamsPlain &params,
                                float decay, float i_bound, float d_bound, float output_bound);

    InputPlain input_; // 输入
    ParamsPlain params_; // 参数

    float force_ = 0.0f;
    float moment_ = 0.0f;

    static constexpr int64_t kInputTimeoutUs = 500000;
    int64_t last_input_rx_us_ = 0;
    bool input_unavailable_ = true;

    uint32_t last_param_version_ = 0;

    Observer observer_x_;
    Observer observer_y_;
    GuidanceLaw guidancelaw_;
    Controller controller_v_;
    Controller controller_psi_;
};

extern ASVApp g_app;
