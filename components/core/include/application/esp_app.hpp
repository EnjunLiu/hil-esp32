/**
 * @file    esp_app.hpp
 * @brief   ESP app 固件应用层
 */

#pragma once

#include "esp_types.hpp"
#include "estimator/estimator.hpp"
#include "control/controller.hpp"

#define ESPAPP_OBSERVER_COUNT 2 // 观测器数量
#define ESPAPP_OBSERVER_X 0 // x方向观测器
#define ESPAPP_OBSERVER_Y 1 // y方向观测器

class EspApp {
public:
    EspApp();

    void init(); // 初始化App
    void setInput(const Input &input); // 设置输入
    bool applyParams(const Params &params, bool reset_state); // 应用参数
    Output step(); // 步进并返回输出
    void resetState(); // 重置App

private:
    bool inputActive_(int64_t now_us) const;
    void syncSubObjectParams_(); // 同步子对象参数
    bool validateParams_(const Params &params) const; // 验证参数

    Input input_; // 输入
    Params params_; // 参数

    float force_ = 0.0f;
    float moment_ = 0.0f;

    int64_t last_input_rx_us_ = 0;
    bool input_active_ = false; 

    Observer observers_[ESPAPP_OBSERVER_COUNT]{}; // 观测器
    GuidanceLaw guidancelaw_; // 引导律
    Controller controller_v_; // 纵向速度控制器
    Controller controller_psi_; // 航向角控制器
};

extern EspApp g_app;
