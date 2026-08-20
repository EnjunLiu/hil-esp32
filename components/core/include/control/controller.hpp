#pragma once

#include "IQmathLib.h"
#include "iq_math_extension/iq_vector.hpp"
#include "iq_math_extension/iq_matrix3X3.hpp"
#include "iq_math_extension/iq_scalar.hpp"

struct Controller {
    _iq Delta_t;

    IQVector<3> state;      // 当前状态
    IQVector<3> next_state; // 下一状态

    IQVector<3> Phi;      // 当前特征向量
    IQVector<3> next_Phi; // 下一特征向量
    IQMatrix3X3 mu;       // 特征向量中心矩阵

    IQVector<3> w_V; // 价值函数权重
    IQVector<3> w_P; // P权重
    IQVector<3> w_I; // I权重
    IQVector<3> w_D; // D权重

    IQMatrix3X3 R;   // RLS矩阵
    IQVector<3> u;   // RLS输入向量

    _iq gamma;       // 折扣因子
    _iq lambda_rls;  // RLS遗忘因子

    _iq decay;        // PID权重衰减因子
    _iq W_bound;      // 权重更新限幅
    _iq I_bound;      // 积分限幅
    _iq D_bound;      // 微分限幅
    _iq output_bound; // 输出限幅

    _iq P; // PID P参数
    _iq I; // PID I参数
    _iq D; // PID D参数

    _iq pid_gain_max;
    _iq pid_gain_min;

    Controller();
    void configure(float delta_t, float gamma_rl, float lambda_rls, float decay,
                   float i_bound, float d_bound, float output_bound,
                   float w_bound, float pid_gain_max, float pid_gain_min);
    void resetAdaptiveState();

    /* 更新控制器并计算控制器输出 */
    _iq output(const _iq& new_error, const _iq& gradsign);

private:
    /* 计算特征向量 */
    IQVector<3> compute_feature(const IQVector<3>& state) const;
};
