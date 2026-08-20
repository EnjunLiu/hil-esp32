#include "control/controller.hpp"

namespace {

void init_r_matrix(IQMatrix3X3 &R)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R.at(i, j) = (i == j) ? _IQ(0.001) : _IQ(0);
        }
    }
}

} // namespace

Controller::Controller()
    : Delta_t(_IQ(0)), state(), next_state(), Phi(), next_Phi(),
      mu(_IQ(1.0), _IQ(0.0), _IQ(0.0),
         _IQ(0.0), _IQ(1.0), _IQ(0.0),
         _IQ(0.0), _IQ(0.0), _IQ(1.0)),
      w_V(), w_P(), w_I(), w_D(), R(), u(),
      gamma(_IQ(0)), lambda_rls(_IQ(0)), decay(_IQ(0)),
      W_bound(_IQ(0)), I_bound(_IQ(0)), D_bound(_IQ(0)), output_bound(_IQ(0)),
      P(), I(), D(), pid_gain_max(_IQ(0)), pid_gain_min(_IQ(0))
{
    init_r_matrix(R);
}

void Controller::configure(float delta_t, float gamma_rl, float lambda_rls, float decay,
                           float i_bound, float d_bound, float output_bound,
                           float w_bound, float pid_gain_max, float pid_gain_min)
{
    Delta_t = _IQ(delta_t);
    gamma = _IQ(gamma_rl);
    lambda_rls = _IQ(lambda_rls);
    decay = _IQ(decay);
    I_bound = _IQ(i_bound);
    D_bound = _IQ(d_bound);
    output_bound = _IQ(output_bound);
    W_bound = _IQ(w_bound);
    this->pid_gain_max = _IQ(pid_gain_max);
    this->pid_gain_min = _IQ(pid_gain_min);
}

void Controller::resetAdaptiveState()
{
    state = next_state = Phi = next_Phi = {};
    w_V = w_P = w_I = w_D = u = {};
    P = I = D = _IQ(0);
    init_r_matrix(R);
}

/* 计算控制器输出 */
_iq Controller::output(const _iq& new_error, const _iq& gradsign) {
    state = next_state;
    Phi = compute_feature(state);

    next_state[0] = new_error;
    next_state[1] = _IQsat(next_state[1] + _IQmpy(new_error, Delta_t), I_bound, -I_bound);
    next_state[2] = _IQsat(_IQdiv(new_error - state[0], Delta_t), D_bound, -D_bound);

    next_Phi = compute_feature(next_state);

    _iq r = _IQdiv2(_IQmpy(state[0], state[0]));                    // 强化信号
    _iq delta = r + _IQmpy(gamma, next_Phi * w_V) - Phi * w_V;      // 时序差分信号

    // 计算Psi临时量
    IQVector<3> Psi = Phi - gamma * next_Phi;

    _iq d = r;

    R = R * lambda_rls;
    u = u * lambda_rls;

    // Givens 旋转更新
    for (int j = 0; j < 3; j++) {
        _iq x = R.at(j, j);
        _iq y = Psi[j];
        _iq rr = _IQhypot(x, y);
        _iq c = _IQ(1.0);
        _iq s = _IQ(0.0);
        if (rr != _IQ(0.0)) {
            c = _IQdiv(x, rr);
            s = _IQdiv(y, rr);
        }

        // 更新 R 第 j 行
        R.at(j, j) = rr;
        for (int col = j + 1; col < 3; col++) {
            _iq old = R.at(j, col);
            R.at(j, col) = _IQmpy(c, old) + _IQmpy(s, Psi[col]);
            Psi[col] = _IQmpy(-s, old) + _IQmpy(c, Psi[col]);
        }

        // 更新 u
        _iq old_u_j = u[j];
        u[j] = _IQmpy(c, old_u_j) + _IQmpy(s, d);
        d = _IQmpy(-s, old_u_j) + _IQmpy(c, d);
    }

    // 更新w_V（对R对角元加最小值保护）
    _iq r22 = (R.at(2, 2) > _IQ(1e-4)) ? R.at(2, 2) : _IQ(1e-4);
    _iq r11 = (R.at(1, 1) > _IQ(1e-4)) ? R.at(1, 1) : _IQ(1e-4);
    _iq r00 = (R.at(0, 0) > _IQ(1e-4)) ? R.at(0, 0) : _IQ(1e-4);
    w_V[2] = _IQdiv(u[2], r22);
    w_V[1] = _IQdiv(u[1] - _IQmpy(R.at(1, 2), w_V[2]), r11);
    w_V[0] = _IQdiv(u[0] - _IQmpy(R.at(0, 1), w_V[1]) - _IQmpy(R.at(0, 2), w_V[2]), r00);

    // ----------Actor 更新----------
    IQVector<3> grad_P = _IQmpy(-_IQmpy(state[0], state[0]), gradsign) * Phi;
    IQVector<3> grad_I = _IQmpy(-_IQmpy(state[0], state[1]), gradsign) * Phi;
    IQVector<3> grad_D = _IQmpy(-_IQmpy(state[0], state[2]), gradsign) * Phi;

    _iq norm_grad_P = grad_P.norm() > _IQ(1e-4) ? grad_P.norm() : _IQ(1e-4);
    _iq norm_grad_I = grad_I.norm() > _IQ(1e-4) ? grad_I.norm() : _IQ(1e-4);
    _iq norm_grad_D = grad_D.norm() > _IQ(1e-4) ? grad_D.norm() : _IQ(1e-4);

    IQVector<3> Delta_w_P = _IQdiv(-delta, norm_grad_P) * grad_P;
    IQVector<3> Delta_w_I = _IQdiv(-delta, norm_grad_I) * grad_I;
    IQVector<3> Delta_w_D = _IQdiv(-delta, norm_grad_D) * grad_D;

    w_P = (w_P + Delta_w_P) * decay;
    w_I = (w_I + Delta_w_I) * decay;
    w_D = (w_D + Delta_w_D) * decay;

    for (int i = 0; i < 3; i++) {
        w_P[i] = _IQsat(w_P[i], W_bound, _IQ(0));
        w_I[i] = _IQsat(w_I[i], W_bound, _IQ(0));
        w_D[i] = _IQsat(w_D[i], W_bound, _IQ(0));
    }

    // ---------- 计算 PID 参数 ----------
    P = _IQsat(w_P * Phi, pid_gain_max, pid_gain_min);
    I = _IQsat(w_I * Phi, pid_gain_max, pid_gain_min);
    D = _IQsat(w_D * Phi, pid_gain_max, pid_gain_min);

    return _IQsat(_IQmpy(P, state[0]) + _IQmpy(I, state[1]) + _IQmpy(D, state[2]), output_bound, -output_bound);
}

/* 计算特征向量 */
IQVector<3> Controller::compute_feature(const IQVector<3>& state) const {
    IQVector<3> mu0(mu.m00, mu.m01, mu.m02);
    IQVector<3> mu1(mu.m10, mu.m11, mu.m12);
    IQVector<3> mu2(mu.m20, mu.m21, mu.m22);

    IQVector<3> dist0 = state - mu0;
    IQVector<3> dist1 = state - mu1;
    IQVector<3> dist2 = state - mu2;

    IQVector<3> Phi;

    Phi[0] = _IQdiv(_IQ(1.0), _IQ(1.0) + dist0.norm());
    Phi[1] = _IQdiv(_IQ(1.0), _IQ(1.0) + dist1.norm());
    Phi[2] = _IQdiv(_IQ(1.0), _IQ(1.0) + dist2.norm());
    return Phi;
}
