#include "iq_math_extension/iq_scalar.hpp"

/* 快速计算_iq类型变量的勾股斜边长 */
_iq _IQhypot(_iq a, _iq b) {
    return _IQsqrt(_IQmpy(a, a) + _IQmpy(b, b));
}

/* _iq类型变量的符号函数 */
_iq _IQsign(_iq x) {
    if (x > _IQ(0.0)) return _IQ(1.0);
    else if (x < _IQ(0.0)) return _IQ(-1.0);
    else return _IQ(0.0);
}

/* _iq类型变量的xi函数 */
/* 1.18920712 ≈ 1/sqrt(2) */
_iq _IQxi(_iq x) {
    return _IQmpy(_IQsqrt(_IQabs(x)) + _IQmpy(_IQ(1.18920712), _IQsqrt(_IQmpy(_IQabs(x), _IQmpy(x, x)))), _IQsign(x));
}

/* 内联快速幂函数实现 */
static inline _iq _IQpowi_impl(_iq base, int e) {
    _iq result = _IQ(1.0);
    _iq b = base;
    while (e) {
        if (e & 1) result = _IQmpy(result, b);
        b = _IQmpy(b, b);
        e >>= 1;
    }
    return result;
}

/* _iq类型变量的整数阶乘方函数 */
_iq _IQpowi(_iq base, _iq exp) {
    int e = _IQint(exp);
    if (e == 0)  return _IQ(1.0);
    if (e == 1)  return base;
    if (e == -1) return _IQdiv(_IQ(1.0), base);
    if (e > 0)   return _IQpowi_impl(base, e);
    return _IQdiv(_IQ(1.0), _IQpowi_impl(base, -e));
}