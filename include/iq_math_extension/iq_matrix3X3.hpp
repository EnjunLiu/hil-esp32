#pragma once

#include "IQmathLib.h"
#include "iq_vector.hpp"

struct IQMatrix3X3 {
    _iq m00, m01, m02,
        m10, m11, m12,
        m20, m21, m22;

    IQMatrix3X3() : m00(_IQ(0)), m01(_IQ(0)), m02(_IQ(0)),
                    m10(_IQ(0)), m11(_IQ(0)), m12(_IQ(0)),
                    m20(_IQ(0)), m21(_IQ(0)), m22(_IQ(0)) {}

    IQMatrix3X3(_iq _m00, _iq _m01, _iq _m02,
                _iq _m10, _iq _m11, _iq _m12,
                _iq _m20, _iq _m21, _iq _m22)
                : m00(_m00), m01(_m01), m02(_m02),
                  m10(_m10), m11(_m11), m12(_m12),
                  m20(_m20), m21(_m21), m22(_m22) {}

    IQMatrix3X3(float _m00, float _m01, float _m02,
                float _m10, float _m11, float _m12,
                float _m20, float _m21, float _m22)
                : m00(_IQ(_m00)), m01(_IQ(_m01)), m02(_IQ(_m02)),
                  m10(_IQ(_m10)), m11(_IQ(_m11)), m12(_IQ(_m12)),
                  m20(_IQ(_m20)), m21(_IQ(_m21)), m22(_IQ(_m22)) {}

    /* 矩阵求逆 */
    IQMatrix3X3 Invert() const;

    _iq at(int x, int y) const {
        return *(&m00 + x * 3 + y);
    }

    _iq& at(int x, int y) {
        return *(&m00 + x * 3 + y);
    }

    /* 矩阵乘法（矩阵 × 矩阵） */
    IQMatrix3X3 operator*(const IQMatrix3X3& other) const;
    /* 矩阵乘法（矩阵 × 向量） */
    IQVector<3> operator*(const IQVector<3>& vec) const;
    /* 矩阵乘法（矩阵 × 标量） */
    IQMatrix3X3 operator*(const _iq& scalar) const;

};
