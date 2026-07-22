#include "iq_math_extension/iq_matrix3X3.hpp"

/* 矩阵求逆 */
IQMatrix3X3 IQMatrix3X3::Invert() const {
    _iq det = _IQmpy(m00, _IQmpy(m11, m22) - _IQmpy(m12, m21))
            - _IQmpy(m01, _IQmpy(m10, m22) - _IQmpy(m12, m20))
            + _IQmpy(m02, _IQmpy(m10, m21) - _IQmpy(m11, m20));

    _iq invdet = _IQdiv(_IQ(1.0), det);

    return IQMatrix3X3(
        _IQmpy(invdet, _IQmpy(m11, m22) - _IQmpy(m12, m21)),
        _IQmpy(invdet, _IQmpy(m02, m21) - _IQmpy(m01, m22)),
        _IQmpy(invdet, _IQmpy(m01, m12) - _IQmpy(m02, m11)),

        _IQmpy(invdet, _IQmpy(m12, m20) - _IQmpy(m10, m22)),
        _IQmpy(invdet, _IQmpy(m00, m22) - _IQmpy(m02, m20)),
        _IQmpy(invdet, _IQmpy(m02, m10) - _IQmpy(m00, m12)),

        _IQmpy(invdet, _IQmpy(m10, m21) - _IQmpy(m11, m20)),
        _IQmpy(invdet, _IQmpy(m01, m20) - _IQmpy(m00, m21)),
        _IQmpy(invdet, _IQmpy(m00, m11) - _IQmpy(m01, m10))
    );
}

/* 矩阵乘法（矩阵 × 矩阵） */
IQMatrix3X3 IQMatrix3X3::operator*(const IQMatrix3X3& other) const {
    IQMatrix3X3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.at(i, j) = _IQ(0);
            for (int k = 0; k < 3; k++) {
                result.at(i, j) += _IQmpy(this->at(i, k), other.at(k, j));
            }
        }
    }
    return result;
}

/* 矩阵乘法（矩阵 × 向量） */
IQVector<3> IQMatrix3X3::operator*(const IQVector<3>& vec) const {
    IQVector<3> result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i] += _IQmpy(this->at(i, j), vec[j]);
        }
    }
    return result;
}

/* 矩阵乘法（矩阵 × 标量） */
IQMatrix3X3 IQMatrix3X3::operator*(const _iq& scalar) const {
    IQMatrix3X3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.at(i, j) = _IQmpy(this->at(i, j), scalar);
        }
    }
    return result;
}
