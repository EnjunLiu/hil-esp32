#pragma once

#include <type_traits>
#include "IQmathLib.h"

template <int N>
class IQVector {
    
    public:

    static_assert(N > 0, "N must be positive");

    _iq data[N];

    IQVector() {
        for (auto &elem : data) {
            elem = _IQ(0);
        }
    }

    template <typename... Args, typename = std::enable_if_t<(std::is_convertible_v<Args, _iq> && ...)>>
    IQVector(Args... args) : data{to_iq(args)...} {
        static_assert(sizeof...(Args) == N, "Number of arguments must match the size of the vector");
    }

    IQVector operator+(const IQVector& other) const {
        IQVector result;
        for (int i = 0; i < N; i++) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    IQVector operator-(const IQVector& other) const {
        IQVector result;
        for (int i = 0; i < N; i++) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    _iq operator*(const IQVector& other) const {
        _iq result = _IQ(0);
        for (int i = 0; i < N; i++) {
            result += _IQmpy(data[i], other.data[i]);
        }
        return result;
    }

    template <typename T, typename = std::enable_if_t<std::is_convertible_v<T, _iq>>>
    IQVector operator*(const T scalar) const {
        IQVector result;
        for (int i = 0; i < N; i++) {
            result.data[i] = _IQmpy(to_iq(scalar), data[i]);
        }
        return result;
    }

    _iq operator[](int index) const {
        return data[index];
    }

    _iq& operator[](int index) {
        return data[index];
    }

    IQVector operator-() const {
        IQVector result;
        for (int i = 0; i < N; i++) {
            result.data[i] = -data[i];
        }
        return result;
    }

    _iq norm() const {
        return _IQsqrt((*this)*(*this));
    }

    private:
    template <typename T>
    static _iq to_iq(T v) {
        if constexpr (std::is_same_v<T, _iq>) {
            return v;       // 已经是 _iq
        } else {
            return _IQ(v);  // float / double → _iq
        }
    }
};

// 自由函数：标量 × 向量（标量在左）
template <int N, typename T, typename = std::enable_if_t<std::is_convertible_v<T, _iq>>>
IQVector<N> operator*(T scalar, const IQVector<N>& vec) {
    return vec * scalar;  // 委托给成员函数 IQVector::operator*(scalar)
}