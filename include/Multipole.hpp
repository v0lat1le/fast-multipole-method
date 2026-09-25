#pragma once

#include <array>
#include <complex>

#include "Vec2.hpp"


template <std::size_t P>
struct Multipole {
    double q;
    std::array<std::complex<double>, P> a;

    constexpr Multipole& operator+=(const Multipole& rhs) noexcept {
        q += rhs.q;
        for (int i=0; i<P; ++i) {
            a[i] += rhs.a[i];
        }
        return *this;
    }
    friend constexpr Multipole operator+(Multipole lhs, const Multipole& rhs) noexcept {
        lhs += rhs;
        return lhs;
    }
};

template <std::size_t P>
constexpr Multipole<P> calculate_multipole(double charge, Vec2d dr) noexcept {
    Multipole<P> result = {charge};
    auto z = std::complex<double>(dr.x, dr.y);
    auto z_power = std::complex<double>(charge);
    for (int k=0; k<P; ++k) {
        z_power *= z;
        result.a[k] -= z_power/(k+1.0);
    }
    return result;
}

template <std::size_t P>
constexpr Vec2d evaluate_multipole(const Multipole<P>& multipole, Vec2d dr) noexcept {
    auto z_inv = 1.0/std::complex<double>(dr.x, dr.y);
    auto accel = multipole.q*z_inv;
    auto z_power = z_inv;
    for (int k=0; k<P; ++k) {
        z_power *= z_inv;
        accel -= (k+1.0)*multipole.a[k]*z_power;
    }
    return Vec2d(-accel.real(), accel.imag());
}

template <std::size_t P>
using Local = std::array<std::complex<double>, P>;

template <std::size_t P>
constexpr Vec2d evaluate_local(const Local<P>& local, Vec2d dr) noexcept {
    auto z = std::complex<double>(dr.x, dr.y);
    auto z_power = std::complex<double>(1.0);
    auto accel = std::complex<double>();
    for (int l=0; l<P; ++l) {
        accel += (l+1.0)*local[l]*z_power;
        z_power *= z;  // TODO: uneccesary mul on last iter
    }
    return Vec2d(-accel.real(), accel.imag());
}

template <std::size_t P>
struct Binomial {
    std::array<double, P*(P+1)/2> coefficients;
    constexpr Binomial() noexcept {
        coefficients[0] = 1.0;
        for (int n=1; n<P; ++n) {
            coefficients[n*(n+1)/2] = 1.0;
            for (int k=1; k<n; ++k) {
                coefficients[n*(n+1)/2+k] = operator()(n-1, k-1) + operator()(n-1, k);
            }
            coefficients[n*(n+1)/2+n] = 1.0;
        }
    }

    constexpr double operator()(int n, int k) const noexcept {
        return coefficients[n*(n+1)/2 + k];
    }
};

template <std::size_t P>
constexpr Multipole<P> translate_multipole(const Multipole<P>& multipole, Vec2d dr) noexcept {
    static constexpr auto binoms = Binomial<P>();
    Multipole<P> result = { multipole.q, {} };
    std::array<std::complex<double>, P+1> z_power = {1.0, std::complex<double>(dr.x, dr.y)};
    for (int k=2; k<P+1; ++k) {
        z_power[k] = z_power[k-1]*z_power[1];
    }
    for (int l=0; l<P; ++l) {
        result.a[l] = -multipole.q*z_power[l+1]/(l+1.0);
        for (int k=0; k<=l; ++k) {
            result.a[l] += multipole.a[k]*z_power[l-k]*binoms(l,k);
        }
    }
    return result;
}

template <std::size_t P>
constexpr Local<P> convert_to_local(const Multipole<P>& multipole, Vec2d dr) noexcept {
    static constexpr auto binoms = Binomial<2*P>();
    auto z0 = std::complex<double>(dr.x, dr.y);
    std::array<std::complex<double>, P> z_power = { 1.0/z0 };
    for (int k=1; k<P; ++k) {
        z_power[k] = z_power[k-1]*z_power[0];
    }
    Local<P> result = {}; // not computing b0 as it doesn't contribute to the force;
    for (int k=0; k<P; ++k) {
        auto v0 = multipole.a[k]*z_power[k];
        if ((k&1) == 0) {
            v0 = -v0;
        }
        for (int l=0; l<P; ++l) {
            result[l] += v0*binoms(l+1+k, k);
        }
    }
    for (int l=0; l<P; ++l) {
        result[l] -= multipole.q/(l+1.0);
        result[l] *= z_power[l];
    }
    return result;
}

template <std::size_t P>
constexpr Local<P> translate_local(const Local<P>& local, Vec2d dr) noexcept {
    auto z0 = std::complex<double>(dr.x, dr.y);
    auto result = local;
    for (int j=0; j<P-1; ++j) {
        for (int k=P-j-2; k<P-1; ++k) {
            result[k] -= z0*result[k+1];
        }
    }
    for (int k=0; k<P-1; ++k) { // one extra round as we dropped b0
        result[k] -= z0*result[k+1];
    }
    return result;
}
