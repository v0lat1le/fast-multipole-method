#pragma once

#include <array>
#include <complex>

#include "Vec2.hpp"


template <std::size_t P>
struct Multipole {
    double q;
    std::array<std::complex<double>, P> a;
};

template <std::size_t P>
Vec2d evaluate_multipole(const Multipole<P>& multipole, Vec2d dr) noexcept {
    auto z_inv = 1.0/std::complex<double>(dr.x, dr.y);  // TODO: this explodes?
    auto accel = multipole.q*z_inv;
    auto z_power = z_inv;
    for (int k=0; k<multipole.a.size(); ++k) {
        z_power *= z_inv;
        accel -= (k+1.0)*multipole.a[k]*z_power;
    }
    return Vec2d(-accel.real(), accel.imag());
}

template <std::size_t P>
using Local = std::array<std::complex<double>, P>;

template <std::size_t P>
Vec2d evaluate_local(const Local<P>& local, Vec2d dr) noexcept {
    auto z = std::complex<double>(dr.x, dr.y);
    auto z_power = std::complex<double>(1.0);
    auto accel = std::complex<double>();
    for (int l=1; l<P; ++l) {
        accel += static_cast<double>(l)*local[l]*z_power;
        z_power *= z;
    }
    return Vec2d(accel.real(), -accel.imag());
}

template <std::size_t P>
struct Binomial {
    std::array<double, P*(P+1)/2> coefficients;
    constexpr Binomial() noexcept {
        coefficients[0] = 1.0;
        for (int n=1; n<P; ++n) {
            coefficients[n*(n+1)/2] = 1.0;
            for (int k=1; k<n; ++k) {
                coefficients[n*(n+1)/2+k] = get(n-1, k-1) + get(n-1, k);
            }
            coefficients[n*(n+1)/2+n] = 1.0;
        }
    }

    constexpr double get(int n, int k) const noexcept {
        return coefficients[n*(n+1)/2 + k];
    }
};

template <std::size_t P>
Multipole<P> translate_multipole(const Multipole<P>& multipole, Vec2d src) noexcept {
    static constexpr auto binoms = Binomial<P>();
    Multipole<P> result;
    result.q = multipole.q;
    std::array<std::complex<double>, P+1> z_power = {1.0, std::complex<double>(src.x, src.y)};
    for (int k=2; k<P+1; ++k) {
        z_power[k] = z_power[k-1]*z_power[1];
    }
    for (int l=0; l<P; ++l) {
        result.a[l] = -multipole.q*z_power[l+1]/(l+1.0);
        for (int k=0; k<=l; ++k) {
            result.a[l] += multipole.a[k]*z_power[l-k]*binoms.get(l,k);
        }
    }
    return result;
}

template <std::size_t P>
Local<P+1> convert_to_local(const Multipole<P>& multipole, Vec2d src) noexcept {
    static constexpr auto binoms = Binomial<2*P>();
    auto z0 = std::complex<double>(src.x, src.y);
    std::array<std::complex<double>, P+1> z_power = { 1.0/z0 };  // TODO: explodes
    for (int k=1; k<P+1; ++k) {
        z_power[k] = z_power[k-1]*z_power[0];
    }
    Local<P+1> result = { multipole.q*std::log(-z0) };
    for (int k=0; k<P; ++k) {
        auto v0 = multipole.a[k]*z_power[k];
        if ((k&1) == 1) {
            v0 = -v0;
        }
        result[0] += v0;
        for (int l=1; l<P+1; ++l) {
            result[l] += v0*binoms.get(l+k-1, k);
        }
    }
    for (int l=1; l<P+1; ++l) {
        result[l] -= multipole.q/static_cast<double>(l);
        result[l] *= z_power[l-1];
    }
    return result;
}

template <std::size_t P>
Local<P> translate_local(const Local<P>& local, Vec2d src) noexcept {
    auto result = local;
    auto z0 = std::complex<double>(src.x, src.y);
    for (int j=0; j<P-1; ++j) {
        for (int k=P-j-1; k<P-1; ++k) {
            result[k] -= z0*result[k+1];
        }
    }
    return result;
}
