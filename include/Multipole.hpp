#pragma once

#include <array>
#include <complex>

#include "glm/vec2.hpp"


template <std::size_t P>
struct Multipole {
    double q;
    std::array<std::complex<double>, P> a;
};

template <std::size_t P>
constexpr void calculate_multipole(double charge, Multipole<P>& dst, glm::dvec2 dr) noexcept {
    double z_real = dr.x;
    double z_imag = dr.y;
    double z_power_real = charge;
    double z_power_imag = 0.0;
    for (std::size_t k=0; k<P; ++k) {
        double z_power_real_next = z_power_real*z_real - z_power_imag*z_imag;
        z_power_imag = z_power_real*z_imag + z_power_imag*z_real;
        z_power_real = z_power_real_next;
        auto& o = reinterpret_cast<double(&)[2]>(dst.a[k]);
        o[0] -= z_power_real/(k+1.0);
        o[1] -= z_power_imag/(k+1.0);
    }
    dst.q += charge;
}

template <std::size_t P>
constexpr glm::dvec2 evaluate_multipole(const Multipole<P>& multipole, glm::dvec2 dr) noexcept {
    double d = dr.x*dr.x + dr.y*dr.y;
    double inv_real = dr.x/d;
    double inv_imag = -dr.y/d;
    double accel_real = 0.0;
    double accel_imag = 0.0;
    for (std::size_t k=P; k-->0;) {
        double tmp_real = accel_real - (k+1.0)*multipole.a[k].real();
        double tmp_imag = accel_imag - (k+1.0)*multipole.a[k].imag();
        accel_real = tmp_real*inv_real - tmp_imag*inv_imag;
        accel_imag = tmp_real*inv_imag + tmp_imag*inv_real;
    }
    double tmp_real = multipole.q + accel_real;
    double tmp_imag = accel_imag;
    accel_real = tmp_imag*inv_imag - tmp_real*inv_real;
    accel_imag = tmp_real*inv_imag + tmp_imag*inv_real;
    return glm::dvec2{ accel_real, accel_imag };
}

template <std::size_t P>
using Local = std::array<std::complex<double>, P>;

template <std::size_t P>
constexpr glm::dvec2 evaluate_local(const Local<P>& local, glm::dvec2 dr) noexcept {
    double z_real = dr.x;
    double z_imag = dr.y;
    double accel_real = (P+1.0)*local[P-1].real();
    double accel_imag = (P+1.0)*local[P-1].imag();
    for (std::size_t l=P-1; l-->1;) {
        double tmp_real = accel_real + (l+1.0)*local[l].real();
        double tmp_imag = accel_imag + (l+1.0)*local[l].imag();
        accel_real = tmp_real*z_real - tmp_imag*z_imag;
        accel_imag = tmp_real*z_imag + tmp_imag*z_real;
    }
    return glm::dvec2{-accel_real-local[0].real(), accel_imag + local[0].imag()};
}

template <std::size_t P>
struct Binomial {
    std::array<double, P*(P+1)/2> coefficients;

    constexpr Binomial() noexcept {
        coefficients[0] = 1.0;
        for (std::size_t n=1; n<P; ++n) {
            coefficients[n*(n+1)/2] = 1.0;
            for (std::size_t k=1; k<n; ++k) {
                coefficients[n*(n+1)/2+k] = operator()(n-1, k-1) + operator()(n-1, k);
            }
            coefficients[n*(n+1)/2+n] = 1.0;
        }
    }

    constexpr double operator()(std::size_t n, std::size_t k) const noexcept {
        return coefficients[n*(n+1)/2 + k];
    }
};

template <std::size_t P>
constexpr void translate_multipole(const Multipole<P>& multipole, Multipole<P>& dst, glm::dvec2 dr) noexcept {
    static constexpr auto binoms = Binomial<P>();
    double z_power_real[P+1] = {1.0, dr.x};
    double z_power_imag[P+1] = {0.0, dr.y};
    for (std::size_t k=2; k<P+1; ++k) {
        z_power_real[k] = z_power_real[k-1]*z_power_real[1] - z_power_imag[k-1]*z_power_imag[1];
        z_power_imag[k] = z_power_real[k-1]*z_power_imag[1] + z_power_imag[k-1]*z_power_real[1];
    }
    for (std::size_t l=0; l<P; ++l) {  // TODO: try loops other way
        auto& o = reinterpret_cast<double(&)[2]>(dst.a[l]);
        double v = -multipole.q/(l+1.0);
        o[0] += v*z_power_real[l+1];
        o[1] += v*z_power_imag[l+1];
        for (std::size_t k=0; k<=l; ++k) {
            double a_real = multipole.a[k].real();
            double a_imag = multipole.a[k].imag();
            o[0] += (a_real*z_power_real[l-k] - a_imag*z_power_imag[l-k])*binoms(l, k);
            o[1] += (a_real*z_power_imag[l-k] + a_imag*z_power_real[l-k])*binoms(l, k);
        }
    }
    dst.q += multipole.q;
}

template <std::size_t P>
struct M2L {
    double coefficients[P][P];

    constexpr M2L() {
        Binomial<2*P> binoms;
        for (std::size_t k=0; k<P; ++k) {
            for (std::size_t l=0; l<P; ++l) {
                coefficients[k][l] = ((k&1) ? 1.0 : -1.0)*binoms(l+1+k, k);
            }
        }
    }

    constexpr double operator()(std::size_t k, std::size_t l) const noexcept {
        return coefficients[k][l];
    }
};

template <std::size_t P>
constexpr void convert_to_local(const Multipole<P>& multipole, Local<P>& local, const double z_power_real[P], const double z_power_imag[P]) noexcept {
    static constexpr auto coefficients = M2L<P>();
    // not computing or storing b0 as it doesn't contribute to the force
    double bs_real[P] = {};
    double bs_imag[P] = {};
    for (std::size_t k=0; k<P; ++k) {
        double a_real = multipole.a[k].real();
        double a_imag = multipole.a[k].imag();
        double az_real = a_real*z_power_real[k] - a_imag*z_power_imag[k];
        double az_imag = a_real*z_power_imag[k] + a_imag*z_power_real[k];
        for (std::size_t l=0; l<P; ++l) {
            bs_real[l] += az_real*coefficients(k, l);
            bs_imag[l] += az_imag*coefficients(k, l);
        }
    }
    for (std::size_t l=0; l<P; ++l) {
        double f_real = bs_real[l] - multipole.q/(l+1.0);
        double f_imag = bs_imag[l];
        auto& o = reinterpret_cast<double(&)[2]>(local[l]);
        o[0] += f_real*z_power_real[l] - f_imag*z_power_imag[l];
        o[1] += f_real*z_power_imag[l] + f_imag*z_power_real[l];
    }
}

template <std::size_t P>
constexpr void convert_to_local(const Multipole<P>& multipole, Local<P>& local, glm::dvec2 dr) noexcept {
    double d = dr.x*dr.x + dr.y*dr.y;
    double inv_real = dr.x/d;
    double inv_imag = -dr.y/d;
    double z_power_real[P] = { inv_real };
    double z_power_imag[P] = { inv_imag };
    for (std::size_t k=1; k<P; ++k) {
        z_power_real[k] = z_power_real[k-1]*inv_real - z_power_imag[k-1]*inv_imag;
        z_power_imag[k] = z_power_real[k-1]*inv_imag + z_power_imag[k-1]*inv_real;
    }
    convert_to_local(multipole, local, z_power_real, z_power_imag);
}

template <std::size_t P>
constexpr void charge_to_local(double q, Local<P>& local, glm::dvec2 dr) noexcept {
    double d = dr.x*dr.x + dr.y*dr.y;
    double inv_real = dr.x/d;
    double inv_imag = -dr.y/d;
    double z_power_real = inv_real;
    double z_power_imag = inv_imag;
    for (std::size_t l=0; l<P; ++l) {
        double f_real = -q/(l+1.0);
        auto& o = reinterpret_cast<double(&)[2]>(local[l]);
        o[0] += f_real*z_power_real;
        o[1] += f_real*z_power_imag;
        double z_power_real_next = z_power_real*inv_real - z_power_imag*inv_imag; // TODO: unnecessary step on the last iter
        z_power_imag = z_power_real*inv_imag + z_power_imag*inv_real;
        z_power_real = z_power_real_next;
    }
}

template <std::size_t P>
constexpr void translate_local(const Local<P>& src, Local<P>& dst, glm::dvec2 dr) noexcept {
    double z0_real = dr.x;
    double z0_imag = dr.y;
    auto tmp = src;
    for (std::size_t j=0; j<P-1; ++j) {
        for (std::size_t k=P-j-2; k<P-1; ++k) {
            auto& o = reinterpret_cast<double(&)[2]>(tmp[k]);
            o[0] -= z0_real*tmp[k+1].real() - z0_imag*tmp[k+1].imag();
            o[1] -= z0_real*tmp[k+1].imag() + z0_imag*tmp[k+1].real();
        }
    }
    for (std::size_t k=0; k<P-1; ++k) {  // one extra round as we're not storing b0
        auto& o = reinterpret_cast<double(&)[2]>(tmp[k]);
        o[0] -= z0_real*tmp[k+1].real() - z0_imag*tmp[k+1].imag();
        o[1] -= z0_real*tmp[k+1].imag() + z0_imag*tmp[k+1].real();
        auto& o2 = reinterpret_cast<double(&)[2]>(dst[k]);
        o2[0] += o[0];
        o2[1] += o[1];
    }
    dst[P-1] += tmp[P-1];
}
