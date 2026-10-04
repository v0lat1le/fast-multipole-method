#pragma once

#include <bit>
#include <span>

#include "glm/vec2.hpp"

#include "QuadTree.hpp"


constexpr bool cmp_zcurve_interleave(const glm::uvec2& lhs, const glm::uvec2& rhs) noexcept {
    return interleave_bits(lhs.x, lhs.y) < interleave_bits(rhs.x, rhs.y);
}

template <typename T>
constexpr int msb_diff(T a, T b) noexcept {
    if constexpr (std::is_unsigned_v<T>) {
        return std::numeric_limits<T>::digits-1-std::countl_zero(a^b);
    }
    if constexpr (std::is_same_v<T, float> or std::is_same_v<T, double>) {
        using FloatAsUInt = std::conditional_t<std::is_same_v<T, float>, std::uint32_t, uint64_t>;
        constexpr auto mantissa_size = std::numeric_limits<T>::digits-1;
        constexpr auto mantissa_mask = (FloatAsUInt(1)<<mantissa_size)-1;
        auto a_bits = std::bit_cast<FloatAsUInt>(a);
        auto b_bits = std::bit_cast<FloatAsUInt>(b);
        auto a_exponent = a_bits >> mantissa_size;
        auto b_exponent = b_bits >> mantissa_size;

        if (a_exponent == b_exponent) {
            return static_cast<int>(a_exponent + msb_diff(a_bits & mantissa_mask, b_bits & mantissa_mask) - mantissa_size);
        } else if (b_exponent < a_exponent) {
            return static_cast<int>(a_exponent);
        } else {
            return static_cast<int>(b_exponent);
        }
    }
}

template<typename T>
constexpr bool cmp_zcurve_bitmagic(const glm::vec<2, T, glm::defaultp>& lhs, const glm::vec<2, T, glm::defaultp>& rhs) noexcept {
    if (msb_diff(lhs.y, rhs.y) < msb_diff(lhs.x, rhs.x)) {
        return lhs.x < rhs.x;
    } else {
        return lhs.y < rhs.y;
    }
}

QuadTree<std::span<const glm::dvec2>> build_quadtree(std::span<const glm::dvec2> points, int max_points=1, int max_levels=31);

void compute_acceleration_direct(std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations);
void compute_acceleration_direct(std::span<const glm::dvec2> src_pos, std::span<const double> src_mass, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc);

void compute_acceleration_multipoles(const QuadTree<std::span<const glm::dvec2>>& cells, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations);
