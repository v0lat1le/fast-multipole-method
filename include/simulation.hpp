#pragma once

#include <bit>
#include <span>

#include "Vec2.hpp"
#include "QuadTree.hpp"


constexpr bool cmp_zcurve_interleave(const Vec2i& lhs, const Vec2i& rhs) noexcept {
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

//constexpr bool cmp_zcurve_bitmagic(const Vec2i& lhs, const Vec2i& rhs) noexcept {
//   auto msb_x = lhs.x ^ rhs.x;
//    auto msb_y = lhs.y ^ rhs.y;
//    if (msb_y < msb_x and msb_y < (msb_x ^ msb_y)) {
//        return lhs.x < rhs.x;
//    } else {
//        return lhs.y < rhs.y;
//    }
//}

template<typename T>
constexpr bool cmp_zcurve_bitmagic(const Vec2<T>& lhs, const Vec2<T>& rhs) noexcept {
    if (msb_diff(lhs.y, rhs.y) < msb_diff(lhs.x, rhs.x)) {
        return lhs.x < rhs.x;
    } else {
        return lhs.y < rhs.y;
    }
}

struct Cell {
    std::span<const Vec2d> points;
    std::uint8_t flags;
};

using Cells = QuadTree<Cell>;

Cells build_quadtree(std::span<const Vec2d> points, int max_levels, int max_points=1);
QuadTree2<std::span<const Vec2d>> build_quadtree2(std::span<const Vec2d> points, int max_levels, int max_points=1);

void compute_acceleration_direct(std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations);
void compute_acceleration_direct(std::span<const Vec2d> src_pos, std::span<const double> src_mass, std::span<const Vec2d> dst_pos, std::span<Vec2d> dst_acc);

void compute_acceleration_multipoles(const Cells& cells, std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations);
void compute_acceleration_multipoles2(const QuadTree2<std::span<const Vec2d>>& cells, std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations);
