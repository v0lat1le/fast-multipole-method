#pragma once

#include <algorithm>
#include <bit>
#include <functional>
#include <future>
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

template<typename Range, typename Proj>
QuadTree<std::pair<std::uint32_t, std::uint32_t>> build_quadtree(Range points, Proj proj, std::uint32_t max_points=1, std::uint8_t max_levels=31) {
    QuadTree<std::pair<std::uint32_t, std::uint32_t>> quadtree({ 0u, static_cast<std::uint32_t>(points.size()) });
    quadtree.cells.reserve(2*points.size()/max_points);

    for (std::uint32_t idx = 0; idx < quadtree.cells.size(); ++idx) {
        if (quadtree.cells.capacity() < quadtree.cells.size()+4) {  // avoid reallocation when adding items in the loop
            quadtree.cells.reserve(static_cast<std::size_t>(quadtree.cells.capacity()*1.5)+4);
        }
        auto& cell = quadtree.cells[idx];
        if (cell.value.second-cell.value.first <= max_points || cell.level == max_levels) {
            continue;
        }

        auto child_cell_size = 1u << (31-cell.level);
        const auto child_cells_coords = {
            cell.coords + glm::uvec2{child_cell_size, 0},
            cell.coords + glm::uvec2{0, child_cell_size},
            cell.coords + glm::uvec2{child_cell_size, child_cell_size},
        };
        auto prev_child_cell_coords = cell.coords;
        auto begin = cell.value.first;
        for (const auto& child_cell_coords: child_cells_coords) {
            auto pivot_coords = interleave_bits(child_cell_coords.x, child_cell_coords.y);
            auto tail = std::ranges::partition(points.begin()+begin, points.begin()+cell.value.second, [pivot_coords](std::uint64_t v){ return v < pivot_coords; }, proj);
            auto end = static_cast<std::uint32_t>(tail.begin()-points.begin());
            if (end != begin) {
                assert(begin < end);
                quadtree.add_cell({ begin, end }, prev_child_cell_coords, idx);
                begin = end;
            }
            prev_child_cell_coords = child_cell_coords;
        }
        if (begin != cell.value.second) {
            assert(begin < cell.value.second);
            quadtree.add_cell({ begin, cell.value.second }, prev_child_cell_coords, idx);
        }
    }

    return quadtree;
}

void compute_acceleration_direct(std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double eps=0.0);
void compute_acceleration_direct(std::span<const glm::dvec2> src_pos, std::span<const double> src_mass, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc, double eps=0.0);

void compute_acceleration_multipoles(const QuadTree<std::pair<std::uint32_t, std::uint32_t>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double direct_eps, std::function<std::future<void>(std::function<void()>)> submit_task);
