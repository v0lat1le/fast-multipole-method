#pragma once

#include <algorithm>
#include <span>

#include "glm/vec2.hpp"
#include "poolstl/poolstl.hpp"

#include "QuadTree.hpp"


template<typename Range, typename Proj>
QuadTree<std::pair<std::uint32_t, std::uint32_t>> build_quadtree(Range points, Proj proj, std::uint32_t max_points=1, std::uint8_t max_levels=31) {
    QuadTree<std::pair<std::uint32_t, std::uint32_t>> quadtree({ 0u, static_cast<std::uint32_t>(points.size()) });
    quadtree.cells.reserve(2*points.size()/max_points);

    std::atomic<std::uint32_t> children_start(1);

    auto process_cell = [&](uint32_t cell_idx, bool parallel) {
        auto& cell = quadtree.cells[cell_idx];
        auto begin_idx = cell.value.first;
        auto end_idx = cell.value.second;
        if (end_idx - begin_idx <= max_points || cell.level == max_levels) {
            return;
        }

        auto child_cell_size = 1u << (31-cell.level);
        const glm::uvec2 child_cells_coords[] = {
            cell.coords,
            cell.coords + glm::uvec2{child_cell_size, 0},
            cell.coords + glm::uvec2{0, child_cell_size},
            cell.coords + glm::uvec2{child_cell_size, child_cell_size},
        };
#ifdef _MSC_VER
#define parallel_partition(...) std::partition(poolstl::par_if(parallel), __VA_ARGS__)
#else  // std::partition doesn't like std::view::zip on gcc/clang
#define parallel_partition(...) std::ranges::partition(__VA_ARGS__).begin()
#endif
        auto half_2 = parallel_partition(points.begin()+begin_idx, points.begin()+end_idx, [pivot_coords=morton_code(child_cells_coords[2]), &proj](const auto& v) { return proj(v) < pivot_coords; });
        auto quarter_2 = parallel_partition(points.begin()+begin_idx, half_2, [pivot_coords=morton_code(child_cells_coords[1]), &proj](const auto& v) { return proj(v) < pivot_coords; });
        auto quarter_4 = parallel_partition(half_2, points.begin()+end_idx, [pivot_coords=morton_code(child_cells_coords[3]), &proj](const auto& v) { return proj(v) < pivot_coords; });
#undef parallel_partition
        auto half_2_idx = static_cast<std::uint32_t>(half_2-points.begin());
        auto quarter_2_idx = static_cast<std::uint32_t>(quarter_2-points.begin());
        auto quarter_4_idx = static_cast<std::uint32_t>(quarter_4-points.begin());

        cell.children_count = (begin_idx != quarter_2_idx) + (quarter_2_idx != half_2_idx) + (half_2_idx != quarter_4_idx) + (quarter_4_idx != end_idx);
        cell.children = children_start.fetch_add(cell.children_count, std::memory_order_relaxed);
        auto child_idx = cell.children;
        if (begin_idx != quarter_2_idx) {
            quadtree.cells[child_idx++] = { { begin_idx, quarter_2_idx }, child_cells_coords[0], cell_idx, 0u, std::uint8_t{ 0 }, static_cast<std::uint8_t>(cell.level+1) };
        }
        if (quarter_2_idx != half_2_idx) {
            quadtree.cells[child_idx++] = { { quarter_2_idx, half_2_idx }, child_cells_coords[1], cell_idx, 0u, std::uint8_t{ 0 }, static_cast<std::uint8_t>(cell.level+1) };
        }
        if (half_2_idx != quarter_4_idx) {
            quadtree.cells[child_idx++] = { { half_2_idx, quarter_4_idx }, child_cells_coords[2], cell_idx, 0u, std::uint8_t{ 0 }, static_cast<std::uint8_t>(cell.level+1) };
        }
        if (quarter_4_idx != end_idx) {
            quadtree.cells[child_idx++] = { { quarter_4_idx, end_idx }, child_cells_coords[3], cell_idx, 0u, std::uint8_t{ 0 }, static_cast<std::uint8_t>(cell.level+1) };
        }
    };

    std::uint32_t level_start = 0;
    std::uint32_t level_end = 1;
    while(level_start < level_end) {
        quadtree.cells.resize(children_start + (level_end-level_start)*4);
        auto range = std::views::iota(level_start, level_end);
        bool parallel_partition = range.size()<5;
        std::for_each(poolstl::par_if(range.size() > 1), range.begin(), range.end(), [&process_cell, parallel_partition](std::uint32_t cell_idx) { process_cell(cell_idx, parallel_partition); });
        level_start = level_end;
        level_end = children_start;
    }
    quadtree.cells.resize(children_start);

    return quadtree;
}

void compute_acceleration_direct(std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double eps=0.0);
void compute_acceleration_direct(std::span<const glm::dvec2> src_pos, std::span<const double> src_mass, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc, double eps=0.0);

void compute_acceleration_multipoles(const QuadTree<std::pair<std::uint32_t, std::uint32_t>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double direct_eps=0.0);
