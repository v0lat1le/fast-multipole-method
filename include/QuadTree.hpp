#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "glm/vec2.hpp"


constexpr std::uint64_t spread_bits(std::uint32_t x) noexcept {
    std::uint64_t val = x;
    val = (val | (val << 16)) & 0x0000FFFF0000FFFF;
    val = (val | (val << 8))  & 0x00FF00FF00FF00FF;
    val = (val | (val << 4))  & 0x0F0F0F0F0F0F0F0F;
    val = (val | (val << 2))  & 0x3333333333333333;
    val = (val | (val << 1))  & 0x5555555555555555;
    return val;
}

constexpr std::uint64_t interleave_bits(std::uint32_t x, std::uint32_t y) noexcept {
    return spread_bits(x) | (spread_bits(y) << 1);
}

struct InterleaveHash {
    static constexpr std::uint64_t operator()(const glm::uvec2& v) noexcept {
        return interleave_bits(v.x, v.y);
    }

    static constexpr std::uint64_t operator()(const glm::dvec2& v) noexcept {
        return interleave_bits(static_cast<std::uint32_t>(std::ldexp(v.x, 32)), static_cast<std::uint32_t>(std::ldexp(v.y, 32)));
    }
};

template<typename T>
struct QuadTree {
    struct Cell {
        T value;
        glm::uvec2 coords;
        std::uint32_t parent;
        std::uint32_t children;
        std::uint8_t children_count;
        std::uint8_t level;
    };
    std::vector<Cell> cells;

    QuadTree(T value) {
        cells.emplace_back(std::move(value), glm::uvec2{0, 0}, 0u, 1u, std::uint8_t{0}, std::uint8_t{0});
    }

    std::uint32_t add_cell(T value, glm::uvec2 coords, std::uint32_t parent_idx) {
        auto& parent = cells[parent_idx];
        assert(parent.level < 31);
        assert(is_parent(coords, parent.level, parent.coords));
        assert(parent.children_count == 0 || parent.children+parent.children_count == cells.size());
        cells.emplace_back(std::move(value), coords, parent_idx, 0u, std::uint8_t{0}, static_cast<std::uint8_t>(parent.level+1));
        parent.children_count += 1;
        parent.children = static_cast<std::uint32_t>(cells.size()) - parent.children_count;
        return cells.size()-1;
    }

    static constexpr bool is_parent(glm::uvec2 coords, std::size_t parent_level, glm::uvec2 parent) noexcept {
        if (parent_level == 0) {
            return true;
        }
        auto parent_mask = 0xFFFFFFFFu << (32-parent_level);
        return (coords.x & parent_mask) == parent.x and (coords.y & parent_mask) == parent.y;
    }

    static constexpr bool is_adjacent(std::size_t a_level, glm::uvec2 a, std::size_t b_level, glm::uvec2 b) noexcept {
        assert(a_level < 32);
        assert(b_level < 32);
        auto a_size = std::uint64_t(1) << (32-a_level);  // uint64_t so we don't overflow
        auto b_size = std::uint64_t(1) << (32-b_level);

        return b.x <= a.x + a_size & b.y <= a.y + a_size & a.x <= b.x + b_size & a.y <= b.y + b_size;
    }
};
