#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Vec2.hpp"


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
    static constexpr std::uint64_t operator()(const Vec2i& v) noexcept {
        return interleave_bits(v.x, v.y);
    }

    static constexpr std::uint64_t operator()(const Vec2d& v) noexcept {
        return interleave_bits(static_cast<std::uint32_t>(std::ldexp(v.x, 32)), static_cast<std::uint32_t>(std::ldexp(v.y, 32)));
    }
};

template<typename T>
struct QuadTree {
    using Level = std::unordered_map<Vec2i, T, InterleaveHash>;
    std::vector<Level> levels;
    static constexpr Vec2i parent_coords(std::size_t level, Vec2i coords) noexcept {
        assert(level > 0);
        assert(level < 33);
        if (level == 1) {
            return Vec2i{};
        }
        auto parent_mask = 0xFFFFFFFFu << (33-level);
        return Vec2i{ coords.x & parent_mask, coords.y & parent_mask };
    }

    static constexpr std::array<Vec2i, 4> children_coords(std::size_t level, Vec2i coords) noexcept {
        assert(level < 32);
        auto child_mask = 1u << (31-level);
        return {
            coords,
            Vec2i{coords.x | child_mask, coords.y},
            Vec2i{coords.x, coords.y | child_mask},
            Vec2i{coords.x | child_mask, coords.y | child_mask},
        };
    }

    static constexpr std::array<Vec2i, 3> siblings_coords(std::size_t level, Vec2i coords) noexcept {
        assert(level > 0);
        assert(level < 32);
        auto sibling_mask = 1u << (32-level);
        return {
            Vec2i{coords.x ^ sibling_mask, coords.y},
            Vec2i{coords.x, coords.y ^ sibling_mask},
            Vec2i{coords.x ^ sibling_mask, coords.y ^ sibling_mask},
        };
    }

    struct NeighbourCoords {
        std::array<Vec2i, 8> data{};
        std::uint8_t count = 0;

        constexpr const Vec2i* begin() const noexcept { return data.data(); }
        constexpr const Vec2i* end() const noexcept { return data.data() + count; }
        constexpr std::size_t size() const noexcept { return count; }
        constexpr bool empty() const noexcept { return count == 0; }
    };

    static constexpr NeighbourCoords neighbours_coords(std::size_t level, Vec2i coords) {
        NeighbourCoords result;
        if (level == 0) {
            return result;
        }

        const auto sibling_mask = 1u << (32 - level);
        const auto overflow = std::numeric_limits<uint32_t>::max() - sibling_mask;

        const bool sub_x = coords.x >= sibling_mask;
        const bool sub_y = coords.y >= sibling_mask;
        const bool add_x = coords.x <= overflow;
        const bool add_y = coords.y <= overflow;

        auto push = [&](std::uint32_t x, std::uint32_t y) {
            result.data[result.count++] = Vec2i{ x, y };
        };

        if (sub_x && sub_y) push(coords.x - sibling_mask, coords.y - sibling_mask);
        if (sub_y)          push(coords.x, coords.y - sibling_mask);
        if (add_x && sub_y) push(coords.x + sibling_mask, coords.y - sibling_mask);
        if (sub_x)          push(coords.x - sibling_mask, coords.y);
        if (add_x)          push(coords.x + sibling_mask, coords.y);
        if (sub_x && add_y) push(coords.x - sibling_mask, coords.y + sibling_mask);
        if (add_y)          push(coords.x, coords.y + sibling_mask);
        if (add_x && add_y) push(coords.x + sibling_mask, coords.y + sibling_mask);

        return result;
    }

    static constexpr bool is_parent(Vec2i coords, std::size_t parent_level, Vec2i parent) {
        if (parent_level == 0) {
            return true;
        }
        auto parent_mask = 0xFFFFFFFFu << (32-parent_level);
        return (coords.x & parent_mask) == parent.x and (coords.y & parent_mask) == parent.y;
    }

    static constexpr bool is_adjacent(std::size_t a_level, Vec2i a, std::size_t b_level, Vec2i b) {
        assert(a_level < 32);
        assert(b_level <= a_level);
        std::int64_t a_size = 1u << (32-a_level); // cast to int64 so we can't overflow
        std::int64_t b_size = 1u << (32-b_level);

        return b.x <= a.x + a_size and b.y <= a.y + a_size and a.x <= b.x + b_size and a.y <= b.y + b_size;
    }

    bool has_children(std::size_t level, Vec2i coords) const {
        if (level == levels.size()-1) {
            return false;
        }
        for (auto& child_cell_coord: children_coords(level, coords)) {
            if (levels[level+1].contains(child_cell_coord)) {
                return true;
            }
        }
        return false;
    }

    auto find_childless_parent(std::size_t level, Vec2i coords) const {
        for (auto parent_level = level-1; parent_level > 0; --parent_level) {
            coords = parent_coords(parent_level+1, coords);
            auto parent = levels[parent_level].find(coords);
            if (parent == levels[parent_level].end()) {
                continue;
            }
            if (has_children(parent_level, coords)) {
                break;
            }
            return std::make_pair(parent_level, parent);
        }
        return std::make_pair(std::size_t{ 0 }, levels[0].end());
    }
};
