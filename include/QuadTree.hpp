#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <span>
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
struct QuadTree2 {
    struct Cell {
        T value;
        Vec2i coords;
        std::uint32_t parent;
        std::uint32_t children;
        std::uint8_t children_count;
        std::uint8_t level;
    };
    std::vector<Cell> cells;

    QuadTree2(T value) {
        cells.emplace_back(value, Vec2i{0,0}, 0, 1, 0, 0);
    }

    Cell& add_cell(T value, Vec2i coords, Cell& parent) {
        assert(is_parent(coords, parent.level, parent.coords));
        assert(parent.children_count == 0 || parent.children+parent.children_count == cells.size());
        auto parent_idx = &parent - cells.data();
        cells.emplace_back(value, coords, parent_idx, 0, 0, parent.level+1);
        parent.children_count += 1;
        parent.children = cells.size() - parent.children_count;
        return cells.back();
    }

    constexpr std::span<const Cell> children(const Cell& cell) const noexcept {
        return std::span<const Cell>(cells).subspan(cell.children, cell.children_count);
    }

    constexpr const Cell& parent(const Cell& cell) const noexcept {
        return cells[cell.parent];
    }

    struct Neighbours {
        std::array<const Cell*, 8> data{};
        std::uint8_t count = 0;

        constexpr auto begin() const noexcept { return data.data(); }
        constexpr auto end() const noexcept { return data.data() + count; }
        constexpr std::size_t size() const noexcept { return count; }
        constexpr bool empty() const noexcept { return count == 0; }
    };

    constexpr const Cell& find(std::uint8_t level, Vec2i coords, const Cell* cell) const noexcept {
        cell = cell == nullptr ? &cells.front() : cell;
        assert(level > cell->level);

        CHECK_CHILDREN:
        for (const Cell& child: children(*cell)) {
            if (is_parent(coords, child.level, child.coords)) {
                if (level == child.level) {
                    return child;
                }
                cell = &child;
                goto CHECK_CHILDREN;
            }
        }
        return *cell;
    }
    
    constexpr Neighbours siblings(const Cell& cell) const noexcept {
        Neighbours result;        
        if (cell.level == 0) {
            return result;
        }
        for (const Cell& sibling: children(parent(cell))) {
            if (sibling.coords != cell.coords) {
                result.data[result.count++] = &sibling;
            }
        }
        return result;
     }

    constexpr Neighbours neighbours(const Cell& cell) const noexcept {
        Neighbours result = siblings(cell);
        if (cell.level == 0) {
            return result;
        }

        const auto sibling_mask = 1u << (32 - cell.level);
        const auto overflow = std::numeric_limits<uint32_t>::max() - sibling_mask;

        bool has_x_neigbour = false;
        std::uint32_t x_neigbour_coord;
        if ((cell.coords.x & sibling_mask) == 0) {
            if (cell.coords.x >= sibling_mask) {
                x_neigbour_coord = cell.coords.x-sibling_mask;
                has_x_neigbour = true;
            }
        } else {
            if (cell.coords.x <= overflow) {
                x_neigbour_coord = cell.coords.x+sibling_mask;
                has_x_neigbour = true;
            }
        }
        bool has_y_neigbour = false;
        std::uint32_t y_neigbour_coord;
        if ((cell.coords.y & sibling_mask) == 0) {
            if (cell.coords.y >= sibling_mask) {
                y_neigbour_coord = cell.coords.y-sibling_mask;
                has_y_neigbour = true;
            }
        } else {
            if (cell.coords.y <= overflow) {
                y_neigbour_coord = cell.coords.y+sibling_mask;
                has_y_neigbour = true;
            }
        }

        const Cell* xy_cell_ptr = nullptr;
        if (has_x_neigbour and has_y_neigbour) {
            const Cell& xy_cell = find(cell.level, { x_neigbour_coord, y_neigbour_coord }, nullptr);
            if (not is_parent(cell.coords, xy_cell.level, xy_cell.coords)) {
                xy_cell_ptr = &xy_cell;
            }
        }
        if (has_x_neigbour) {
            const Cell* parent_cell = nullptr;
            if (xy_cell_ptr && is_parent({ x_neigbour_coord, cell.coords.y }, xy_cell_ptr->level, xy_cell_ptr->coords)) {
                parent_cell = xy_cell_ptr;
                xy_cell_ptr = nullptr;
            }
            const Cell& x_cell = find(cell.level, { x_neigbour_coord, cell.coords.y }, parent_cell);
            if (not is_parent(cell.coords, x_cell.level, x_cell.coords)) {
                result.data[result.count++] = &x_cell;
                if (x_cell.level == cell.level) {
                    for (const Cell& sibling: children(parent(x_cell))) {
                        if (sibling.coords == Vec2i{ x_neigbour_coord, cell.coords.y ^ sibling_mask }) {
                            result.data[result.count++] = &sibling;
                            break;
                        }
                    }
                }
                if (x_cell.level+1 == cell.level) {
                    for (const Cell& sibling: children(x_cell)) {
                        if (sibling.coords == Vec2i{ x_neigbour_coord, cell.coords.y ^ sibling_mask }) {
                            result.data[result.count-1] = &sibling;
                            break;
                        }
                    }
                }
            }
        }
        if (has_y_neigbour) {
            const Cell* parent_cell = nullptr;
            if (xy_cell_ptr && is_parent({ cell.coords.x, y_neigbour_coord }, xy_cell_ptr->level, xy_cell_ptr->coords)) {
                parent_cell = xy_cell_ptr;
                xy_cell_ptr = nullptr;
            }
            const Cell& y_cell = find(cell.level, { cell.coords.x, y_neigbour_coord }, parent_cell);
            if (not is_parent(cell.coords, y_cell.level, y_cell.coords)) {
                result.data[result.count++] = &y_cell;
                if (y_cell.level == cell.level) {
                    for (const Cell& sibling: children(parent(y_cell))) {
                        if (sibling.coords == Vec2i{ cell.coords.x ^ sibling_mask, y_neigbour_coord }) {
                            result.data[result.count++] = &sibling;
                            break;
                        }
                    }
                }
                if (y_cell.level+1 == cell.level) {
                    for (const Cell& sibling: children(y_cell)) {
                        if (sibling.coords == Vec2i{ cell.coords.x ^ sibling_mask, y_neigbour_coord }) {
                            result.data[result.count-1] = &sibling;
                            break;
                        }
                    }
                }
            }
        }

        if (xy_cell_ptr) {
            result.data[result.count++] = xy_cell_ptr;
        }

        return result;
    }

    static constexpr bool is_parent(Vec2i coords, std::size_t parent_level, Vec2i parent) noexcept {
        if (parent_level == 0) {
            return true;
        }
        auto parent_mask = 0xFFFFFFFFu << (32-parent_level);
        return (coords.x & parent_mask) == parent.x and (coords.y & parent_mask) == parent.y;
    }

    static constexpr bool is_adjacent(std::size_t a_level, Vec2i a, std::size_t b_level, Vec2i b) noexcept {
        // TODO: is this equvalent for cheicking for single bit difference?
        assert(a_level < 32);
        assert(b_level <= a_level);
        auto a_size = std::uint64_t(1) << (32-a_level);  // uint64_t so we don't overflow
        auto b_size = std::uint64_t(1) << (32-b_level);

        return b.x <= a.x + a_size and b.y <= a.y + a_size and a.x <= b.x + b_size and a.y <= b.y + b_size;
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

    static constexpr NeighbourCoords neighbours_coords(std::size_t level, Vec2i coords) noexcept {
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

    static constexpr bool is_parent(Vec2i coords, std::size_t parent_level, Vec2i parent) noexcept {
        if (parent_level == 0) {
            return true;
        }
        auto parent_mask = 0xFFFFFFFFu << (32-parent_level);
        return (coords.x & parent_mask) == parent.x and (coords.y & parent_mask) == parent.y;
    }

    static constexpr bool is_adjacent(std::size_t a_level, Vec2i a, std::size_t b_level, Vec2i b) noexcept {
        assert(a_level < 32);
        assert(b_level <= a_level);
        auto a_size = std::uint64_t(1) << (32-a_level);  // uint64_t so we don't overflow
        auto b_size = std::uint64_t(1) << (32-b_level);

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
            if constexpr (requires { parent->second.flags; }) {
                if ((parent->second.flags & 15) != 0) {
                    break;
                }
            } else {
                if (has_children(parent_level, coords)) {
                    break;
                }
            }
            return std::make_pair(parent_level, parent);
        }
        return std::make_pair(std::size_t{ 0 }, levels[0].end());
    }
};
