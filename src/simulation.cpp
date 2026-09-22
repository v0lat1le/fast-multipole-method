#include <algorithm>
#include <complex>
#include <cmath>

#include "Vec2.hpp"
#include "QuadTree.hpp"
#include "simulation.hpp"


QuadTree2<std::span<const Vec2d>> build_quadtree2(std::span<const Vec2d> points, int max_levels, int max_points) {
    QuadTree2<std::span<const Vec2d>> cells(points);

    for (std::size_t idx = 0; idx < cells.cells.size(); ++idx) {
        cells.cells.reserve(cells.cells.size()+4);  // HACK: to avoid ref invalidation when adding items
        auto& parent = cells.cells[idx];
        if (parent.value.size() <= max_points || parent.level == max_levels) {
            continue;
        }

        auto begin = parent.value.begin();
        auto child_cell_size = 1u << (31-parent.level);
        const auto child_cells_coords = {
            parent.coords + Vec2i{child_cell_size, 0},
            parent.coords + Vec2i{0, child_cell_size},
            parent.coords + Vec2i{child_cell_size, child_cell_size},
        };
        std::uint8_t child_flag = 1;
        auto prev_child_cell_coords = parent.coords;
        for (const auto& child_cell_coords: child_cells_coords) {
            auto double_space_coords = Vec2d{ std::ldexp(child_cell_coords.x, -32), std::ldexp(child_cell_coords.y, -32) };
            auto end = std::upper_bound(begin, parent.value.end(), double_space_coords, static_cast<bool(*)(const Vec2d&, const Vec2d&)>(cmp_zcurve_bitmagic));
            if (begin != end) {
                cells.add_cell({ begin, end }, prev_child_cell_coords, parent);
                begin = end;
            }
            prev_child_cell_coords = child_cell_coords;
            child_flag <<= 1;
        }
        if (begin != parent.value.end()) {
            cells.add_cell({ begin, parent.value.end() }, prev_child_cell_coords, parent);
        }
    }

    return cells;
}

Cells build_quadtree(std::span<const Vec2d> points, int max_levels, int max_points) {
    Cells cells;
    cells.levels.reserve(max_levels); // HACK: to prevent iterator invalidation in the loop below
    cells.levels.push_back({});
    cells.levels[0].emplace(std::make_pair(Vec2i{ 0, 0 }, Cell{ points }));
    for (int level = 0; level < max_levels-1 && !cells.levels[level].empty(); ++level) {
        cells.levels.push_back({});
        for (auto& cell: cells.levels[level]) {
            if (cell.second.points.size() <= max_points) {
                continue;
            }

            auto begin = cell.second.points.begin();
            auto child_cell_size = 1u << (31-level);
            const auto child_cells_coords = {
                cell.first + Vec2i{child_cell_size, 0},
                cell.first + Vec2i{0, child_cell_size},
                cell.first + Vec2i{child_cell_size, child_cell_size},
            };
            std::uint8_t child_flag = 1;
            auto prev_child_cell_coords = cell.first;
            for (const auto& child_cell_coords: child_cells_coords) {
                auto double_space_coords = Vec2d{ std::ldexp(child_cell_coords.x, -32), std::ldexp(child_cell_coords.y, -32) };
                auto end = std::upper_bound(begin, cell.second.points.end(), double_space_coords, static_cast<bool(*)(const Vec2d&, const Vec2d&)>(cmp_zcurve_bitmagic));
                if (begin != end) {
                    cells.levels[level+1].emplace(std::make_pair(prev_child_cell_coords, Cell{ {begin, end} }));
                    cell.second.flags |= child_flag;
                    begin = end;
                }
                prev_child_cell_coords = child_cell_coords;
                child_flag <<= 1;
            }
            if (begin != cell.second.points.end()) {
                cells.levels[level+1].emplace(std::make_pair(prev_child_cell_coords, Cell{ {begin, cell.second.points.end()} }));
                cell.second.flags |= child_flag;
            }
        }
    }

    return cells;
}

void compute_acceleration_direct(std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations) {
    for (std::size_t i=0; i<positions.size(); i++) {
        for (std::size_t j=i+1; j<positions.size(); j++) {
            auto dr = positions[j] - positions[i];
            double d2 = dr.x*dr.x + dr.y*dr.y + 1e-14;
            accelerations[i] += dr*masses[j]/d2;
            accelerations[j] -= dr*masses[i]/d2;
        }
    }
}

void compute_acceleration_direct(std::span<const Vec2d> src_pos, std::span<const double> src_mass, std::span<const Vec2d> dst_pos, std::span<Vec2d> dst_acc) {
    for (std::size_t i=0; i<src_pos.size(); i++) {
        for (std::size_t j=0; j<dst_pos.size(); j++) {
            auto dr = dst_pos[j] - src_pos[i];
            double d2 = dr.x*dr.x + dr.y*dr.y + 1e-14;
            dst_acc[j] -= dr*src_mass[i]/d2;
        }
    }
}

template <std::size_t P>
struct Multipole {
    double q;
    std::array<std::complex<double>, P> a;
    std::uint8_t flags;
};

template <std::size_t P>
QuadTree<Multipole<P>> compute_multipoles(const Cells& cells, std::span<const Vec2d> positions, std::span<const double> masses) {
    auto multipoles = QuadTree<Multipole<P>>();
    multipoles.levels.resize(cells.levels.size());
    for (std::size_t level=cells.levels.size(); level-- > 0;) {
        for (auto& cell: cells.levels[level]) {
            auto child_mask = 1u << (31-level);
            auto cell_center = Vec2d(std::ldexp(cell.first.x | child_mask, -32), std::ldexp(cell.first.y | child_mask, -32));
            auto multipole = multipoles.levels[level].emplace(std::make_pair(cell.first, Multipole<P>{}));
            multipole.first->second.flags = cell.second.flags;
            std::ptrdiff_t offset = cell.second.points.data() - positions.data();
            for (int i=0; i<cell.second.points.size(); ++i) {
                auto mass = masses[offset+i];
                multipole.first->second.q += mass;
                auto relative_coords = cell.second.points[i]-cell_center;
                auto z = std::complex(relative_coords.x, relative_coords.y);
                auto z_power = std::complex(mass);
                for (int k=0; k<multipole.first->second.a.size(); ++k) {
                    z_power *= z;
                    multipole.first->second.a[k] -= z_power/(k+1.0);
                }
            }
        }
    }
    return multipoles;
}

template <std::size_t P>
QuadTree2<Multipole<P>> compute_multipoles2(const QuadTree2<std::span<const Vec2d>>& cells, std::span<const Vec2d> positions, std::span<const double> masses) {
    auto multipoles = QuadTree2<Multipole<P>>({});
    multipoles.cells.resize(cells.cells.size());
    for (std::size_t idx=0; idx < multipoles.cells.size(); ++idx) {
        auto& cell = cells.cells[idx];
        auto& multipole = multipoles.cells[idx];
        multipole.coords = cell.coords;
        multipole.parent = cell.parent;
        multipole.children = cell.children;
        multipole.children_count = cell.children_count;
        multipole.level = cell.level;

        auto child_mask = 1u << (31-cell.level);
        auto cell_center = Vec2d(std::ldexp(cell.coords.x | child_mask, -32), std::ldexp(cell.coords.y | child_mask, -32));
        std::ptrdiff_t offset = cell.value.data() - positions.data();
        for (int i=0; i<cell.value.size(); ++i) {
            auto mass = masses[offset+i];
            multipole.value.q += mass;
            auto relative_coords = cell.value[i]-cell_center;
            auto z = std::complex(relative_coords.x, relative_coords.y);
            auto z_power = std::complex(mass);
            for (int k=0; k<multipole.value.a.size(); ++k) {
                z_power *= z;
                multipole.value.a[k] -= z_power/(k+1.0);
            }
        }
    }
    return multipoles;
}

template<std::size_t P>
void compute_acceleration_multipole(Vec2d src_pos, const Multipole<P>& multipole, std::span<const Vec2d> dst_pos, std::span<Vec2d> dst_acc) {
    for (std::size_t j=0; j<dst_pos.size(); j++) {
        auto dr = dst_pos[j] - src_pos;
        auto z_inv = 1.0/std::complex(dr.x, dr.y);
        auto accel = multipole.q*z_inv;
        auto z_power = z_inv;
        for (int k=0; k<multipole.a.size(); ++k) {
            z_power *= z_inv;
            accel -= (k+1.0)*multipole.a[k]*z_power;
        }
        dst_acc[j] -= Vec2d(accel.real(), -accel.imag());
    }
}

void compute_acceleration_multipoles(const Cells& cells, std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations) {
    auto multipoles = compute_multipoles<8>(cells, positions, masses);

    for (std::size_t level=1; level<multipoles.levels.size(); ++level) {  // skipping level 0 to avoid buncha level==0 checks
        for (auto& cell: cells.levels[level]) {
            const auto has_children = (cell.second.flags & 15);
            if (not has_children) {
                std::ptrdiff_t src_offset = cell.second.points.data() - positions.data();
                compute_acceleration_direct(cell.second.points, masses.subspan(src_offset, cell.second.points.size()), accelerations.subspan(src_offset, cell.second.points.size()));

                for (auto coords: multipoles.siblings_coords(level, cell.first)) {
                    auto sibling = cells.levels[level].find(coords);
                    if (sibling != cells.levels[level].end()) {
                        std::ptrdiff_t dst_offset = sibling->second.points.data() - positions.data();
                        compute_acceleration_direct(cell.second.points, masses.subspan(src_offset, cell.second.points.size()), sibling->second.points, accelerations.subspan(dst_offset, sibling->second.points.size()));
                    }
                }
            }

            std::array<std::pair<std::size_t, Vec2i>, 8> skip_list = {};
            std::size_t skip_list_count = 0;
            const auto parent_coords = multipoles.parent_coords(level, cell.first);
            for (auto parent_neighbours_coords: multipoles.neighbours_coords(level-1, parent_coords)) {
                if (std::find_if(skip_list.data(), skip_list.data()+skip_list_count, [&](auto& skip) { return multipoles.is_parent(parent_neighbours_coords, skip.first, skip.second); }) != skip_list.data()+skip_list_count) {
                    continue;
                }
                auto parent_level = level-1;                    
                auto parent_neighbour = cells.levels[parent_level].find(parent_neighbours_coords);
                if (parent_neighbour == cells.levels[parent_level].end()) {
                    auto [found_level, found_cell] = cells.find_childless_parent(parent_level, parent_neighbours_coords);  // TODO: uses slow has_children
                    if (found_level == 0) {
                        continue;
                    }
                    skip_list[skip_list_count++] = std::make_pair(found_level, found_cell->first);
                    parent_neighbour = found_cell;
                    parent_level = found_level;
                }
                if (not multipoles.is_adjacent(level, cell.first, parent_level, parent_neighbour->first)) {
                    std::ptrdiff_t dst_offset = parent_neighbour->second.points.data() - positions.data();
                    auto child_mask = 1u << (31-level);
                    auto cell_center = Vec2d(std::ldexp(cell.first.x | child_mask, -32), std::ldexp(cell.first.y | child_mask, -32));
                    auto multipole = multipoles.levels[level].find(cell.first);
                    compute_acceleration_multipole(cell_center, multipole->second,
                        parent_neighbour->second.points, accelerations.subspan(dst_offset, parent_neighbour->second.points.size()));
                } else if ((parent_neighbour->second.flags & 15) != 0) {
                    std::uint8_t has_child_mask = 1;
                    for (auto& cousin_coords: multipoles.children_coords(parent_level, parent_neighbour->first)) {
                        if ((parent_neighbour->second.flags & has_child_mask) == 0) {
                            has_child_mask <<= 1;
                            continue;
                        }
                        auto cousin = cells.levels[level].find(cousin_coords);
                        if (not cells.is_adjacent(level, cell.first, level, cousin_coords)) {
                            auto child_mask = 1u << (31-level);
                            auto cell_center = Vec2d(std::ldexp(cell.first.x | child_mask, -32), std::ldexp(cell.first.y | child_mask, -32));
                            auto multipole = multipoles.levels[level].find(cell.first);
                            std::ptrdiff_t dst_offset = cousin->second.points.data() - positions.data();
                            compute_acceleration_multipole(cell_center, multipole->second,
                                cousin->second.points, accelerations.subspan(dst_offset, cousin->second.points.size()));
                        } else if (not has_children) {
                            std::ptrdiff_t dst_offset = cousin->second.points.data() - positions.data();
                            std::ptrdiff_t src_offset = cell.second.points.data() - positions.data();
                            compute_acceleration_direct(cell.second.points, masses.subspan(src_offset, cell.second.points.size()),
                                cousin->second.points, accelerations.subspan(dst_offset, cousin->second.points.size()));
                        }
                        has_child_mask <<= 1;
                    }
                } else if (not has_children) {
                    std::ptrdiff_t src_offset = cell.second.points.data() - positions.data();
                    std::ptrdiff_t dst_offset = parent_neighbour->second.points.data() - positions.data();
                    compute_acceleration_direct(cell.second.points, masses.subspan(src_offset, cell.second.points.size()),
                        parent_neighbour->second.points, accelerations.subspan(dst_offset, parent_neighbour->second.points.size()));
                }
            }
        }
    }
}

void compute_acceleration_multipoles2(const QuadTree2<std::span<const Vec2d>>& cells, std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations) {
    auto multipoles = compute_multipoles2<32>(cells, positions, masses);

    for (std::size_t idx=0; idx<multipoles.cells.size(); ++idx) {
        auto& cell = cells.cells[idx];
        if (cell.children_count == 0) {
            std::ptrdiff_t src_offset = cell.value.data() - positions.data();
            compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()), accelerations.subspan(src_offset, cell.value.size()));

            for (auto& sibling: cells.siblings(cell)) {
                std::ptrdiff_t dst_offset = sibling->value.data() - positions.data();
                compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()), sibling->value, accelerations.subspan(dst_offset, sibling->value.size()));
            }
        }

        for (auto& parent_neighbour: cells.neighbours(cells.parent(cell))) {
            if (parent_neighbour->level+1 != cell.level && parent_neighbour->children_count != 0) {
                continue;
            }
            if (not multipoles.is_adjacent(cell.level, cell.coords, parent_neighbour->level, parent_neighbour->coords)) {
                std::ptrdiff_t dst_offset = parent_neighbour->value.data() - positions.data();
                auto child_mask = 1u << (31-cell.level);
                auto cell_center = Vec2d(std::ldexp(cell.coords.x | child_mask, -32), std::ldexp(cell.coords.y | child_mask, -32));
                compute_acceleration_multipole(cell_center, multipoles.cells[idx].value,
                    parent_neighbour->value, accelerations.subspan(dst_offset, parent_neighbour->value.size()));
            } else if (parent_neighbour->children_count != 0) {
                for (auto& cousin: cells.children(*parent_neighbour)) {
                    if (not cells.is_adjacent(cell.level, cell.coords, cousin.level, cousin.coords)) {
                        auto child_mask = 1u << (31-cell.level);
                        auto cell_center = Vec2d(std::ldexp(cell.coords.x | child_mask, -32), std::ldexp(cell.coords.y | child_mask, -32));
                        std::ptrdiff_t dst_offset = cousin.value.data() - positions.data();
                        compute_acceleration_multipole(cell_center, multipoles.cells[idx].value,
                            cousin.value, accelerations.subspan(dst_offset, cousin.value.size()));
                    } else if (cell.children_count == 0) {
                        std::ptrdiff_t dst_offset = cousin.value.data() - positions.data();
                        std::ptrdiff_t src_offset = cell.value.data() - positions.data();
                        compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()),
                            cousin.value, accelerations.subspan(dst_offset, cousin.value.size()));
                    }
                }
            } else if (cell.children_count == 0) {
                std::ptrdiff_t dst_offset = parent_neighbour->value.data() - positions.data();
                std::ptrdiff_t src_offset = cell.value.data() - positions.data();
                compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()),
                    parent_neighbour->value, accelerations.subspan(dst_offset, parent_neighbour->value.size()));
            }
        }
    }
}
