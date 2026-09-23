#include <algorithm>
#include <complex>
#include <cmath>

#include "Vec2.hpp"
#include "QuadTree.hpp"
#include "Multipole.hpp"
#include "simulation.hpp"


QuadTree<std::span<const Vec2d>> build_quadtree(std::span<const Vec2d> points, int max_levels, int max_points) {
    QuadTree<std::span<const Vec2d>> cells(points);

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

void compute_acceleration_direct(std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations) {
    for (std::size_t i=0; i<positions.size(); i++) {
        for (std::size_t j=i+1; j<positions.size(); j++) {
            auto dr = positions[j] - positions[i];
            double d2 = dr.x*dr.x + dr.y*dr.y + 1e-10;
            accelerations[i] += dr*masses[j]/d2;
            accelerations[j] -= dr*masses[i]/d2;
        }
    }
}

void compute_acceleration_direct(std::span<const Vec2d> src_pos, std::span<const double> src_mass, std::span<const Vec2d> dst_pos, std::span<Vec2d> dst_acc) {
    for (std::size_t i=0; i<src_pos.size(); i++) {
        for (std::size_t j=0; j<dst_pos.size(); j++) {
            auto dr = dst_pos[j] - src_pos[i];
            double d2 = dr.x*dr.x + dr.y*dr.y + 1e-10;
            dst_acc[j] -= dr*src_mass[i]/d2;
        }
    }
}

template <std::size_t P>
QuadTree<Multipole<P>> compute_multipoles(const QuadTree<std::span<const Vec2d>>& cells, std::span<const Vec2d> positions, std::span<const double> masses) {
    auto multipoles = QuadTree<Multipole<P>>({});
    multipoles.cells.resize(cells.cells.size());
    for (std::size_t idx = multipoles.cells.size(); idx-- > 0;) {
        auto& cell = cells.cells[idx];
        auto& multipole = multipoles.cells[idx];
        multipole.coords = cell.coords;
        multipole.parent = cell.parent;
        multipole.children = cell.children;
        multipole.children_count = cell.children_count;
        multipole.level = cell.level;
        auto cell_mask = 1u << (31-cell.level);
        auto cell_center = Vec2d(std::ldexp(cell.coords.x | cell_mask, -32), std::ldexp(cell.coords.y | cell_mask, -32));
        if (cell.children_count == 0) {
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
        } else {
            for (const auto& child_mp: multipoles.children(multipole)) {
                auto child_mask = 1u << (31-child_mp.level);
                auto child_center = Vec2d(std::ldexp(child_mp.coords.x | child_mask, -32), std::ldexp(child_mp.coords.y | child_mask, -32));
                auto translated = translate_multipole(child_mp.value, child_center-cell_center);
                multipole.value.q += translated.q;
                for (int k=0; k<P; ++k) {
                    multipole.value.a[k] += translated.a[k];
                }
            }
        }
    }
    return multipoles;
}

template<std::size_t P>
void compute_acceleration_multipole(Vec2d src_pos, const Multipole<P>& multipole, std::span<const Vec2d> dst_pos, std::span<Vec2d> dst_acc) {
    for (std::size_t j=0; j<dst_pos.size(); j++) {
        auto dr = dst_pos[j] - src_pos;
        auto z_inv = 1.0/std::complex(dr.x, dr.y);  // TODO: this explodes?
        auto accel = multipole.q*z_inv;
        auto z_power = z_inv;
        for (int k=0; k<multipole.a.size(); ++k) {
            z_power *= z_inv;
            accel -= (k+1.0)*multipole.a[k]*z_power;
        }
        dst_acc[j] -= Vec2d(accel.real(), -accel.imag());
    }
}

void compute_acceleration_multipoles(const QuadTree<std::span<const Vec2d>>& cells, std::span<const Vec2d> positions, std::span<const double> masses, std::span<Vec2d> accelerations) {
    auto multipoles = compute_multipoles<32>(cells, positions, masses);

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
