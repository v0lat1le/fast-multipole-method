#include <algorithm>
#include <complex>
#include <cmath>
#include <ranges>

#include "glm/geometric.hpp"
#include "poolstl/poolstl.hpp"

#include "QuadTree.hpp"
#include "Multipole.hpp"
#include "simulation.hpp"


constexpr glm::dvec2 cell_corner(glm::uvec2 coords) noexcept {
    return glm::ldexp(glm::dvec2{ coords }, glm::ivec2{ -32 });
}

constexpr glm::dvec2 cell_center(std::uint8_t level, glm::uvec2 coords) noexcept {
    assert(level < 32);
    auto child_mask = 1u << (31-level);
    return cell_corner(coords | child_mask);
}

template<std::size_t P>
struct M2LDrPowerTable {
    struct Powers {
        std::array<double, P> z_power_real;
        std::array<double, P> z_power_imag;
    };
    Powers powers[30][49];

    M2LDrPowerTable() {
        for (std::uint8_t level=0; level<30; ++level) {
            auto center_coords = glm::uvec2{ 3, 3 };
            auto center_coords_d = glm::ldexp(glm::dvec2{ center_coords }, glm::ivec2{ -2-level });
            for (std::uint32_t y=0; y<7; ++y) {
                for (std::uint32_t x=0; x<7; ++x) {
                    if (x == 3 and y == 3) continue;
                    auto target_coords = glm::uvec2{ x, y };
                    auto idx = y*7+x;
                    auto target_coords_d = glm::ldexp(glm::dvec2{ target_coords }, glm::ivec2{ -2-level });
                    auto dr = target_coords_d - center_coords_d;
                    double d = dr.x*dr.x + dr.y*dr.y;
                    double inv_real = dr.x/d;
                    double inv_imag = -dr.y/d;
                    powers[level][idx].z_power_real[0] = inv_real;
                    powers[level][idx].z_power_imag[0] = inv_imag;
                    for (std::size_t k=1; k<P; ++k) {
                        powers[level][idx].z_power_real[k] = powers[level][idx].z_power_real[k-1]*inv_real - powers[level][idx].z_power_imag[k-1]*inv_imag;
                        powers[level][idx].z_power_imag[k] = powers[level][idx].z_power_real[k-1]*inv_imag + powers[level][idx].z_power_imag[k-1]*inv_real;
                    }
                }
            }
        }
    }

    const Powers& get_powers(std::uint8_t level, glm::uvec2 src_coords, glm::uvec2 dst_coords) const noexcept {
        assert(level > 1);
        assert(level < 32);
        auto src_coords_local = (src_coords>>(32u-level));
        auto dst_coords_local = (dst_coords>>(32u-level));
        auto coords = (glm::uvec2{ 3, 3 } + src_coords_local) - dst_coords_local;
        auto idx = coords.y*7 + coords.x;
        return powers[level-2][idx];
    }
};

void compute_acceleration_direct(std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double eps) {
    for (std::size_t i=0; i<positions.size(); i++) {
        auto accel_i = glm::dvec2{0.0, 0.0};
        for (std::size_t j=i+1; j<positions.size(); j++) {
            auto dr = positions[j] - positions[i];
            auto d2_inv = 1.0 / (glm::dot(dr, dr) + eps);
            accel_i += dr*(masses[j]*d2_inv);
            accelerations[j] -= dr*(masses[i]*d2_inv);
        }
        accelerations[i] += accel_i;
    }
}

void compute_acceleration_direct(std::span<const glm::dvec2> src_pos, std::span<const double> src_mass, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc, double eps) {
    for (std::size_t j=0; j<dst_pos.size(); j++) {
        glm::dvec2 acc{};
        for (std::size_t i=0; i<src_pos.size(); i++) {
            auto dr = dst_pos[j] - src_pos[i];
            auto d2 = glm::dot(dr, dr) + eps;
            acc -= dr*(src_mass[i]/d2);
        }
        dst_acc[j] += acc;
    }
}

template<std::size_t P>
void compute_acceleration_multipole(glm::dvec2 src_pos, const Multipole<P>& multipole, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc) {
    for (std::size_t j=0; j<dst_pos.size(); j++) {
        dst_acc[j] += evaluate_multipole(multipole, dst_pos[j] - src_pos);
    }
}

void compute_acceleration_multipoles(const QuadTree<std::pair<std::uint32_t, std::uint32_t>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double direct_eps) {
    static const auto m2l_dr_power_table = M2LDrPowerTable<12>();

    auto levels = std::vector<std::uint32_t>{1};
    for (std::uint32_t i = 1; i<quadtree.cells.size(); ++i) {
        if (quadtree.cells[i].level != quadtree.cells[levels.back()].level) {
            levels.push_back(i);
        }
    }
    levels.push_back(static_cast<std::uint32_t>(quadtree.cells.size()));

    auto multipoles = std::vector<Multipole<12>>(quadtree.cells.size());
    auto multipole_upward_pass = [&](std::uint32_t cell_idx) {
        auto& cell = quadtree.cells[cell_idx];
        auto this_cell_center = cell_center(cell.level, cell.coords);
        if (cell.children_count == 0) {
            for (auto i=cell.value.first; i<cell.value.second; ++i) {
                calculate_multipole<12>(masses[i], multipoles[cell_idx], positions[i]-this_cell_center);
            }
        } else {
            for (auto child_idx = cell.children; child_idx < cell.children + cell.children_count; ++child_idx) {
                auto& child_mp = quadtree.cells[child_idx];
                auto child_center = cell_center(child_mp.level, child_mp.coords);
                translate_multipole(multipoles[child_idx], multipoles[cell_idx], child_center-this_cell_center);
            }
        }
    };

    for (std::size_t i=levels.size()-1; i-->0; ) {
        auto range = std::views::iota(levels[i], levels[i+1]);
        std::for_each(poolstl::par_if(range.size() > 16), range.begin(), range.end(), [&multipole_upward_pass](std::uint32_t cell_idx){ multipole_upward_pass(cell_idx); });
    }

    auto locals = std::vector<Local<12>>(quadtree.cells.size());
    auto neighbours = std::vector<std::array<std::uint32_t, 8>>(quadtree.cells.size());

    auto process_neighbour = [&](std::uint32_t cell_idx, std::uint32_t neighbour_idx, auto& recurse) {
        auto& cell = quadtree.cells[cell_idx];
        auto& neighbour = quadtree.cells[neighbour_idx];
        if (neighbour.children_count == 0) {
            compute_acceleration_direct(positions.subspan(neighbour.value.first, neighbour.value.second-neighbour.value.first), masses.subspan(neighbour.value.first, neighbour.value.second-neighbour.value.first), positions.subspan(cell.value.first, cell.value.second-cell.value.first), accelerations.subspan(cell.value.first, cell.value.second-cell.value.first), direct_eps);
            return;
        }
        for (auto descendant_idx = neighbour.children; descendant_idx < neighbour.children+neighbour.children_count; ++descendant_idx) {
            auto& descendant = quadtree.cells[descendant_idx];
            if (quadtree.is_adjacent(descendant.level, descendant.coords, cell.level, cell.coords)) {
                recurse(cell_idx, descendant_idx, recurse);
            } else {
                auto descendant_cell_center = cell_center(descendant.level, descendant.coords);
                compute_acceleration_multipole(descendant_cell_center, multipoles[descendant_idx], positions.subspan(cell.value.first, cell.value.second-cell.value.first), accelerations.subspan(cell.value.first, cell.value.second-cell.value.first));
            }
        }
    };

    auto process_cell = [&](std::uint32_t cell_idx) {
        auto& cell = quadtree.cells[cell_idx];
        auto& parent = quadtree.cells[cell.parent];

        if (cell.children_count == 0) {
            compute_acceleration_direct(positions.subspan(cell.value.first, cell.value.second-cell.value.first), masses.subspan(cell.value.first, cell.value.second-cell.value.first), accelerations.subspan(cell.value.first, cell.value.second-cell.value.first), direct_eps);
        }

        std::size_t neighbour_slot = 0;
        for (auto sibling_idx=parent.children; sibling_idx<parent.children+parent.children_count; ++sibling_idx) {
            if (sibling_idx == cell_idx) continue;
            neighbours[cell_idx][neighbour_slot++] = sibling_idx;
            if (cell.children_count == 0) {
                process_neighbour(cell_idx, sibling_idx, process_neighbour);
            }
        }
        for (auto parent_neighbour_idx: neighbours[cell.parent]) {
            if (parent_neighbour_idx == 0) break;
            auto& parent_neighbour = quadtree.cells[parent_neighbour_idx];
            if (parent_neighbour.children_count > 0) {
                for (auto cousin_idx=parent_neighbour.children; cousin_idx<parent_neighbour.children+parent_neighbour.children_count; ++cousin_idx) {
                    auto& cousin = quadtree.cells[cousin_idx];
                    if (quadtree.is_adjacent(cell.level, cell.coords, cousin.level, cousin.coords)) {
                        neighbours[cell_idx][neighbour_slot++] = cousin_idx;
                        if (cell.children_count == 0) {
                            process_neighbour(cell_idx, cousin_idx, process_neighbour);
                        }
                    } else {
                        auto& powers = m2l_dr_power_table.get_powers(cell.level, cousin.coords, cell.coords); 
                        convert_to_local<12>(multipoles[cousin_idx], locals[cell_idx], powers.z_power_real, powers.z_power_imag);
                    }
                }
            } else if (quadtree.is_adjacent(cell.level, cell.coords, parent_neighbour.level, parent_neighbour.coords)) {
                neighbours[cell_idx][neighbour_slot++] = parent_neighbour_idx;
                if (cell.children_count == 0) {
                    process_neighbour(cell_idx, parent_neighbour_idx, process_neighbour);
                }
            } else {
                auto this_cell_center = cell_center(cell.level, cell.coords);
                for (auto point_idx=parent_neighbour.value.first; point_idx<parent_neighbour.value.second; ++point_idx) {
                    charge_to_local<12>(masses[point_idx], locals[cell_idx], positions[point_idx] - this_cell_center);
                }
            }
        }
    };
    for (std::size_t i=0; i<levels.size()-1; ++i) {
        auto range = std::views::iota(levels[i], levels[i+1]);
        std::for_each(poolstl::par_if(range.size() > 16), range.begin(), range.end(), [&process_cell](std::uint32_t cell_idx) { process_cell(cell_idx); });
    }

    auto local_expansion_down_pass = [&](std::uint32_t cell_idx) {
        auto& cell = quadtree.cells[cell_idx];
        auto this_cell_center = cell_center(cell.level, cell.coords);

        auto& parent = quadtree.cells[cell.parent];
        auto parent_center = cell_center(parent.level, parent.coords);

        translate_local(locals[cell.parent], locals[cell_idx], parent_center-this_cell_center);

        if (cell.children_count == 0) {
            for (auto point_idx=cell.value.first; point_idx<cell.value.second; ++point_idx) {
                accelerations[point_idx] += evaluate_local(locals[cell_idx], positions[point_idx]-this_cell_center);
            }
        }
    };
    for (std::size_t i=0; i<levels.size()-1; ++i) {
        auto range = std::views::iota(levels[i], levels[i+1]);
        std::for_each(poolstl::par_if(range.size() > 16), range.begin(), range.end(), [&local_expansion_down_pass](std::uint32_t cell_idx){ local_expansion_down_pass(cell_idx); });
    }
}
