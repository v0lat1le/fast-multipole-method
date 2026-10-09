#include <algorithm>
#include <complex>
#include <cmath>
#include <future>
#include <ranges>

#include "glm/geometric.hpp"

#include "QuadTree.hpp"
#include "Multipole.hpp"
#include "simulation.hpp"


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

constexpr glm::dvec2 cell_center(std::uint8_t level, glm::uvec2 coords) noexcept {
    assert(level < 32);
    auto child_mask = 1u << (31-level);
    return glm::ldexp(glm::dvec2{ coords | child_mask }, glm::ivec2{ -32 });
}

template<std::size_t P>
void compute_acceleration_multipole(glm::dvec2 src_pos, const Multipole<P>& multipole, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc) {
    for (std::size_t j=0; j<dst_pos.size(); j++) {
        dst_acc[j] += evaluate_multipole(multipole, dst_pos[j] - src_pos);
    }
}

void compute_acceleration_multipoles(const QuadTree<std::pair<std::uint32_t, std::uint32_t>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double direct_eps, std::function<std::future<void>(std::function<void()>)> submit_task) {
    auto parallel_for = [&submit_task](std::uint32_t begin, std::uint32_t end, auto&& func) {
        auto size = end-begin;
        auto chunk_base = size/8;
        auto remainder = size%8;
        std::vector<std::future<void>> futures;
        for (std::uint32_t i=0; i<8 and begin<end; ++i) {
            auto chunk_end = begin + chunk_base + (i<remainder ? 1 : 0);
            futures.push_back(submit_task([begin, chunk_end, &func]() {
                for (auto j=begin; j<chunk_end; ++j) {
                    func(j);
                }
            }));
            begin = chunk_end;
        }
        for (auto& f: futures) f.get();
    };

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
        auto begin = levels[i];
        auto end = levels[i+1];
        parallel_for(begin, end, [&quadtree, &multipole_upward_pass](std::uint32_t cell_idx){ multipole_upward_pass(cell_idx); });
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
                        auto dr = cell_center(cousin.level, cousin.coords) - cell_center(cell.level, cell.coords);
                        convert_to_local(multipoles[cousin_idx], locals[cell_idx], dr);
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
        auto begin = levels[i];
        auto end = levels[i+1];
        parallel_for(begin, end, [&process_cell](std::uint32_t cell_idx) { process_cell(cell_idx); });
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
        auto begin = levels[i];
        auto end = levels[i+1];
        parallel_for(begin, end, [&local_expansion_down_pass](std::uint32_t cell_idx) { local_expansion_down_pass(cell_idx); });
    }
}
