#include <algorithm>
#include <complex>
#include <cmath>
#include <mutex>
#include <thread>

#include "glm/geometric.hpp"

#include "QuadTree.hpp"
#include "Multipole.hpp"
#include "simulation.hpp"


QuadTree<std::span<const glm::dvec2>> build_quadtree(std::span<const glm::dvec2> points, int max_points, int max_levels) {
    QuadTree<std::span<const glm::dvec2>> quadtree(points);
    quadtree.cells.reserve(2*points.size()/max_points);

    for (std::size_t idx = 0; idx < quadtree.cells.size(); ++idx) {
        if (quadtree.cells.capacity() < quadtree.cells.size()+4) {  // avoid reallocation when adding items in the loop
            quadtree.cells.reserve(quadtree.cells.capacity()*1.5+4);
        }
        auto& parent = quadtree.cells[idx];
        if (parent.value.size() <= max_points || parent.level == max_levels) {
            continue;
        }

        auto begin = parent.value.begin();
        auto child_cell_size = 1u << (31-parent.level);
        const auto child_cells_coords = {
            parent.coords + glm::uvec2{child_cell_size, 0},
            parent.coords + glm::uvec2{0, child_cell_size},
            parent.coords + glm::uvec2{child_cell_size, child_cell_size},
        };
        auto prev_child_cell_coords = parent.coords;
        for (const auto& child_cell_coords: child_cells_coords) {
            auto double_space_coords = glm::ldexp(glm::dvec2{child_cell_coords}, glm::ivec2{-32});
            auto end = std::lower_bound(begin, parent.value.end(), double_space_coords, static_cast<bool(*)(const glm::dvec2&, const glm::dvec2&)>(cmp_zcurve_bitmagic));
            if (begin != end) {
                quadtree.add_cell({ begin, end }, prev_child_cell_coords, idx);
                begin = end;
            }
            prev_child_cell_coords = child_cell_coords;
        }
        if (begin != parent.value.end()) {
            quadtree.add_cell({ begin, parent.value.end() }, prev_child_cell_coords, idx);
        }
    }

    return quadtree;
}

void compute_acceleration_direct(std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double eps) {
    for (std::size_t i=0; i<positions.size(); i++) {
        for (std::size_t j=i+1; j<positions.size(); j++) {
            auto dr = positions[j] - positions[i];
            auto d2 = glm::dot(dr, dr) + eps;
            accelerations[i] += dr*masses[j]/d2;
            accelerations[j] -= dr*masses[i]/d2;
        }
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

template <std::size_t P>
std::vector<Multipole<P>> compute_multipoles(const QuadTree<std::span<const glm::dvec2>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses) {
    auto multipoles = std::vector<Multipole<P>>(quadtree.cells.size());
    for (std::size_t idx = quadtree.cells.size(); idx-- > 0;) {
        auto& cell = quadtree.cells[idx];
        auto this_cell_center = cell_center(cell.level, cell.coords);
        if (cell.children_count == 0) {
            std::ptrdiff_t offset = cell.value.data() - positions.data();
            for (int i=0; i<cell.value.size(); ++i) {
                calculate_multipole<P>(masses[offset+i], multipoles[idx], cell.value[i]-this_cell_center);
            }
        } else {
            for (auto child_idx = cell.children; child_idx < cell.children + cell.children_count; ++child_idx) {
                auto& child_mp = quadtree.cells[child_idx];
                auto child_center = cell_center(child_mp.level, child_mp.coords);
                translate_multipole(multipoles[child_idx], multipoles[idx], child_center-this_cell_center);
            }
        }
    }
    return multipoles;
}

template<std::size_t P>
void compute_acceleration_multipole(glm::dvec2 src_pos, const Multipole<P>& multipole, std::span<const glm::dvec2> dst_pos, std::span<glm::dvec2> dst_acc) {
    for (std::size_t j=0; j<dst_pos.size(); j++) {
        dst_acc[j] += evaluate_multipole(multipole, dst_pos[j] - src_pos);
    }
}

void compute_acceleration_multipoles(const QuadTree<std::span<const glm::dvec2>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double direct_eps) {
    std::vector<std::uint32_t> neighbour_storage;
    std::vector<std::uint32_t> neighbours;
    neighbours.emplace_back(0);
    for (std::uint32_t cell_idx=1; cell_idx<quadtree.cells.size(); ++cell_idx) {
        auto& cell = quadtree.cells[cell_idx];
        auto& parent = quadtree.cells[cell.parent];
        neighbours.emplace_back(neighbour_storage.size());
        for (std::uint32_t sibling_idx=parent.children; sibling_idx<parent.children+parent.children_count; ++sibling_idx) {
            if (sibling_idx != cell_idx) {
                neighbour_storage.push_back(sibling_idx);
            }
        }
        for (std::uint32_t parent_neighbour_idx=neighbours[cell.parent]; parent_neighbour_idx < neighbours[cell.parent+1]; ++parent_neighbour_idx) {
            auto& parent_neighbour = quadtree.cells[neighbour_storage[parent_neighbour_idx]];
            if (parent_neighbour.children_count == 0) {
                if (quadtree.is_adjacent(cell.level, cell.coords, parent_neighbour.level, parent_neighbour.coords)) {
                    neighbour_storage.push_back(neighbour_storage[parent_neighbour_idx]);
                }
            } else {
                for (std::uint32_t cousin_idx=parent_neighbour.children; cousin_idx<parent_neighbour.children+parent_neighbour.children_count; ++cousin_idx) {
                    auto& cousin = quadtree.cells[cousin_idx];
                    if (quadtree.is_adjacent(cell.level, cell.coords, cousin.level, cousin.coords)) {
                        neighbour_storage.push_back(cousin_idx);
                    }
                }
            }
        }
    }
    neighbours.emplace_back(neighbour_storage.size());

    auto multipoles = compute_multipoles<12>(quadtree, positions, masses);
    auto locals = std::vector<Local<12>>(quadtree.cells.size());

    auto process_neighbour = [&](std::uint32_t cell_idx, std::uint32_t neighbour_idx, auto& recurse){
        auto& cell = quadtree.cells[cell_idx];
        auto& neighbour = quadtree.cells[neighbour_idx];
        if (not quadtree.is_adjacent(neighbour.level, neighbour.coords, cell.level, cell.coords)) {
            auto neighbour_cell_center = cell_center(neighbour.level, neighbour.coords);
            std::ptrdiff_t dst_offset = cell.value.data() - positions.data();
            compute_acceleration_multipole(neighbour_cell_center, multipoles[neighbour_idx], cell.value, accelerations.subspan(dst_offset, cell.value.size()));
            return;
        }
        if (neighbour.children_count == 0) {
            std::ptrdiff_t dst_offset = cell.value.data() - positions.data();
            std::ptrdiff_t src_offset = neighbour.value.data() - positions.data();
            compute_acceleration_direct(neighbour.value, masses.subspan(src_offset, neighbour.value.size()), cell.value, accelerations.subspan(dst_offset, cell.value.size()), direct_eps);
            return;
        }
        for (auto neighbour_descendant_idx = neighbour.children; neighbour_descendant_idx < neighbour.children+neighbour.children_count; ++neighbour_descendant_idx) {
            recurse(cell_idx, neighbour_descendant_idx, recurse);
        }
    };

    auto process_cell = [&](std::uint32_t cell_idx) {
        auto& cell = quadtree.cells[cell_idx];
        if (cell.children_count == 0) {
            std::ptrdiff_t dst_offset = cell.value.data() - positions.data();
            compute_acceleration_direct(cell.value, masses.subspan(dst_offset, cell.value.size()), accelerations.subspan(dst_offset, cell.value.size()), direct_eps);
            for (auto neighbour_idx=neighbours[cell_idx]; neighbour_idx < neighbours[cell_idx+1]; ++neighbour_idx) {
                process_neighbour(cell_idx, neighbour_storage[neighbour_idx], process_neighbour);
            }
        }
        auto& parent = quadtree.cells[cell.parent];
        for (std::uint32_t parent_neighbour_idx=neighbours[cell.parent]; parent_neighbour_idx < neighbours[cell.parent+1]; ++parent_neighbour_idx) {
            auto& parent_neighbour = quadtree.cells[neighbour_storage[parent_neighbour_idx]];
            if (parent_neighbour.children_count > 0) {
                for (auto cousin_idx = parent_neighbour.children; cousin_idx < parent_neighbour.children+parent_neighbour.children_count; ++cousin_idx) {
                    auto& cousin = quadtree.cells[cousin_idx];
                    if (not quadtree.is_adjacent(cell.level, cell.coords, cousin.level, cousin.coords)) {
                        auto dr = cell_center(cousin.level, cousin.coords) - cell_center(cell.level, cell.coords);
                        convert_to_local(multipoles[cousin_idx], locals[cell_idx], dr);
                    }
                }            
            } else if (not quadtree.is_adjacent(cell.level, cell.coords, parent_neighbour.level, parent_neighbour.coords)) {
                auto this_cell_center = cell_center(cell.level, cell.coords);
                for (std::size_t point_idx=parent_neighbour.value.data() - positions.data(); point_idx<parent_neighbour.value.data() + parent_neighbour.value.size() - positions.data(); ++point_idx) {
                    charge_to_local<12>(masses[point_idx], locals[cell_idx], positions[point_idx] - this_cell_center);
                }
            }
        }
    };

    {
        std::atomic<std::size_t> next_cell = 1;
        auto thread_func = [&]() {
            while (true) {
                auto cell_idx = next_cell.fetch_add(1, std::memory_order_relaxed);
                if (cell_idx >= quadtree.cells.size()) return;
                process_cell(cell_idx);
            }
        };
        std::array<std::jthread, 6> threads;
        for (int i=0; i<threads.size(); ++i) {
            threads[i] = std::jthread{ thread_func };
        };
    }

    for (std::size_t idx=1; idx<quadtree.cells.size(); ++idx) {
        auto& cell = quadtree.cells[idx];
        auto this_cell_center = cell_center(cell.level, cell.coords) ;

        auto& parent = quadtree.cells[cell.parent];
        auto parent_center = cell_center(parent.level, parent.coords);

        translate_local(locals[cell.parent], locals[idx], parent_center-this_cell_center);

        if (cell.children_count == 0) {
            for (auto& p: cell.value) {
                accelerations[&p - positions.data()] += evaluate_local(locals[idx], p-this_cell_center);
            }
        }
    }
}
