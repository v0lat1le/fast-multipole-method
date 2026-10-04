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

template <std::size_t P>
std::vector<Multipole<P>> compute_multipoles(const QuadTree<std::span<const glm::dvec2>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses) {
    auto multipoles = std::vector<Multipole<P>>(quadtree.cells.size());
    for (std::size_t idx = quadtree.cells.size(); idx-- > 0;) {
        auto& cell = quadtree.cells[idx];
        auto cell_mask = 1u << (31-cell.level);
        auto cell_center = glm::ldexp(glm::dvec2{cell.coords | cell_mask}, glm::ivec2{-32});
        if (cell.children_count == 0) {
            std::ptrdiff_t offset = cell.value.data() - positions.data();
            for (int i=0; i<cell.value.size(); ++i) {
                multipoles[idx] += calculate_multipole<P>(masses[offset+i], cell.value[i]-cell_center);
            }
        } else {
            for (auto child_idx = cell.children; child_idx < cell.children + cell.children_count; ++child_idx) {
                auto& child_mp = quadtree.cells[child_idx];
                auto child_mask = 1u << (31-child_mp.level);
                auto child_center = glm::ldexp(glm::dvec2{child_mp.coords | child_mask}, glm::ivec2{-32});
                multipoles[idx] += translate_multipole(multipoles[child_idx], child_center-cell_center);
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


template<std::size_t P>
void update_local(Local<P>& dst, const Local<P>& src) {
    for (int i=0; i<P; ++i) {
        dst[i] += src[i];
    }
}



void compute_acceleration_multipoles(const QuadTree<std::span<const glm::dvec2>>& quadtree, std::span<const glm::dvec2> positions, std::span<const double> masses, std::span<glm::dvec2> accelerations, double direct_eps) {
    std::vector<std::uint32_t> neighbour_storage;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> neighbours;
    std::array<std::vector<std::tuple<int, int, glm::dvec2>>, 6> queues;
    
    auto multipoles = compute_multipoles<12>(quadtree, positions, masses);
    auto locals = std::vector<Local<12>>(quadtree.cells.size());

    neighbours.emplace_back(0, 0);
    for (std::uint32_t cell_idx=1; cell_idx<quadtree.cells.size(); ++cell_idx) {
        auto& cell = quadtree.cells[cell_idx];
        auto& parent = quadtree.cells[cell.parent];
        neighbours.emplace_back(neighbour_storage.size(), 0);
        if (cell.children_count == 0) {
            std::ptrdiff_t src_offset = cell.value.data() - positions.data();
            compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()), accelerations.subspan(src_offset, cell.value.size()), direct_eps);
        }
        for (std::uint32_t sibling_idx=parent.children; sibling_idx<parent.children+parent.children_count; ++sibling_idx) {
            if (sibling_idx != cell_idx) {
                neighbour_storage.push_back(sibling_idx);
                neighbours.back().second++;
                if (cell.children_count == 0) {
                    auto& sibling = quadtree.cells[sibling_idx];
                    std::ptrdiff_t src_offset = cell.value.data() - positions.data();
                    std::ptrdiff_t dst_offset = sibling.value.data() - positions.data();
                    // TODO: local expansion from each particle to non-adjacent children, otherwise direct
                    compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()), sibling.value, accelerations.subspan(dst_offset, sibling.value.size()), direct_eps);
                }
            }
        }
        auto [offset, count] = neighbours[cell.parent];
        for (std::uint32_t parent_neighbour_idx=offset; parent_neighbour_idx < offset+count; ++parent_neighbour_idx) {
            auto& parent_neighbour = quadtree.cells[neighbour_storage[parent_neighbour_idx]];
            if (parent_neighbour.children_count == 0) {
                if (quadtree.is_adjacent(cell.level, cell.coords, parent_neighbour.level, parent_neighbour.coords)) {
                    neighbour_storage.push_back(neighbour_storage[parent_neighbour_idx]);
                    neighbours.back().second++;
                    if (cell.children_count == 0) {
                        std::ptrdiff_t src_offset = cell.value.data() - positions.data();
                        std::ptrdiff_t dst_offset = parent_neighbour.value.data() - positions.data();
                        compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()), parent_neighbour.value, accelerations.subspan(dst_offset, parent_neighbour.value.size()), direct_eps);
                    }
                } else {
                    auto child_mask = 1u << (31-cell.level);
                    auto cell_center = glm::ldexp(glm::dvec2{ cell.coords | child_mask }, glm::ivec2{ -32 });
                    std::ptrdiff_t dst_offset = parent_neighbour.value.data() - positions.data();
                    compute_acceleration_multipole(cell_center, multipoles[cell_idx], parent_neighbour.value, accelerations.subspan(dst_offset, parent_neighbour.value.size()));
                }
            } else {
                for (std::uint32_t cousin_idx=parent_neighbour.children; cousin_idx<parent_neighbour.children+parent_neighbour.children_count; ++cousin_idx) {
                    auto& cousin = quadtree.cells[cousin_idx];
                    if (quadtree.is_adjacent(cell.level, cell.coords, cousin.level, cousin.coords)) {
                        assert(cousin.children_count == 0 || cell.level == cousin.level);
                        neighbour_storage.push_back(cousin_idx);
                        neighbours.back().second++;
                        if (cell.children_count == 0) {
                            std::ptrdiff_t src_offset = cell.value.data() - positions.data();
                            std::ptrdiff_t dst_offset = cousin.value.data() - positions.data();
                            // TODO: local expansion from each particle to non-adjacent children, otherwise direct
                            compute_acceleration_direct(cell.value, masses.subspan(src_offset, cell.value.size()), cousin.value, accelerations.subspan(dst_offset, cousin.value.size()), direct_eps);
                        }
                    } else {
                        assert(cell.level == cousin.level);
                        auto child_mask = 1u << (31-cell.level);
                        auto cell_center = glm::ldexp(glm::dvec2{ cell.coords | child_mask }, glm::ivec2{ -32 });
                        auto cousin_cell_center = glm::ldexp(glm::dvec2{ cousin.coords | child_mask }, glm::ivec2{ -32 });
                        queues[cousin_idx%queues.size()].push_back({ cousin_idx , cell_idx, cell_center-cousin_cell_center });
                    }
                }
            }
        }
    }

    {
        auto thread_func = [&](int id) {
            for (auto& [local_idx, multipole_idx, dr]: queues[id]) {
                update_local(locals[local_idx], convert_to_local(multipoles[multipole_idx], dr));
            }
        };
        std::array<std::jthread, queues.size()> threads;
        for (int i=0; i<threads.size(); ++i) {
            threads[i] = std::jthread{ thread_func, i };
        };
    }

    for (std::size_t idx=1; idx<quadtree.cells.size(); ++idx) {
        auto& cell = quadtree.cells[idx];

        auto child_mask = 1u << (31-cell.level);
        auto cell_center = glm::ldexp(glm::dvec2{ cell.coords | child_mask }, glm::ivec2{ -32 });

        auto& parent = quadtree.cells[cell.parent];
        auto parent_mask = 1u << (31-parent.level);
        auto parent_center = glm::ldexp(glm::dvec2{ parent.coords | parent_mask }, glm::ivec2{ -32 });

        auto translated_local = translate_local(locals[cell.parent], parent_center-cell_center);
        for (int i=0; i<translated_local.size(); ++i) {
            locals[idx][i] += translated_local[i];
        }

        if (cell.children_count == 0) {
            for (auto& p: cell.value) {
                accelerations[&p - positions.data()] += evaluate_local(locals[idx], p-cell_center);
            }
        }
    }
}
