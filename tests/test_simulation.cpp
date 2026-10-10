#include <algorithm>
#include <random>
#include <ranges>

#include "grtest.h"
#include "simulation.hpp"


void check_quadtree(std::vector<glm::dvec2>& points) {
    std::vector<std::uint64_t> keys(points.size());
    for (std::size_t i=0; i<points.size(); ++i) {
        keys[i] = morton_code(points[i]);
    }
    auto zipped = std::views::zip(keys, points);
    auto proj = [](const auto& v) { return std::get<0>(v); };

    auto quadtree = build_quadtree(zipped, proj);
    for (auto& cell: quadtree.cells) {
        double x = std::ldexp(cell.coords.x, -32);
        double y = std::ldexp(cell.coords.y, -32);
        double cell_size = std::ldexp(1.0, -static_cast<int>(cell.level));
        for (auto point_idx=cell.value.first; point_idx<cell.value.first; ++point_idx) {
            auto& point = points[point_idx];
            assert(point.x >= x && point.y >= y && point.x < x+cell_size && point.y < y+cell_size);
        }
    }
}

TEST_CASE(test_build_quadtree_3_points_1) {
    auto positions = std::vector<glm::dvec2>{
        glm::dvec2{0.54341204441673263, 0.74620193825305203 },
        glm::dvec2{0.69151111077974448, 0.66069690242163481 },
        glm::dvec2{0.75000000000000000, 0.50000000000000000 },
    };
    check_quadtree(positions);
}

TEST_CASE(test_build_quadtree_random) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    std::vector<glm::dvec2> positions;
    for (int q=3; q<10; ++q) {
        positions.resize(q);
        for (int k=0; k<1000; ++k) {
            for (std::size_t i=0; i<positions.size(); ++i) {
                positions[i] = glm::dvec2{ distrib(gen), distrib(gen) };
            }
            check_quadtree(positions);
        }
    }
}

bool equal_approx(double a, double b, double eps=1e-10) {
    return std::abs(a - b) <= eps;
}

bool equal_approx(glm::dvec2 a, glm::dvec2 b, double eps=1e-10) {
    return equal_approx(a.x, b.x, eps) && equal_approx(a.y, b.y, eps);
}

void run_test(std::vector<glm::dvec2> positions, std::vector<double> masses) {
    std::vector<std::uint64_t> keys(positions.size());
    std::vector<glm::dvec2> accelerations_exepected(positions.size());
    std::vector<glm::dvec2> accelerations(positions.size());

    for (std::size_t i=0; i<keys.size(); ++i) {
        keys[i] = morton_code(positions[i]);
    }
    auto zipped = std::views::zip(keys, positions, masses);
    auto proj = [](const auto& v) { return std::get<0>(v); };

    auto quadtree = build_quadtree(zipped, proj);
    compute_acceleration_multipoles(quadtree, positions, masses, accelerations);

    compute_acceleration_direct(positions, masses, accelerations_exepected);
    for (int i=0; i<accelerations.size(); ++i) {
        assert(equal_approx(accelerations[i], accelerations_exepected[i], 1e-2));
    }
}

void run_test(std::vector<glm::dvec2> positions) {
    run_test(positions, std::vector<double>(positions.size(), 1.0));
}

TEST_CASE(test_3_points_1) {
    run_test({
        glm::dvec2{0.24400525008387419, 0.74469842234102612 },
        glm::dvec2{0.31160388924049598, 0.64410738395399525 },
        glm::dvec2{0.28948115461189083, 0.67172580077738531 },
    });
}

TEST_CASE(test_3_points_2) {
    run_test({
        glm::dvec2{0.54341204441673263, 0.74620193825305203 },
        glm::dvec2{0.69151111077974448, 0.66069690242163481 },
        glm::dvec2{0.75000000000000000, 0.50000000000000000 },
    });
}

TEST_CASE(test_4_points_1) {
    run_test({
        glm::dvec2{0.125, 0.625},
        glm::dvec2{0.375, 0.875},
        glm::dvec2{0.5625, 0.8125},
        glm::dvec2{0.6875, 0.8125}
    });
}

TEST_CASE(test_4_points_2) {
    run_test({
        glm::dvec2{0.75, 0.75},
        glm::dvec2{0.375, 0.875},
        glm::dvec2{0.3125, 0.5625},
        glm::dvec2{0.4375, 0.5625}
    });
}

TEST_CASE(test_5_points_1) {
    run_test({
        glm::dvec2{0.125, 0.125},
        glm::dvec2{0.625, 0.125},
        glm::dvec2{0.125, 0.625},
        glm::dvec2{0.625, 0.625},
        glm::dvec2{0.875, 0.875}
    });
}

TEST_CASE(test_circles) {
    auto tau = 6.283185307179586;
    for (int q=3; q<10; ++q) {
        std::vector<glm::dvec2> positions(q);
        for (int i=0; i<q; ++i) {
            positions[i] = {0.25*std::cos(tau*i/q)+0.5, 0.25*std::sin(tau*i/q)+0.5};
        }
        run_test(positions);
    }
}

TEST_CASE(test_random) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    std::vector<glm::dvec2> positions;
    std::vector<double> masses;
    for (int q=3; q<10; ++q) {
        positions.resize(q);
        masses.resize(q);
        for (int k=0; k<100; ++k) {
            for (std::size_t i=0; i<positions.size(); ++i) {
                positions[i] = { distrib(gen), distrib(gen) };
                masses[i] = distrib(gen);
            }
            run_test(positions, masses);
        }
    }
}
