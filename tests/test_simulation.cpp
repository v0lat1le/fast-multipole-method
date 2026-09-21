#include <algorithm>
#include <complex>
#include <random>
#include <ranges>

#include "grtest.h"
#include "simulation.hpp"


template<typename T>
void test_cmp_zcurve() {
    for (uint32_t x1=0; x1<8; ++x1) {
        for (uint32_t y1=0; y1<8; ++y1) {
            for (uint32_t x2=0; x2<8; ++x2) {
                for (uint32_t y2=0; y2<8; ++y2) {
                    auto interleave = cmp_zcurve_interleave(Vec2i{ x1, y1 }, Vec2i{ x2, y2 });
                    auto bit_magic = cmp_zcurve_bitmagic(Vec2<T>{ static_cast<T>(x1), static_cast<T>(y1) }, Vec2<T>{ static_cast<T>(x2), static_cast<T>(y2) });
                    assert(interleave == bit_magic);
                }
            }
        }
    }
}

TEST_CASE(test_cmp_zcurve) {
    test_cmp_zcurve<uint32_t>();
    test_cmp_zcurve<double>();
}

TEST_CASE(test_build_quadtree) {
    auto points = std::vector<Vec2d>({ {0.11, 0.11}, {0.1, 0.1}, {0.1, 0.11}, {0.11, 0.1} });
    std::sort(points.begin(), points.end(), static_cast<bool(*)(const Vec2d&, const Vec2d&)>(cmp_zcurve_bitmagic));
    auto cells = build_quadtree(points, 32, 1);
    assert(cells.levels[6].size() == 4);
}

bool equal_approx(double a, double b, double eps=1e-10) {
    return std::abs(a - b) <= eps;
}

bool equal_approx(Vec2d a, Vec2d b, double eps=1e-10) {
    return equal_approx(a.x, b.x, eps) && equal_approx(a.y, b.y, eps);
}

void run_test(std::vector<Vec2d> positions, std::vector<double> masses) {
    std::vector<Vec2d> accelerations_exepected(positions.size());
    std::vector<Vec2d> accelerations(positions.size());

    auto zipped = std::ranges::views::zip(positions, masses);
    std::ranges::sort(zipped, [](const auto& lhs, const auto& rhs) {
        return cmp_zcurve_bitmagic(std::get<0>(lhs), std::get<0>(rhs));
    });
    std::sort(positions.begin(), positions.end(), static_cast<bool(*)(const Vec2d&, const Vec2d&)>(cmp_zcurve_bitmagic));
    compute_acceleration_direct(positions, masses, accelerations_exepected);

    auto cells = build_quadtree(positions, 16, 1);
    compute_acceleration_multipoles(cells, positions, masses, accelerations);
    for (int i=0; i<accelerations.size(); ++i) {
        assert(equal_approx(accelerations[i], accelerations_exepected[i], 0.01));
    }
}

void run_test(std::vector<Vec2d> positions) {
    run_test(positions, std::vector<double>(positions.size(), 1.0));
}

//TEST_CASE(test_compute_acceleration_trivial) {
//    std::vector<Vec2d> positions;
//    std::vector<double> masses;
//    std::vector<Vec2d> accelerations;
//
//    Cells cells;
//    cells.levels.emplace_back();
//    cells.levels.emplace_back();
//    cells.levels.emplace_back();
//
//    // empty space
//    compute_acceleration_multipoles(cells, positions, masses, accelerations);
//
//    // single particle
//    positions.emplace_back(0.1, 0.1);
//    masses.emplace_back(1.0);
//    accelerations.emplace_back(0.0, 0.0);
//    cells.levels[0][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.end()} };
//    cells.levels[1][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.end()} };
//    compute_acceleration_multipoles(cells, positions, masses, accelerations);
//    assert((accelerations[0] == Vec2{ 0.0, 0.0 }));
//
//    // two particles in the same cell
//    positions.emplace_back(0.2, 0.1);
//    masses.emplace_back(1.0);
//    accelerations.emplace_back(0.0, 0.0);
//    cells.levels[0][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.end()} };
//    cells.levels[1][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.end()} };
//    compute_acceleration_multipoles(cells, positions, masses, accelerations);
//    assert(equal_approx(accelerations[0], Vec2{ 10.0, 0.0 }));
//    assert(equal_approx(accelerations[1], Vec2{ -10.0, 0.0 }));
//
//    // two particles in neighbouring cells
//    positions.at(1) = { 0.6, 0.1 };
//    std::fill(accelerations.begin(), accelerations.end(), Vec2d{ 0.0, 0.0 });
//    cells.levels[0][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.end()} };
//    cells.levels[1][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.begin()+1} };
//    cells.levels[1][Vec2i{ 1u<<31,0 }] = Cell{ {positions.begin()+1, positions.end()} };
//    compute_acceleration_multipoles(cells, positions, masses, accelerations);
//    assert(equal_approx(accelerations[0], Vec2{ 2.0, 0.0 }));
//    assert(equal_approx(accelerations[1], Vec2{ -2.0, 0.0 }));
//
//    // two particles in far cells
//    positions.at(1) = { 0.6, 0.1 };
//    std::fill(accelerations.begin(), accelerations.end(), Vec2d{ 0.0, 0.0 });
//    cells.levels[0][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.end()} };
//    cells.levels[1][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.begin()+1} };
//    cells.levels[1][Vec2i{ 1u<<31,0 }] = Cell{ {positions.begin()+1, positions.end()} };
//    cells.levels[2][Vec2i{ 0,0 }] = Cell{ {positions.begin(), positions.begin()+1} };
//    cells.levels[2][Vec2i{ 1u<<31,0 }] = Cell{ {positions.begin()+1, positions.end()} };
//    std::fill(accelerations.begin(), accelerations.end(), Vec2d{ 0.0, 0.0 });
//    compute_acceleration_multipoles(cells, positions, masses, accelerations);
//    assert(equal_approx(accelerations[0], Vec2{ 2.0, 0.0 }, 1e-6));
//    assert(equal_approx(accelerations[1], Vec2{ -2.0, 0.0 }, 1e-6));
//}

TEST_CASE(test_4_points_1) {
    run_test({
        Vec2d{0.125, 0.625},
        Vec2d{0.375, 0.875},
        Vec2d{0.5625, 0.8125},
        Vec2d{0.6875, 0.8125}
    });
}

TEST_CASE(test_4_points_2) {
    run_test({
        Vec2d{0.75, 0.75},
        Vec2d{0.375, 0.875},
        Vec2d{0.3125, 0.5625},
        Vec2d{0.4375, 0.5625}
    });
}

TEST_CASE(test_5_points_1) {
    run_test({
        Vec2d{0.125, 0.125},
        Vec2d{0.625, 0.125},
        Vec2d{0.125, 0.625},
        Vec2d{0.625, 0.625},
        Vec2d{0.875, 0.875}
    });
}

TEST_CASE(test_random) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    std::vector<Vec2d> positions;
    std::vector<double> masses;
    for (int q=3; q<10; ++q) {
        positions.resize(q);
        masses.resize(q);
        for (int k=0; k<1000; ++k) {
            for (std::size_t i=0; i<positions.size(); ++i) {
                positions[i] = { distrib(gen), distrib(gen) };
                masses[i] = distrib(gen);
            }
            run_test(positions, masses);
        }
    }
}
