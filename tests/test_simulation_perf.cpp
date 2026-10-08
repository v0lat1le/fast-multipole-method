#include <algorithm>
#include <chrono>
#include <random>
#include <ranges>

#include "grtest.h"
#include "simulation.hpp"
#include "ThreadPool.hpp"


template<typename T>
void run_test(T& distribution, std::size_t n) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<std::uint64_t> keys(n);
    std::vector<glm::dvec2> positions(n);
    std::vector<glm::dvec2> accelerations(n);
    std::vector<double> masses(n, 1.0);
    for (std::size_t i=0; i<positions.size(); ++i) {
        positions[i] = { distribution(gen), distribution(gen) };
        keys[i] = InterleaveHash::operator()(positions[i]);
    }
    auto zipped = std::ranges::views::zip(keys, positions, masses);
    auto proj = [](const auto& v) { return std::get<0>(v); };
    auto quadtree = build_quadtree(zipped, proj, 20);

    ThreadPool thread_pool(8);

    for (std::size_t i=0; i<10; ++i) {
        std::chrono::time_point start = std::chrono::steady_clock::now();
        compute_acceleration_multipoles(quadtree, positions, masses, accelerations, 1e-9, [&thread_pool](std::function<void()> func){return thread_pool.submit(func);});
        std::chrono::time_point stop = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Time taken: " << duration.count() << " μs" << std::endl;
    }
}

TEST_CASE(test_uniform_random) {
    std::uniform_real_distribution<double> distrib(0, 1);
    run_test(distrib, 100000);
}

TEST_CASE(test_gaussian_random) {
    std::normal_distribution<double> distrib(0.5, 0.25);
    auto truncated = [&distrib](auto& gen) {
        double x;
        do {
            x = distrib(gen);
        } while (x < 0 || x > 1);
        return x;
    };
    run_test(truncated, 100000);
}
