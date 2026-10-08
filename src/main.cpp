#include <algorithm>
#include <cmath>
#include <random>
#include <ranges>

#ifdef FMM_RENDERER_VULKAN
#define RGFW_VULKAN
#else
#define GLAD_GL_IMPLEMENTATION
#include "glad/gl.h"
#define RGFW_OPENGL
#endif

#define RGFW_IMPLEMENTATION
#define NOMINMAX
#include "RGFW.h"

#include "glm/gtc/random.hpp"

#include "QuadTree.hpp"
#include "simulation.hpp"
#include "ThreadPool.hpp"


void populate_system(std::span<glm::dvec2> positions, std::span<glm::dvec2> velocities, std::span<double> masses, double r=0.5) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(-r, r);
    glm::dvec2 system_vel{};
    double density = std::sqrt(positions.size())*0.3;
    for (std::size_t i=0; i<positions.size(); ++i) {
        double x, y, d2;
        do {
            x = distrib(gen)/5.0;
            y = distrib(gen);
            d2 = x*x + y*y;
        } while (x*x + y*y > r*r);
        positions[i] = {x+0.5, y+0.5};
        velocities[i] = {-y*density/std::sqrt(d2), x*density/std::sqrt(d2)};
        masses[i] = 1.0;
        system_vel += velocities[i];
    }
    for (auto& vel: velocities) {
        vel -= system_vel/static_cast<double>(positions.size());
    }
}

void make_ring(std::span<glm::dvec2> positions, std::span<glm::dvec2> velocities, std::span<double> masses, double r=0.5) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(-r, r);

    positions[0] = { 0.5, 0.5 };
    velocities[0] = { 0.0, 0.0 };
    masses[0] = 100.0;

    auto v = std::sqrt(masses[0]);
    for (std::size_t i=1; i<positions.size(); ++i) {
        double x, y, d2;
        do {
            x = distrib(gen);
            y = distrib(gen);
            d2 = x*x + y*y;
        } while (d2 > r*r || d2 < 0.25*r*r);
        positions[i] = { x+0.5, y+0.5 };
        velocities[i] = { -y*v/std::sqrt(d2), x*v/std::sqrt(d2) };
        masses[i] = 0.01;
    }
}

void make_ring_and_planet(std::span<glm::dvec2> positions, std::span<glm::dvec2> velocities, std::span<double> masses, double r=0.5) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(-r, r);

    auto M = 100.0;
    auto m = 5.0;
    auto v = std::sqrt(M);

    positions[0]  = { 0.5, 0.5 };
    velocities[0] = { v*m/(M-m), 0.0 };
    masses[0]     = M;

    positions[1]  = { 0.5, 0.5+r*0.75 };
    velocities[1] = { -v*M/(M-m), 0.0};
    masses[1]     = m;

    for (std::size_t i=2; i<positions.size(); ++i) {
        double x, y, d2;
        do {
            x = distrib(gen);
            y = distrib(gen);
            d2 = x*x + y*y;
        } while (d2 > r*r || d2 < 0.25*r*r);
        positions[i] = { x+0.5, y+0.5 };
        velocities[i] = { -y*v/std::sqrt(d2), x*v/std::sqrt(d2) };
        masses[i] = 0.001;
    }
}

void make_disk(std::span<glm::dvec2> positions, std::span<glm::dvec2> velocities, std::span<double> masses, double r=0.5) {
    auto m = 0.01;
    auto M = m*positions.size();
    auto v = std::sqrt(M);

    for (std::size_t i=0; i<positions.size(); ++i) {
        auto pos = glm::diskRand(r);
        positions[i] = pos + 0.5;
        velocities[i] = glm::dvec2{ -pos.y, pos.x }*(v/r);
        masses[i] = m;
    }
}

void make_two_stars(std::span<glm::dvec2> positions, std::span<glm::dvec2> velocities, std::span<double> masses, double r=0.5) {
    auto m = 0.01;
    auto M = m*positions.size()*0.5;
    auto v = std::sqrt(M);
    auto disk_r = 0.1*r;

    for (std::size_t i=0; i<positions.size(); ++i) {
        auto pos = glm::diskRand(disk_r);

        positions[i] = pos;
        velocities[i] = glm::dvec2{ -pos.y, pos.x }*(v/disk_r);
        if (i%2) {
            positions[i] += glm::dvec2{ r-disk_r, 0.0 };
            velocities[i] += glm::dvec2{ 0, v*0.2 };
        } else {
            positions[i] -= glm::dvec2{ r-disk_r, 0.0 };
            velocities[i] -=  glm::dvec2{ 0, v*0.2 };
        }
        positions[i] += 0.5;
        masses[i] = m;
    }
}

template<typename T>
std::vector<glm::vec2> quadtree_lines(const QuadTree<T>& cells, int max_level=7) {
    std::vector<glm::vec2> lineVertices;
    for (auto& cell: cells.cells) {
        if (cell.level > max_level) continue;
        glm::vec2 p = glm::ldexp(glm::dvec2(cell.coords), glm::ivec2{-32});
        float cell_size = std::ldexp(1.0, -static_cast<int>(cell.level));
        lineVertices.insert(lineVertices.end(), {
            p, p+glm::vec2{cell_size, 0},
            p+glm::vec2{cell_size, 0}, p+cell_size,
            p+cell_size, p+glm::vec2{0, cell_size},
            p+glm::vec2{0, cell_size}, p
        });
    }
    return lineVertices;
}

struct Simulation {
    std::vector<std::uint64_t> keys;
    std::vector<glm::dvec2> positions;
    std::vector<glm::dvec2> velocities;
    std::vector<double> masses;
    std::vector<glm::dvec2> accelerations;
    QuadTree<std::pair<std::uint32_t, std::uint32_t>>quadtree;
    ThreadPool thread_pool;

    Simulation() : quadtree({}), thread_pool(8) {}

    void init() {
        for (std::size_t i=0; i<keys.size(); ++i) {
            keys[i] = InterleaveHash::operator()(positions[i]);
        }
        auto zipped = std::ranges::views::zip(keys, positions, velocities, masses);
        auto proj = [](const auto& v) { return std::get<0>(v); };
        quadtree = build_quadtree(zipped, proj, 20);
    }

    void update(double dt) {
        compute_acceleration_multipoles(quadtree, positions, masses, accelerations, 1e-9, [this](std::function<void()> func) {return thread_pool.submit(func); });
        std::size_t out = 0;
        for (std::size_t i=0; i<positions.size();++i) {
            velocities[i] += accelerations[i]*dt;
            accelerations[i] = {};
            positions[i] += velocities[i]*dt;
            if (positions[i].x > 0.0 && positions[i].x < 1.0 && positions[i].y > 0.0 && positions[i].y < 1.0) {
                positions[out] = positions[i];
                velocities[out] = velocities[i];
                masses[out] = masses[i];
                ++out;
            }
        }
        keys.resize(out);
        positions.resize(out);
        velocities.resize(out);
        accelerations.resize(out);
        masses.resize(out);

        init();
    }

    void resize(std::size_t n) {
        keys.resize(n);
        positions.resize(n);
        velocities.resize(n);
        masses.resize(n);
        accelerations.resize(n);
    }
};

#ifdef FMM_RENDERER_VULKAN
#include "VulkanRenderer.cpp"
#else
#include "OpenGLRenderer.cpp"
#endif

int main(void) {
    Simulation simulation;
#ifndef NDEBUG
    constexpr auto SIM_SIZE = 1000;
#else
    constexpr auto SIM_SIZE = 100000;
#endif
    simulation.resize(SIM_SIZE);
    make_two_stars(simulation.positions, simulation.velocities, simulation.masses, 0.3);
    simulation.init();

    bool step_once = false;
    int steps = 0;

#ifdef FMM_RENDERER_VULKAN
    RGFW_init("fmm", RGFW_initVulkan);
#else
    RGFW_init("fmm", RGFW_initOpenGL);
#endif

    RGFW_window* window = RGFW_createWindow("FMM", 0, 0, 1200, 1200, RGFW_windowCenter);
    if (!window) {
        return -1;
    };
    RGFW_window_setExitKey(window, RGFW_keyEscape);

    Renderer renderer(window);

    while (RGFW_window_shouldClose(window) == RGFW_FALSE) {
        RGFW_event event;
        while (RGFW_window_checkEvent(window, &event)) {
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keySpace and not event.key.repeat) {
                steps = steps>0 ? 0 : 1;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyS and not event.key.repeat) {
                step_once = true;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyQ and not event.key.repeat) {
                renderer.display_quadtree = not renderer.display_quadtree;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyRight and not event.key.repeat) {
                steps += 1;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyLeft and not event.key.repeat) {
                steps -= 1;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_key1 and not event.key.repeat) {
                simulation.resize(SIM_SIZE);
                make_two_stars(simulation.positions, simulation.velocities, simulation.masses, 0.3);
                simulation.init();
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_key2 and not event.key.repeat) {
                simulation.resize(SIM_SIZE);
                make_ring_and_planet(simulation.positions, simulation.velocities, simulation.masses, 0.4);
                simulation.init();
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_key3 and not event.key.repeat) {
                simulation.resize(SIM_SIZE);
                make_disk(simulation.positions, simulation.velocities, simulation.masses, 0.4);
                simulation.init();
            }
        }

        for (int i=0; i<steps or step_once; ++i) {
            step_once = false;
            simulation.update(0.0001);
        }

        renderer.render(simulation);
    }

    RGFW_window_close(window);
    RGFW_deinit();
    return 0;
}
