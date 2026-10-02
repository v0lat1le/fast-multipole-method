#include <algorithm>
#include <cmath>
#include <random>
#include <ranges>
#include <memory>

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
    auto M = 0.01*positions.size()*0.5;
    auto v = std::sqrt(M);
    auto disk_r = 0.4*r;

    for (std::size_t i=0; i<positions.size(); ++i) {
        auto pos = glm::diskRand(disk_r);

        positions[i] = pos;
        velocities[i] = glm::dvec2{ -pos.y, pos.x }*(v/disk_r);
        if (i%2) {
            positions[i] += glm::dvec2{ r-disk_r, 0.0 };
            auto w = positions[i]+(r-disk_r);
            velocities[i] += glm::dvec2{ -w.y, w.x}*(v/w.length());
        } else {
            positions[i] -= glm::dvec2{ r-disk_r, 0.0 };
            auto w = positions[i]-(r-disk_r);
            velocities[i] += glm::dvec2{ -w.y, w.x }*(v/w.length());
        }
        positions[i] += 0.5;
        masses[i] = 0.01;
    }
}

std::vector<float> quadtree_lines(const QuadTree<std::span<const glm::dvec2>>& cells, int max_level=7) {
    std::vector<float> lineVertices;
    for (auto& cell: cells.cells) {
        if (cell.level > max_level) continue;
        float x = std::ldexp(cell.coords.x, -32);
        float y = std::ldexp(cell.coords.y, -32);
        float cell_size = std::ldexp(1.0, -static_cast<int>(cell.level));
        lineVertices.insert(lineVertices.end(), {
            x, y, x + cell_size, y,
            x + cell_size, y, x + cell_size, y + cell_size,
            x + cell_size, y + cell_size, x, y + cell_size,
            x, y + cell_size, x, y
            });
    }
    return lineVertices;
}

struct Simulation {
    std::vector<glm::dvec2> positions;
    std::vector<glm::dvec2> velocities;
    std::vector<double> masses;
    std::vector<glm::dvec2> accelerations;
    QuadTree<std::span<const glm::dvec2>>quadtree;

    Simulation() : quadtree({}) {}

    void init() {
        auto zipped = std::ranges::views::zip(positions, velocities, masses);
        std::ranges::sort(zipped, [](const auto& lhs, const auto& rhs) {
            return cmp_zcurve_bitmagic(std::get<0>(lhs), std::get<0>(rhs));
        });

        //std::size_t bad = 0;
        //for (std::size_t i=positions.size()-1; i>0; --i) {
        //    for (std::size_t j=i-1; j>1; --j) {
        //        auto dr = positions[i]-positions[j];
        //        if (dr.x*dr.x + dr.y*dr.y > 1e-6) {
        //            i = j-1;
        //            break;
        //        }
        //        velocities[i] = (velocities[j]*masses[j] + velocities[i]*masses[i])/(masses[j]+masses[i]);
        //        masses[i] += masses[j];
        //        bad++;
        //        std::swap(positions[j], positions[positions.size()-bad]);
        //        std::swap(velocities[j], velocities[velocities.size()-bad]);
        //        std::swap(masses[j], masses[masses.size()-bad]);
        //    }
        //}
        //positions.resize(positions.size()-bad);
        //velocities.resize(velocities.size()-bad);
        //accelerations.resize(accelerations.size()-bad);
        //masses.resize(masses.size()-bad);
        //zipped = std::ranges::views::zip(positions, velocities, masses);

        std::ranges::sort(zipped, [](const auto& lhs, const auto& rhs) {
            return cmp_zcurve_bitmagic(std::get<0>(lhs), std::get<0>(rhs));
        });
        quadtree = build_quadtree(positions, 32, 20);
    }

    void update(double dt) {
        compute_acceleration_multipoles(quadtree, positions, masses, accelerations);
        std::size_t bad = 0;
        for (std::size_t i=positions.size(); i-->0;) {
            velocities[i] += accelerations[i]*dt;
            accelerations[i] = {};
            positions[i] += velocities[i]*dt;
            if (positions[i].x <= 0.0 || positions[i].x >= 1.0 || positions[i].y <= 0.0 || positions[i].y >= 1.0) {
                bad++;
                std::swap(positions[i], positions[positions.size()-bad]);
                std::swap(velocities[i], velocities[velocities.size()-bad]);
                std::swap(masses[i], masses[masses.size()-bad]);
            }
        }
        positions.resize(positions.size()-bad);
        velocities.resize(velocities.size()-bad);
        accelerations.resize(accelerations.size()-bad);
        masses.resize(masses.size()-bad);

        init();
    }

    void resize(std::size_t n) {
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
    simulation.resize(1000);
#else
    simulation.resize(20000);
#endif
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
                renderer.display_quad_tree = not renderer.display_quad_tree;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyRight and not event.key.repeat) {
                steps += 1;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyLeft and not event.key.repeat) {
                steps -= 1;
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
