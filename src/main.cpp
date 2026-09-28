#include <algorithm>
#include <cmath>
#include <random>
#include <ranges>

#define GLAD_GL_IMPLEMENTATION
#include "glad/gl.h"
#define NOMINMAX
#define RGFW_IMPLEMENTATION
#define RGFW_OPENGL
#include "RGFW.h"
#undef NOMINMAX

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
        masses[i] = 0.001;
    }
}

void make_ring_and_planet(std::span<glm::dvec2> positions, std::span<glm::dvec2> velocities, std::span<double> masses, double r=0.5) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(-r, r);

    auto M = 100.0;
    auto v = std::sqrt(M);

    positions[0]  = { 0.5, 0.5 };
    velocities[0] = { v*0.01, 0.0 };
    masses[0]     = M;

    positions[1]  = { 0.5, 0.5+r*0.75 };
    velocities[1] = { -v*0.99, 0.0 };
    masses[1]     = 1.0;

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
        quadtree = build_quadtree(positions, 32, 32);
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

bool equal_approx(double a, double b, double eps=1e-10) {
    return std::abs(a - b) <= eps;
}

bool equal_approx(glm::dvec2 a, glm::dvec2 b, double eps=1e-10) {
    return equal_approx(a.x, b.x, eps) && equal_approx(a.y, b.y, eps);
}

int main(void) {
    Simulation simulation;
    simulation.resize(12000);
    make_ring_and_planet(simulation.positions, simulation.velocities, simulation.masses, 0.4);
    simulation.init();

    bool update_simulation = false;
    bool step_once = false;
    bool display_quad_tree = false;

    RGFW_init("fmm", RGFW_initOpenGL);

    RGFW_window* window = RGFW_createWindow("FMM", 0, 0, 1200, 1200, RGFW_windowCenter | RGFW_windowNoResize | RGFW_windowOpenGL);
    if (!window) {
        return -1;
    }
    RGFW_window_setExitKey(window, RGFW_keyEscape);
    RGFW_window_makeCurrentContext_OpenGL(window);

    if (!gladLoadGL((GLADloadfunc)RGFW_getProcAddress_OpenGL)) {
        return -1;
    }

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    unsigned int pointsArrayObject, pointsBufferObject, quadTreeArrayObject, quadTreeBufferObject;
    glGenVertexArrays(1, &pointsArrayObject);
    glGenBuffers(1, &pointsBufferObject);

    glBindVertexArray(pointsArrayObject);
    glBindBuffer(GL_ARRAY_BUFFER, pointsBufferObject);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &quadTreeArrayObject);
    glGenBuffers(1, &quadTreeBufferObject);
    glBindVertexArray(quadTreeArrayObject);
    glBindBuffer(GL_ARRAY_BUFFER, quadTreeBufferObject);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    const GLchar* vertexShaderSource =
    R"(#version 330 core
    layout(location = 0) in vec2 aPos;
    void main() {
        gl_Position = vec4(2*aPos-vec2(1,1), 0.0, 1.0);
        gl_PointSize = 1.2;
    })";
    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    const GLchar* fragmentShaderSource =
    R"(#version 330 core
    uniform vec4 inColor;
    out vec4 FragColor;
    void main() {
        FragColor = inColor;
    })";
    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    unsigned int shaderProgram;
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    int colorUniformLocation = glGetUniformLocation(shaderProgram, "inColor");

    std::vector<float> points;
    while (RGFW_window_shouldClose(window) == RGFW_FALSE) {
        RGFW_event event;
        while (RGFW_window_checkEvent(window, &event)) {
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keySpace and not event.key.repeat) {
                update_simulation = not update_simulation;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyS and not event.key.repeat) {
                step_once = true;
            }
            if (event.type == RGFW_keyPressed and event.key.value == RGFW_keyQ and not event.key.repeat) {
                display_quad_tree = not display_quad_tree;
            }
        }


        if (update_simulation or step_once) {
            step_once = false;
            simulation.update(0.0001);
        }
        points.resize(simulation.positions.size()*2);
        for (std::size_t i=0; i<simulation.positions.size(); ++i) {
            points[2*i] = simulation.positions[i].x;
            points[2*i+1] = simulation.positions[i].y;
        }
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (display_quad_tree) {
            std::vector<float> lineVertices = quadtree_lines(simulation.quadtree);
            glBindVertexArray(quadTreeArrayObject);
            glBindBuffer(GL_ARRAY_BUFFER, quadTreeBufferObject);
            glBufferData(GL_ARRAY_BUFFER, lineVertices.size() * sizeof(float), lineVertices.data(), GL_DYNAMIC_DRAW);
            glLineWidth(1.0f);
            glUseProgram(shaderProgram);
            glUniform4f(colorUniformLocation, 0.0f, 0.0f, 0.6f, 1.0f);
            glDrawArrays(GL_LINES, 0, lineVertices.size()/2);
        }

        glBindVertexArray(pointsArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, pointsBufferObject);
        glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(float), points.data(), GL_DYNAMIC_DRAW);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glUseProgram(shaderProgram);
        glUniform4f(colorUniformLocation, 1.0f, 1.0f, 1.0f, 1.0f);
        glDrawArrays(GL_POINTS, 0, points.size());

        RGFW_window_swapBuffers_OpenGL(window);
    }

    RGFW_window_close(window);
    RGFW_deinit();
    return 0;
}
