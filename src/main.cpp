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

#include "Vec2.hpp"
#include "QuadTree.hpp"
#include "simulation.hpp"


void populate_system(std::span<Vec2d> positions, std::span<Vec2d> velocities, double r=0.5) {
    double density = std::sqrt(positions.size())*0.3;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(-r, r);
    Vec2d system_vel{};
    for (std::size_t i=0; i<positions.size(); ++i) {
        double x, y;
        do {
            x = distrib(gen)/5.0;
            y = distrib(gen);
        } while (x*x + y*y > r*r);
        positions[i] = {x+0.5, y+0.5};
        velocities[i] = {-y*density/r, x*density/r};
        system_vel += velocities[i];
    }
    for (auto& vel: velocities) {
        vel -= system_vel/positions.size();
    }
}

std::vector<float> quadtree_lines(const QuadTree<std::span<const Vec2d>>& cells) {
    std::vector<float> lineVertices;
    for (auto& cell: cells.cells) {
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
    std::vector<Vec2d> positions;
    std::vector<Vec2d> velocities;
    std::vector<double> masses;
    std::vector<Vec2d> accelerations;
    QuadTree<std::span<const Vec2d>>quadtree;

    Simulation() : quadtree({}) {}

    void init() {
        auto zipped = std::ranges::views::zip(positions, velocities, masses);
        std::ranges::sort(zipped, [](const auto& lhs, const auto& rhs) {
            return cmp_zcurve_bitmagic(std::get<0>(lhs), std::get<0>(rhs));
        });
        quadtree = build_quadtree(positions, 32, 16);
    }

    void update(double dt) {
        compute_acceleration_multipoles(quadtree, positions, masses, accelerations);
        for (int i=0; i<velocities.size(); i++) {
            velocities[i] += accelerations[i]*dt;
            accelerations[i] = {};
            positions[i] += velocities[i]*dt;
        }

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

bool equal_approx(Vec2d a, Vec2d b, double eps=1e-10) {
    return equal_approx(a.x, b.x, eps) && equal_approx(a.y, b.y, eps);
}

int main(void) {
    Simulation simulation;
    simulation.resize(10000);
    std::fill(simulation.masses.begin(), simulation.masses.end(), 1.0);
    populate_system(simulation.positions, simulation.velocities, 0.3);
    simulation.init();

    //std::random_device rd;
    //std::mt19937 gen(rd());
    //std::uniform_real_distribution<double> distrib(0, 1);

    //bool found_bad = false;
    //for (int q=3; q < 6 and not found_bad; ++q) {
    //    simulation.resize(q);
    //    std::fill(simulation.masses.begin(), simulation.masses.end(), 1.0);
    //    for (int k=0; k<100000 and not found_bad; ++k) {
    //        for (std::size_t i=0; i<simulation.positions.size(); ++i) {
    //            simulation.positions[i] = { distrib(gen), distrib(gen) };
    //        }
    //        std::sort(simulation.positions.begin(), simulation.positions.end(), static_cast<bool(*)(const Vec2d&, const Vec2d&)>(cmp_zcurve_bitmagic));
    //        std::fill(simulation.accelerations.begin(), simulation.accelerations.end(), Vec2d{ 0.0, 0.0 });
    //        auto accelerations_exepected = simulation.accelerations;
    //        compute_acceleration_direct(simulation.positions, simulation.masses, accelerations_exepected);
    //        simulation.quadtree = build_quadtree(simulation.positions, 16, 1);
    //        compute_acceleration_multipoles(simulation.quadtree, simulation.positions, simulation.masses, simulation.accelerations);
    //        for (int i=0; i<simulation.accelerations.size(); ++i) {
    //            if (not equal_approx(simulation.accelerations[i], accelerations_exepected[i], 1)) {
    //                found_bad = true;
    //                break;
    //            }
    //        }
    //    }
    //}

    bool update_simulation = true;
    bool display_quad_tree = false;

    RGFW_init("fmm", RGFW_initOpenGL);

    RGFW_window* window = RGFW_createWindow("FMM", 0, 0, 1280, 960, RGFW_windowCenter | RGFW_windowNoResize | RGFW_windowOpenGL);
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

    std::vector<float> points(simulation.positions.size()*2);
    while (RGFW_window_shouldClose(window) == RGFW_FALSE) {
        if (update_simulation) {
            simulation.update(0.0001);
        }
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
        RGFW_pollEvents();
    }

    RGFW_window_close(window);
    RGFW_deinit();
    return 0;
}
