#include <random>

#include "grtest.h"
#include "Multipole.hpp"


bool equal_approx(double a, double b, double eps=1e-10) {
    return std::abs(a - b) <= eps;
}

bool equal_approx(std::complex<double> a, std::complex<double> b, double eps=1e-10) {
    return equal_approx(a.real(), b.real(), eps) && equal_approx(a.imag(), b.imag(), eps);
}

bool equal_approx(glm::dvec2 a, glm::dvec2 b, double eps=1e-10) {
    return equal_approx(a.x, b.x, eps) && equal_approx(a.y, b.y, eps);
}

TEST_CASE(test_calculate_and_evaluate_multipole) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    auto charge = distrib(gen);
    auto multipole = calculate_multipole<32>(charge, glm::dvec2{});
    assert(multipole.q == charge);
    for (int i=0; i<multipole.a.size(); ++i) {
        assert(multipole.a[i] == 0.0);
    }

    std::array<glm::dvec2, 16> tests;
    std::array<glm::dvec2, 16> forces;
    for (int i=0; i<16; ++i) {
        auto test = std::polar(distrib(gen)+2.0, distrib(gen)*6.283);
        tests[i] = glm::dvec2{ test.real(), test.imag() };
        auto dist = tests[i].x*tests[i].x + tests[i].y*tests[i].y;
        forces[i] = glm::dvec2{ -charge*tests[i].x/dist, -charge*tests[i].y/dist };
        auto f = evaluate_multipole(multipole, tests[i]);
        assert(equal_approx(f, forces[i], 1e-9));
    }

    for (int j=0; j<16; ++j) {
        charge = distrib(gen);
        auto src = std::polar(distrib(gen), distrib(gen)*6.283);
        multipole += calculate_multipole<multipole.a.size()>(charge, glm::dvec2{src.real(), src.imag()});
        for (int i=0; i<16; ++i) {
            auto dr = tests[i] - glm::dvec2{ src.real(), src.imag() };
            auto dist = dr.x*dr.x + dr.y*dr.y;
            forces[i] += glm::dvec2{ -charge*dr.x/dist, -charge*dr.y/dist };
            auto f = evaluate_multipole(multipole, tests[i]);
            assert(equal_approx(f, forces[i], 1e-6));
        }
    }
}

TEST_CASE(test_translate_multipole) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    auto multipole = calculate_multipole<32>(distrib(gen), glm::dvec2{});
    for (int i=0; i<16; ++i) {
        auto dst = std::polar(1.0, distrib(gen)*6.283);
        auto translated = translate_multipole(multipole, glm::dvec2{-dst.real(), -dst.imag()});
        for (int j=0; j<16; ++j) {
            auto dr = std::polar(distrib(gen)+4.0, distrib(gen)*6.283);
            auto f1 = evaluate_multipole(multipole, glm::dvec2{ dst.real()+dr.real(), dst.imag()+dr.imag() });
            auto f2 = evaluate_multipole(translated, glm::dvec2{ dr.real(), dr.imag() });
            assert(equal_approx(f1, f2, 1e-6));
        }
    }

    for (int i=0; i<16; ++i) {
        auto src = std::polar(distrib(gen), distrib(gen)*6.283);
        multipole += calculate_multipole<multipole.a.size()>(distrib(gen), glm::dvec2{ src.real(), src.imag() });
    }
    for (int i=0; i<16; ++i) {
        auto dst = std::polar(1.0, distrib(gen)*6.283);
        auto translated = translate_multipole(multipole, glm::dvec2{ -dst.real(), -dst.imag() });
        for (int j=0; j<16; ++j) {
            auto dr = std::polar(distrib(gen)+4.0, distrib(gen)*6.283);
            auto f1 = evaluate_multipole(multipole, glm::dvec2{ dst.real()+dr.real(), dst.imag()+dr.imag() });
            auto f2 = evaluate_multipole(translated, glm::dvec2{ dr.real(), dr.imag() });
            assert(equal_approx(f1, f2, 1e-6));
        }
    }
}

TEST_CASE(test_convert_to_local_and_evaluate) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    auto multipole = calculate_multipole<32>(distrib(gen), glm::dvec2{});

    for (int i=0; i<16; ++i) {
        auto dst = std::polar(distrib(gen)+3.0, distrib(gen)*6.283);
        auto local = convert_to_local(multipole, glm::dvec2{ -dst.real(), -dst.imag() });
        auto f1 = evaluate_multipole(multipole, glm::dvec2{ dst.real(), dst.imag() });
        auto f2 = evaluate_local(local, glm::dvec2{});
        assert(equal_approx(f1, f2, 1e-6));
        for (int j=0; j<16; ++j) {
            auto dr = std::polar(distrib(gen), distrib(gen)*6.283);
            auto f3 = evaluate_multipole(multipole, glm::dvec2{ dst.real()+dr.real(), dst.imag()+dr.imag() });
            auto f4 = evaluate_local(local, glm::dvec2{ dr.real(), dr.imag() });
            assert(equal_approx(f3, f4, 1e-6));
        }
    }

    for (int i=0; i<16; ++i) {
        auto src = std::polar(distrib(gen), distrib(gen)*6.283);
        multipole += calculate_multipole<multipole.a.size()>(distrib(gen), glm::dvec2{ src.real(), src.imag() });
    }

    for (int i=0; i<16; ++i) {
        auto dst = std::polar(distrib(gen)+3.0, distrib(gen)*6.283);
        auto local = convert_to_local(multipole, glm::dvec2{ -dst.real(), -dst.imag() });
        auto f1 = evaluate_multipole(multipole, glm::dvec2{ dst.real(), dst.imag() });
        auto f2 = evaluate_local(local, glm::dvec2{});
        assert(equal_approx(f1, f2, 1e-6));
        for (int j=0; j<16; ++j) {
            auto dr = std::polar(distrib(gen), distrib(gen)*6.283);
            auto f3 = evaluate_multipole(multipole, glm::dvec2{ dst.real()+dr.real(), dst.imag()+dr.imag() });
            auto f4 = evaluate_local(local, glm::dvec2{ dr.real(), dr.imag() });
            assert(equal_approx(f3, f4, 1e-6));
        }
    }
}

TEST_CASE(test_charge_to_local) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    for (int i=0; i<16; ++i) {
        auto charge = distrib(gen);
        auto multipole = calculate_multipole<32>(charge, glm::dvec2{});
        auto dst = std::polar(distrib(gen)+3.0, distrib(gen)*6.283);
        auto local = convert_to_local(multipole, glm::dvec2{ -dst.real(), -dst.imag() });
        auto local2 = charge_to_local<32>(charge, glm::dvec2{ -dst.real(), -dst.imag() });
        for (int j=0; j<32; ++j) {
            assert(equal_approx(local[j], local2[j]));
        }
    }
}

TEST_CASE(test_translate_local) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    auto multipole = calculate_multipole<32>(distrib(gen), glm::dvec2{});

    auto local_dst = std::polar(distrib(gen)+3.0, distrib(gen)*6.283);
    auto local = convert_to_local(multipole, glm::dvec2{ -local_dst.real(), -local_dst.imag() });
    for (int i=0; i<16; ++i) {
        auto dst = std::polar(distrib(gen)*0.4, distrib(gen)*6.283);
        auto translated = translate_local(local, glm::dvec2(-dst.real(), -dst.imag()));
        auto f1 = evaluate_local(local, glm::dvec2{ dst.real(), dst.imag() });
        auto f2 = evaluate_local(translated, glm::dvec2{});
        assert(equal_approx(f1, f2, 1e-9));
        for (int j=0; j<16; ++j) {
            auto dr = std::polar(distrib(gen)*0.4, distrib(gen)*6.283);
            auto f3 = evaluate_local(local, glm::dvec2{ dst.real()+dr.real(), dst.imag()+dr.imag() });
            auto f4 = evaluate_local(translated, glm::dvec2{ dr.real(), dr.imag() });
            assert(equal_approx(f3, f4, 1e-9));
        }
    }

    for (int i=0; i<16; ++i) {
        auto src = std::polar(distrib(gen), distrib(gen)*6.283);
        multipole += calculate_multipole<multipole.a.size()>(distrib(gen), glm::dvec2{ src.real(), src.imag() });
    }
    
    local = convert_to_local(multipole, glm::dvec2{ -local_dst.real(), -local_dst.imag() });
    for (int i=0; i<16; ++i) {
        auto dst = std::polar(distrib(gen)*0.4, distrib(gen)*6.283);
        auto translated = translate_local(local, glm::dvec2(-dst.real(), -dst.imag()));
        auto f1 = evaluate_local(local, glm::dvec2{ dst.real(), dst.imag() });
        auto f2 = evaluate_local(translated, glm::dvec2{});
        assert(equal_approx(f1, f2, 1e-9));
        for (int j=0; j<16; ++j) {
            auto dr = std::polar(distrib(gen)*0.4, distrib(gen)*6.283);
            auto f3 = evaluate_local(local, glm::dvec2{ dst.real()+dr.real(), dst.imag()+dr.imag() });
            auto f4 = evaluate_local(translated, glm::dvec2{ dr.real(), dr.imag() });
            assert(equal_approx(f3, f4, 1e-9));
        }
    }
}