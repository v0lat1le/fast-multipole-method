#include <random>

#include "grtest.h"
#include "Multipole.hpp"


bool equal_approx(double a, double b, double eps=1e-10) {
    return std::abs(a - b) <= eps;
}

bool equal_approx(std::complex<double> a, std::complex<double> b, double eps=1e-10) {
    return equal_approx(a.real(), b.real(), eps) && equal_approx(a.imag(), b.imag(), eps);
}

bool equal_approx(Vec2d a, Vec2d b, double eps=1e-10) {
    return equal_approx(a.x, b.x, eps) && equal_approx(a.y, b.y, eps);
}

TEST_CASE(test_translate_multipole) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    Multipole<16> original;
    original.q = distrib(gen);
    for (int k=0; k<original.a.size(); ++k) {
        original.a[k] = std::complex<double>(distrib(gen), distrib(gen));
    }

    Vec2d src = { distrib(gen), distrib(gen) };

    auto translated = translate_multipole(original, src);
    auto untranslated = translate_multipole(translated, Vec2d{}-src);

    assert(equal_approx(original.q, untranslated.q));
    for (int k=0; k<original.a.size(); ++k) {
        assert(equal_approx(original.a[k], untranslated.a[k], 1e-6));
        assert(not equal_approx(original.a[k], translated.a[k], 1e-6));
    }
}

TEST_CASE(test_translate_local) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    Local<16> original;
    for (int k=0; k<16; ++k) {
        original[k] = std::complex<double>(distrib(gen), distrib(gen));
    }

    Vec2d src = { distrib(gen), distrib(gen) };

    auto translated = translate_local(original, src);
    auto untranslated = translate_local(translated, Vec2d{}-src);

    bool all_same = true;
    for (int k=0; k<16; ++k) {
        assert(equal_approx(original[k], untranslated[k], 1e-6));
        all_same = all_same and equal_approx(original[k], translated[k], 1e-6);
    }
    assert(not all_same);
}

TEST_CASE(test_convert_to_local) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distrib(0, 1);

    Multipole<16> original{};
    original.q = distrib(gen);
    for (int k=0; k<original.a.size(); ++k) {
        original.a[k] = std::complex<double>(distrib(gen), distrib(gen));
    }

    Vec2d src = { 20.0, 20.0 };  // TODO: this needs to be in the convergence window, probably init multipole deterministically

    auto converted = convert_to_local(original, src);

    Vec2d expected = evaluate_multipole(original, src);
    Vec2d actual = evaluate_local(converted, Vec2d{});
    assert(equal_approx(expected, actual, 1e-3));
}