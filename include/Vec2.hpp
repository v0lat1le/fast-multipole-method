#pragma once

#include <cstdint>


template<typename T>
struct Vec2 {
    T x;
    T y;
    bool operator==(const Vec2&) const = default;
    constexpr Vec2& operator-=(const Vec2& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    friend constexpr Vec2 operator-(Vec2 lhs, const Vec2& rhs) noexcept {
        lhs -= rhs;
        return lhs;
    }
    constexpr Vec2& operator+=(const Vec2& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    friend Vec2 operator+(Vec2 lhs, const Vec2& rhs) noexcept {
        lhs += rhs;
        return lhs;
    }
    constexpr Vec2& operator*=(const T& rhs) noexcept {
        x *= rhs;
        y *= rhs;
        return *this;
    }
    friend constexpr Vec2 operator*(Vec2 lhs, const T& rhs) noexcept {
        lhs *= rhs;
        return lhs;
    }
    constexpr Vec2& operator/=(const T& rhs) noexcept {
        x /= rhs;
        y /= rhs;
        return *this;
    }
    friend constexpr Vec2 operator/(Vec2 lhs, const T& rhs) noexcept {
        lhs /= rhs;
        return lhs;
    }
};
using Vec2i = Vec2<std::uint32_t>;
using Vec2d = Vec2<double>;
