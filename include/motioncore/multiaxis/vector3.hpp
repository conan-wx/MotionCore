#pragma once

#include <cmath>
#include <stdexcept>

namespace motioncore::multiaxis {

// 三维向量。项目内部统一使用 SI 单位（位置 m、速度 m/s 等），
// 这个类本身不携带单位，调用方应保持三个分量单位一致。
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
    [[nodiscard]] double squared_norm() const noexcept { return x * x + y * y + z * z; }
    [[nodiscard]] double norm() const noexcept { return std::sqrt(squared_norm()); }
    [[nodiscard]] double length() const noexcept { return norm(); }
    [[nodiscard]] double magnitude() const noexcept { return norm(); }
    [[nodiscard]] double dot(const Vector3& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }
    [[nodiscard]] Vector3 cross(const Vector3& other) const noexcept {
        return {y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x};
    }
    [[nodiscard]] Vector3 normalized(double tolerance = 1e-15) const {
        const double length = norm();
        if (!std::isfinite(length) || length <= tolerance) {
            throw std::invalid_argument("cannot normalize a zero or non-finite vector");
        }
        return {x / length, y / length, z / length};
    }

    Vector3& operator+=(const Vector3& rhs) noexcept {
        x += rhs.x; y += rhs.y; z += rhs.z; return *this;
    }
    Vector3& operator-=(const Vector3& rhs) noexcept {
        x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this;
    }
    Vector3& operator*=(double scalar) noexcept {
        x *= scalar; y *= scalar; z *= scalar; return *this;
    }
    Vector3& operator/=(double scalar) {
        if (!std::isfinite(scalar) || scalar == 0.0) {
            throw std::invalid_argument("vector divisor must be finite and non-zero");
        }
        x /= scalar; y /= scalar; z /= scalar; return *this;
    }
};

inline Vector3 operator+(Vector3 lhs, const Vector3& rhs) noexcept { return lhs += rhs; }
inline Vector3 operator-(Vector3 lhs, const Vector3& rhs) noexcept { return lhs -= rhs; }
inline Vector3 operator-(const Vector3& value) noexcept { return {-value.x, -value.y, -value.z}; }
inline Vector3 operator*(Vector3 value, double scalar) noexcept { return value *= scalar; }
inline Vector3 operator*(double scalar, Vector3 value) noexcept { return value *= scalar; }
inline Vector3 operator/(Vector3 value, double scalar) { return value /= scalar; }
inline bool operator==(const Vector3& lhs, const Vector3& rhs) noexcept {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}
inline bool operator!=(const Vector3& lhs, const Vector3& rhs) noexcept { return !(lhs == rhs); }

}  // namespace motioncore::multiaxis
