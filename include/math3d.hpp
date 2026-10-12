#ifndef PROJECT3_MATH3D_HPP
#define PROJECT3_MATH3D_HPP

#include <array>
#include <cstddef>

namespace math3d {

using Vector3 = std::array<double, 3>;

Vector3 add(const Vector3& a, const Vector3& b);
Vector3 subtract(const Vector3& a, const Vector3& b);
Vector3 scale(const Vector3& v, double factor);
double dot(const Vector3& a, const Vector3& b);
double norm(const Vector3& v);
Vector3 normalized(const Vector3& v);

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

Quaternion normalized(const Quaternion& q);
Quaternion multiply(const Quaternion& a, const Quaternion& b);
Quaternion from_rpy(const Vector3& rpy);
Quaternion from_axis_angle(const Vector3& axis, double angle);

// Matrices are column-major: element(row, column) is data[4 * column + row].
struct Matrix4 {
    std::array<double, 16> data{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0};

    double operator()(std::size_t row, std::size_t column) const {
        return data[4 * column + row];
    }
    double& operator()(std::size_t row, std::size_t column) {
        return data[4 * column + row];
    }
};

Matrix4 from_translation_rotation(const Vector3& translation, const Quaternion& rotation);
Matrix4 from_origin(const Vector3& xyz, const Vector3& rpy);
Matrix4 multiply(const Matrix4& parent, const Matrix4& child);
Vector3 translation_of(const Matrix4& transform);
Quaternion rotation_of(const Matrix4& transform);
Vector3 transform_point(const Matrix4& transform, const Vector3& point);

} // namespace math3d

#endif
