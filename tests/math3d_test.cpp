#include "math3d.hpp"

#include <cassert>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace {

constexpr double tolerance = 1e-10;
constexpr double pi = std::numbers::pi_v<double>;

void near(double actual, double expected, double limit = tolerance) {
    assert(std::abs(actual - expected) <= limit);
}

void near_vector(const math3d::Vector3& actual, const math3d::Vector3& expected) {
    for (int i = 0; i < 3; ++i) near(actual[i], expected[i]);
}

void valid_rotation(const math3d::Matrix4& m) {
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            double product = 0.0;
            for (int row = 0; row < 3; ++row) product += m(row, a) * m(row, b);
            near(product, a == b ? 1.0 : 0.0);
        }
    }
    double determinant =
        m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
        m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
        m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    near(determinant, 1.0);
    near(m(3, 0), 0.0);
    near(m(3, 1), 0.0);
    near(m(3, 2), 0.0);
    near(m(3, 3), 1.0);
}

void same_rotation(const math3d::Matrix4& a, const math3d::Matrix4& b) {
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            near(a(row, column), b(row, column));
        }
    }
}

} // namespace

int main() {
    using namespace math3d;
    near_vector(add({1, 2, 3}, {4, 5, 6}), {5, 7, 9});
    near_vector(subtract({4, 5, 6}, {1, 2, 3}), {3, 3, 3});
    near_vector(scale({1, 2, 3}, 2), {2, 4, 6});
    near(dot({1, 2, 3}, {4, 5, 6}), 32);
    near(norm({3, 4, 0}), 5);
    near_vector(normalized(Vector3{0, 0, 2}), {0, 0, 1});
    bool rejected_zero = false;
    try { normalized(Vector3{0, 0, 0}); }
    catch (const std::invalid_argument&) { rejected_zero = true; }
    assert(rejected_zero);

    Matrix4 translated = from_translation_rotation({1, 2, 3}, {});
    near_vector(translation_of(translated), {1, 2, 3});
    near_vector(transform_point(translated, {4, 5, 6}), {5, 7, 9});
    assert(translated.data[12] == 1 && translated.data[13] == 2 && translated.data[14] == 3);

    Matrix4 roll = from_origin({0, 0, 0}, {pi / 2, 0, 0});
    Matrix4 pitch = from_origin({0, 0, 0}, {0, pi / 2, 0});
    Matrix4 yaw = from_origin({0, 0, 0}, {0, 0, pi / 2});
    near_vector(transform_point(roll, {0, 1, 0}), {0, 0, 1});
    near_vector(transform_point(pitch, {0, 0, 1}), {1, 0, 0});
    near_vector(transform_point(yaw, {1, 0, 0}), {0, 1, 0});
    near_vector(transform_point(from_origin({0, 0, 0}, {pi / 2, pi / 2, 0}), {0, 1, 0}),
                {1, 0, 0});

    Matrix4 general = from_translation_rotation(
        {0, 0, 0}, from_axis_angle({1, 1, 1}, 2 * pi / 3));
    near_vector(transform_point(general, {1, 0, 0}), {0, 1, 0});
    valid_rotation(general);

    Matrix4 parent = from_origin({1, 2, 3}, {0, 0, pi / 2});
    Matrix4 child = from_origin({2, 0, 0}, {pi / 2, 0, 0});
    Matrix4 composed = multiply(parent, child);
    near_vector(translation_of(composed), {1, 4, 3});
    near_vector(transform_point(composed, {0, 1, 0}), {1, 4, 4});
    same_rotation(composed, from_translation_rotation({1, 4, 3},
                  multiply(from_rpy({0, 0, pi / 2}), from_rpy({pi / 2, 0, 0}))));
    valid_rotation(composed);

    for (const Vector3 axis : {Vector3{1, 0, 0}, Vector3{0, 1, 0},
                               Vector3{0, 0, 1}, Vector3{1, 2, 3}}) {
        for (double angle : {pi, pi - 1e-8, pi + 1e-8}) {
            Matrix4 original = from_translation_rotation({0, 0, 0},
                                                          from_axis_angle(axis, angle));
            Quaternion recovered = rotation_of(original);
            near(std::hypot(std::hypot(recovered.x, recovered.y),
                            std::hypot(recovered.z, recovered.w)), 1.0);
            Matrix4 round_trip = from_translation_rotation({0, 0, 0}, recovered);
            same_rotation(original, round_trip);
            valid_rotation(round_trip);
        }
    }
}
