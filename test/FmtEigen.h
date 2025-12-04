/**
 * Copyright © 2022 Zen Shawn. All rights reserved.
 *
 * @file Types.h
 * @author: Zen Shawn
 * @email: xiaozisheng2008@qq.com
 * @date: 19:46:23, April 01, 2022
 */
#pragma once

// using numpy array format
#ifndef EIGEN_DEFAULT_IO_FORMAT
#define EIGEN_DEFAULT_IO_FORMAT                                                \
    Eigen::IOFormat(6, 0, ", ", ",\n", "", "", "[", "]")
#endif

#include <Eigen/Dense>

#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <numbers>

typedef Eigen::Vector<float, 1> Vec1f;
typedef Eigen::Vector<double, 1> Vec1d;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector<float, 6> Vec6f;
typedef Eigen::Vector<double, 6> Vec6d;
typedef Eigen::VectorXd VecXd;
typedef Eigen::VectorXf VecXf;

typedef Eigen::Vector3i Vec3i;

typedef Eigen::Isometry3f Isometry3f;
typedef Eigen::Isometry3d Isometry3d;

typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix4d Mat4d;
typedef Eigen::MatrixXd MatXd;
typedef Eigen::MatrixXf MatXf;

FMT_BEGIN_NAMESPACE
template <typename T, typename Char>
    requires std::is_base_of_v<Eigen::DenseBase<T>, T>
struct is_range<T, Char> {
    static constexpr const bool value = false;
};
FMT_END_NAMESPACE

template <typename T>
    requires std::derived_from<T, Eigen::DenseBase<T>>
struct fmt::formatter<T> {
    constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

    auto format(const T &m, format_context &ctx) const
    {
        static Eigen::IOFormat vecfmt(6, 0, ", ", "", "", "", "[", "]");
        if (m.rows() == 1 || m.cols() == 1) {
            return fmt::format_to(ctx.out(), "{}",
                                  fmt::streamed(m.format(vecfmt)));
        } else {
            return fmt::format_to(ctx.out(), "\n{}", fmt::streamed(m));
        }
    }
};

template <typename T>
    requires std::derived_from<T, Eigen::QuaternionBase<T>>
struct fmt::formatter<T> {
    constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
    auto format(const Eigen::QuaternionBase<T> &q, format_context &ctx) const
    {
        return fmt::format_to(ctx.out(), "Quaternion {}",
                              q.coeffs().transpose());
    }
};

template <std::floating_point Real>
struct fmt::formatter<Eigen::Transform<Real, 3, Eigen::Isometry>> {
    bool simple_format = false;

    constexpr auto parse(format_parse_context &ctx)
    {
        auto it = ctx.begin();
        if (it == ctx.end() || *it == '}') {
            return it;
        }
        if (*it++ == 's') {
            simple_format = true;
        }
        return it;
    }

    auto format(const Isometry3d &pose, format_context &ctx) const
    {
        constexpr double r2d = 180.0 / std::numbers::pi;
        if (simple_format) {
            Vec3d t = pose.translation();
            auto rpy = pose.rotation().eulerAngles(2, 1, 0);
            return fmt::format_to(
                ctx.out(),
                "XYZRPY [{:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}]", t(0),
                t(1), t(2), rpy(2) * r2d, rpy(1) * r2d, rpy(0) * r2d);
        } else {
            return fmt::format_to(ctx.out(), "\n{}", pose.matrix());
        }
    }
};
