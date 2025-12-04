#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, 0, ", ")

#include <spdlog/spdlog.h>
#include <fmt/ostream.h>


#include "FmtEigen.h"

#include <hexapod_kinematics/hexapod_kinematics.h>

#include <numbers>

using std::numbers::pi;

constexpr double DegToRad(double deg)
{
    return deg / 180.0 * pi;
}

struct MyHexapodConfig
{
    double angle_large = DegToRad(25.0);
    double angle_small = DegToRad(25.0);
    double radius_large = 0.046;
    double radius_small = 0.046;

    operator hexkins::HexapodConfig()
    {
        std::array<Eigen::Vector3d, 6> base_joints;
        std::array<Eigen::Vector3d, 6> platform_joints;
        for (int i = 0; i < 3; i++)
        {
            double theta = (pi * 2 / 3 - angle_large) / 2.0;
            double a = theta + i * pi * 2.0 / 3.0;
            base_joints[2 * i] =
                Eigen::Vector3d{radius_large * std::cos(a),
                                radius_large * std::sin(a),
                                0};
            a += angle_large;
            base_joints[2 * i + 1] =
                Eigen::Vector3d{radius_large * std::cos(a),
                                radius_large * std::sin(a),
                                0};
            a = angle_small / 2 + i * pi * 2.0 / 3.0;
            platform_joints[2 * i] =
                Eigen::Vector3d{radius_small * std::cos(a),
                                radius_small * std::sin(a),
                                0};
            a += (2 * pi / 3 - angle_small);
            platform_joints[2 * i + 1] =
                Eigen::Vector3d{radius_small * std::cos(a),
                                radius_small * std::sin(a),
                                0};
        }

        return hexkins::HexapodConfig{
            .base_joints = base_joints,
            .platform_joints = platform_joints,
        };
    }
};

void Test(hexkins::HexapodConfig config, double init_strut, std::array<double, 6> target_qs)
{
    using namespace Eigen;
    using namespace hexkins;
    using Vector6d = Vector<double, 6>;
    std::array<double, 6> qs;
    qs.fill(init_strut);
    Vector3d pos{0, 0, init_strut};
    Quaterniond quat;
    quat.setIdentity();
    std::tie(pos, quat) = forward_kinematics(config, qs, pos, quat);
    Vector6d qs0 = Map<Vector6d>(qs.data());
    Vector6d qs1 = Map<Vector6d>(target_qs.data());
    double length = (qs1 - qs0).norm();
    double ds = 0.1;
    double s = 0.0;
    while (s < length)
    {
        Map<Vector6d>(qs.data()) = qs0 + (qs1 - qs0) * s / length;
        std::tie(pos, quat) = forward_kinematics(config, qs, pos, quat);
        s += ds;
    }
    qs = target_qs;
    std::tie(pos, quat) = forward_kinematics(config, qs, pos, quat);
    SPDLOG_INFO("qs: {}, pos: {}, ori: {}", qs, pos, quaternion_to_rpy(quat) / pi * 180);
    qs = inverse_kinematics(config, pos, quat);
    SPDLOG_DEBUG("pos: {}, ori: {}, qs: {}", pos, quaternion_to_rpy(quat) / pi * 180, qs);
}

int main()
{
    MyHexapodConfig my_config{
        .angle_large = DegToRad(25.0),
        .angle_small = DegToRad(25.0),
        .radius_large = 0.046,
        .radius_small = 0.046,
    };
    hexkins::HexapodConfig config = my_config;

    try
    {
        std::array<double, 6> joints;
        joints.fill(0.118);
        Test(config, 0.118, joints);
        for (int i = 0; i < 6; i++)
        {
            SPDLOG_INFO("i: {}", i);
            joints.fill(0.118);
            joints[i] = 0.128;
            Test(config, 0.118, joints);
        }

        using namespace Eigen;
        // joints.fill(0.118);
        Eigen::Vector3d current_pos;
        current_pos << -0.031503, -0.057208, 0.09709;
        Eigen::Quaterniond current_orient;
        Vector3d rpy = Vector3d(-0.327, -11.370, -17.619) / 180.0 * pi;
        current_orient = hexkins::rpy_to_quaternion(rpy.reverse());
        joints = hexkins::inverse_kinematics(config, current_pos, current_orient);
        SPDLOG_INFO("ik, target pos: {}, target ori: {}, joints: {}", current_pos, current_orient.coeffs(), joints);
        // spdlog::info("joints: {}, seed pos: {}, seed ori: {}", joints, current_pos, current_orient.coeffs());
        // auto [pos, ori] = hexkins::forward_kinematics(config, joints, current_pos, current_orient);
        // spdlog::info("fk, joints: {}, pos: {}, ori: {}", joints, pos, ori.coeffs());

        // current_pos << 0, 0, 0.120;
        // current_orient.setIdentity();
    }
    catch (const std::exception &e)
    {
        spdlog::error("{}", e.what());
    }

    spdlog::info("done");
    return 0;
}