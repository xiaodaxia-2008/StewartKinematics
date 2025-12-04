// Copyright (c) 2021 Conroy Cheers

#ifndef HEXAPOD_KINEMATICS__HEXAPOD_KINEMATICS_H_
#define HEXAPOD_KINEMATICS__HEXAPOD_KINEMATICS_H_

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <string>

namespace hexkins
{
  class Exception : public std::exception
  {
  public:
    Exception(int error_code, const std::string &message) noexcept;
    virtual ~Exception() = default;
    const char *what() const noexcept override;

  private:
    int error_code;
    std::string message;
  };

  class ConvergenceFailure : public Exception
  {
  public:
    explicit ConvergenceFailure(const std::string &message) noexcept;
    virtual ~ConvergenceFailure() = default;
  };

#define HEXKINS_INVALID_JOINT_LENGTH 0
#define HEXKINS_MAX_ERROR_EXCEEDED 1
#define HEXKINS_TOO_MANY_ITERATIONS 2
#define HEXKINS_ZERO_STRUT_VECTOR 3

  struct HexapodConfig
  {
    /// Base plane joint positions in the base coordinate system (unit: meters is suggested)
    /// Arranged in counter-clockwise order, with the first point being the one closest to the +x axis and located in the +x and +y quadrant.
    std::array<Eigen::Vector3d, 6> base_joints;

    /// Simuliar to @ref base_joints, but in the top platform plane
    std::array<Eigen::Vector3d, 6> platform_joints;

    /// Maximum allowed accumulated error of all struts
    double kins_max_allowed_error = 500.0;

    /// Maximum number of iterations to perform Newton's method
    int kins_max_iterations = 120;

    /// Strut tolerance for convergence
    double kins_convergence_criterion = 1e-9;

    /// When computing fk, we jit the seed position and orientation a little to try again
    int kins_fwd_max_retries = 10;

    bool lead_correction_enable = false;
    double lead_correction_screw_lead = 0.0;
  };

  Eigen::Quaterniond rpy_to_quaternion(Eigen::Vector3d rpy);

  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond quat);

  std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
  forward_kinematics(
      const HexapodConfig &config,
      const std::array<double, 6> &joints,
      const Eigen::Vector3d &current_position,
      const Eigen::Quaterniond &current_orientation);

  std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
  forward_kinematics(
      const HexapodConfig &config,
      const std::array<double, 6> &joints,
      const Eigen::Vector3d &current_position,
      const Eigen::Quaterniond &current_orientation,
      const std::array<Eigen::Vector3d, 6> &base_joint_directions,
      const std::array<Eigen::Vector3d, 6> &platform_joint_directions);

  std::array<double, 6> inverse_kinematics(
      const HexapodConfig &config,
      const Eigen::Vector3d &current_position,
      const Eigen::Quaterniond &current_orientation);
  std::array<double, 6> inverse_kinematics(
      const HexapodConfig &config,
      const Eigen::Vector3d &current_position,
      const Eigen::Quaterniond &current_orientation,
      const std::array<Eigen::Vector3d, 6> &base_joint_directions,
      const std::array<Eigen::Vector3d, 6> &platform_joint_directions);

  std::array<Eigen::Vector3d, 6> get_joint_vectors(
      const HexapodConfig &config,
      const Eigen::Vector3d &current_position,
      const Eigen::Quaterniond &current_orientation);

} // namespace hexkins

#endif // HEXAPOD_KINEMATICS__HEXAPOD_KINEMATICS_H_
