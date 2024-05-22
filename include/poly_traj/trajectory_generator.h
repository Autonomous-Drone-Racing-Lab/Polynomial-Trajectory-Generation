#pragma once
#include <Eigen/Dense>
#include <vector>

namespace poly_traj{
bool generateTrajectory(const std::vector<Eigen::Vector3d> &waypoints, double v_max, double a_max, double sampling_intervall, const double startTimeOffset, const Eigen::Vector3d& initialVel, const Eigen::Vector3d& initialAcc, Eigen::MatrixXd &result);
//bool generateTrajectory(const std::vector<Eigen::MatrixXd>& waypoints, double v_max, double a_max, double sampling_intervall, const double startTimeOffset, const Eigen::Vector3d& initialVel, const Eigen::Vector3d& initialAcc, Eigen::MatrixXd& result);
} // namespace poly_traj