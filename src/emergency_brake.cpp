#include "ego_planner_driver/emergency_brake.h"

#include <algorithm>
#include <cmath>

#include "ego_planner_driver/polynomial_traj.h"

namespace ego_planner_driver
{

EmergencyBrakeGenerator::EmergencyBrakeGenerator(const EmergencyBrakeConfig& config)
    : config_(config)
{
}

bool EmergencyBrakeGenerator::generate(const TrajectorySample& start,
                                       TimedTrajectory& output) const
{
  output = TimedTrajectory();
  if (!start.pos.allFinite() || !start.vel.allFinite() || !start.acc.allFinite() ||
      !std::isfinite(start.yaw))
    return false;
  const double speed = start.vel.norm();
  if (speed > config_.max_velocity_mps * 1.05 ||
      start.acc.norm() > config_.max_acceleration_mps2 * 1.05)
    return false;

  const double decel = std::max(1e-3, config_.nominal_deceleration_mps2);
  double duration = std::max(config_.minimum_duration_s,
      speed / decel + start.acc.norm() / std::max(1e-3, config_.max_jerk_mps3));
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  if (speed > 1e-6) direction = start.vel / speed;
  // Conservative stop displacement. Duration growth changes polynomial shape,
  // not endpoint, preserving minimum stopping-distance intent.
  const Eigen::Vector3d stop = start.pos + direction * speed * speed / (2.0 * decel);

  while (duration <= config_.maximum_duration_s + 1e-9)
  {
    OneSegmentPolynomial polynomial = oneSegmentTrajGen(
        start.pos, start.vel, start.acc, stop,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), duration);
    TimedTrajectory candidate;
    candidate.duration = duration;
    const double dt = std::max(0.005, config_.sample_period_s);
    const size_t samples = std::max<size_t>(2,
        static_cast<size_t>(std::ceil(duration / dt)) + 1);
    for (size_t i = 0; i < samples; ++i)
    {
      const double t = i + 1 == samples ? duration
          : duration * static_cast<double>(i) / static_cast<double>(samples - 1);
      candidate.t.push_back(t);
      candidate.pts.push_back(polynomial.evaluate(t));
      candidate.vel.push_back(polynomial.evaluateVel(t));
      candidate.acc.push_back(polynomial.evaluateAcc(t));
      candidate.yaw.push_back(start.yaw);
    }
    if (validate(candidate))
    {
      output = candidate;
      return true;
    }
    duration *= 1.25;
  }
  return false;
}

bool EmergencyBrakeGenerator::validate(const TimedTrajectory& trajectory) const
{
  const size_t n = trajectory.t.size();
  if (n < 2 || trajectory.pts.size() != n || trajectory.vel.size() != n ||
      trajectory.acc.size() != n || trajectory.yaw.size() != n ||
      std::abs(trajectory.t.front()) > 1e-9 ||
      std::abs(trajectory.t.back() - trajectory.duration) > 1e-6)
    return false;
  for (size_t i = 0; i < n; ++i)
  {
    if (!std::isfinite(trajectory.t[i]) || !trajectory.pts[i].allFinite() ||
        !trajectory.vel[i].allFinite() || !trajectory.acc[i].allFinite() ||
        !std::isfinite(trajectory.yaw[i]) ||
        trajectory.vel[i].norm() > config_.max_velocity_mps * 1.001 ||
        trajectory.acc[i].norm() > config_.max_acceleration_mps2 * 1.001 ||
        (i > 0 && trajectory.t[i] <= trajectory.t[i - 1]))
      return false;
    if (i > 0)
    {
      const double dt = trajectory.t[i] - trajectory.t[i - 1];
      const double jerk = (trajectory.acc[i] - trajectory.acc[i - 1]).norm() / dt;
      if (jerk > config_.max_jerk_mps3 * 1.01) return false;
    }
  }
  return trajectory.vel.back().norm() <= 1e-4 &&
         trajectory.acc.back().norm() <= 1e-4;
}

}  // namespace ego_planner_driver