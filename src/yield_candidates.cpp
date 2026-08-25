#include "ego_planner_driver/yield_candidates.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner_driver
{

YieldCandidateGenerator::YieldCandidateGenerator(const YieldCandidateConfig& config)
    : config_(config)
{
}

const char* YieldCandidateGenerator::maneuverName(YieldManeuver maneuver)
{
  switch (maneuver)
  {
    case YieldManeuver::LEFT: return "LEFT";
    case YieldManeuver::RIGHT: return "RIGHT";
    case YieldManeuver::SLOW: return "SLOW";
    case YieldManeuver::FAST: return "FAST";
  }
  return "UNKNOWN";
}

TimedTrajectory YieldCandidateGenerator::lateral(const TimedTrajectory& nominal,
                                                  double sign) const
{
  TimedTrajectory out = retime(nominal, config_.lateral_time_scale);
  if (nominal.t.size() < 2 || nominal.duration <= 0.0) return TimedTrajectory();
  Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
  for (const Eigen::Vector3d& velocity : nominal.vel)
  {
    tangent = velocity.head<2>();
    if (tangent.norm() > 1e-6) break;
  }
  if (tangent.norm() <= 1e-6)
    tangent = (nominal.pts.back() - nominal.pts.front()).head<2>();
  if (tangent.norm() <= 1e-6) return TimedTrajectory();
  tangent.normalize();
  const Eigen::Vector3d normal(-tangent.y(), tangent.x(), 0.0);
  const double amplitude = sign * config_.lateral_offset_m;
  for (size_t i = 0; i < out.t.size(); ++i)
  {
    const double s = std::max(0.0, std::min(1.0, out.t[i] / out.duration));
    // C2 bell: value, first derivative and second derivative are all zero at
    // both endpoints, preserving P/V/A continuity when candidate is committed.
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s, s6 = s5 * s;
    const double bump = 64.0 * (s3 - 3.0 * s4 + 3.0 * s5 - s6);
    const double bump_dot = 64.0 * (3.0 * s2 - 12.0 * s3 + 15.0 * s4 - 6.0 * s5) /
                            out.duration;
    const double bump_ddot = 64.0 * (6.0 * s - 36.0 * s2 + 60.0 * s3 - 30.0 * s4) /
                             (out.duration * out.duration);
    out.pts[i] += normal * amplitude * bump;
    out.vel[i] += normal * amplitude * bump_dot;
    out.acc[i] += normal * amplitude * bump_ddot;
  }
  return out;
}

TimedTrajectory YieldCandidateGenerator::retime(const TimedTrajectory& nominal,
                                                 double scale) const
{
  if (scale <= 0.0) return TimedTrajectory();
  TimedTrajectory out = nominal;
  out.duration *= scale;
  for (size_t i = 0; i < out.t.size(); ++i)
  {
    out.t[i] *= scale;
    out.vel[i] /= scale;
    out.acc[i] /= (scale * scale);
  }
  return out;
}

Eigen::Vector3d YieldCandidateGenerator::samplePeer(const NeighborTrajectory& peer,
                                                     double t)
{
  if (peer.pts.empty()) return Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  const double relative = t - peer.start_time;
  if (relative <= peer.t.front()) return peer.pts.front();
  if (relative >= peer.t.back()) return peer.pts.back();
  for (size_t i = 0; i + 1 < peer.t.size(); ++i)
  {
    if (relative <= peer.t[i + 1])
    {
      const double alpha = (relative - peer.t[i]) /
          (peer.t[i + 1] - peer.t[i]);
      return peer.pts[i] * (1.0 - alpha) + peer.pts[i + 1] * alpha;
    }
  }
  return peer.pts.back();
}

Eigen::Vector3d YieldCandidateGenerator::samplePeerVelocity(
    const NeighborTrajectory& peer, double t)
{
  if (peer.pts.size() < 2 || peer.t.size() < 2)
    return Eigen::Vector3d::Zero();
  const double dt = std::max(1e-6, (peer.t.back() - peer.t.front()) /
      static_cast<double>(peer.pts.size() - 1));
  return (samplePeer(peer, t + dt) - samplePeer(peer, t - dt)) / (2.0 * dt);
}

bool YieldCandidateGenerator::validate(
    const TimedTrajectory& trajectory,
    const std::vector<NeighborTrajectory>& peers,
    double& minimum_clearance, std::string& reason) const
{
  minimum_clearance = std::numeric_limits<double>::infinity();
  const size_t n = trajectory.t.size();
  if (n < 2 || trajectory.pts.size() != n || trajectory.vel.size() != n ||
      trajectory.acc.size() != n || trajectory.yaw.size() != n)
  {
    reason = "shape"; return false;
  }
  // 动态 emergency 距离：用候选起点（t[0]=now）时刻的 closing speed 计算，
  // 与 SafetyPredictor::evaluatePair 使用同一公式和同一时间基准，保证候选
  // 提交后下一 safety tick 不会立即被判 EMERGENCY（缺陷 3）。
  std::vector<double> emergency_distance(peers.size(),
      std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < n; ++i)
  {
    if (!trajectory.pts[i].allFinite() || !trajectory.vel[i].allFinite() ||
        !trajectory.acc[i].allFinite() || !std::isfinite(trajectory.t[i]))
    { reason = "nonfinite"; return false; }
    if (trajectory.vel[i].norm() > config_.max_velocity_mps * 1.001)
    { reason = "velocity"; return false; }
    if (trajectory.acc[i].norm() > config_.max_acceleration_mps2 * 1.05)
    { reason = "acceleration"; return false; }
    if (i > 0)
    {
      const double dt = trajectory.t[i] - trajectory.t[i - 1];
      if (dt <= 0.0) { reason = "time"; return false; }
      if ((trajectory.acc[i] - trajectory.acc[i - 1]).norm() / dt >
          config_.max_jerk_mps3 * 1.05)
      { reason = "jerk"; return false; }
    }
    for (size_t p = 0; p < peers.size(); ++p)
    {
      const double t = trajectory.t[i];
      const Eigen::Vector3d peer_pos = samplePeer(peers[p], t);
      const double distance = (trajectory.pts[i] - peer_pos).norm();
      minimum_clearance = std::min(minimum_clearance, distance);
      if (distance < config_.clearance_m)
      { reason = "clearance"; return false; }
      if (i == 0)
      {
        const Eigen::Vector3d peer_vel = samplePeerVelocity(peers[p], t);
        const Eigen::Vector3d r = peer_pos - trajectory.pts[i];
        const Eigen::Vector3d v_rel = peer_vel - trajectory.vel[i];
        const double r_norm = r.norm();
        const double closing = r_norm > 1e-9
            ? std::max(0.0, -r.dot(v_rel) / r_norm) : 0.0;
        const double reaction = std::max(0.0, config_.control_delay_s) +
            std::max(0.0, config_.local_compute_budget_s) +
            std::max(0.0, peers[p].data_age_s >= 0.0
                ? peers[p].data_age_s : config_.peer_data_age_cap_s);
        const double decel = std::max(1e-6, config_.emergency_decel_mps2);
        const double margin = closing * reaction +
            closing * closing / (2.0 * decel);
        emergency_distance[p] = config_.protected_distance_m + margin +
            std::max(0.0, config_.emergency_buffer_m);
      }
      if (distance < emergency_distance[p])
      { reason = "emergency_boundary"; return false; }
    }
  }
  reason.clear();
  return true;
}

std::vector<YieldCandidate> YieldCandidateGenerator::generate(
    const TimedTrajectory& nominal,
    const std::vector<NeighborTrajectory>& peers) const
{
  std::vector<YieldCandidate> out(4);
  out[0].maneuver = YieldManeuver::LEFT;
  out[0].trajectory = lateral(nominal, 1.0);
  out[0].lateral_offset_m = config_.lateral_offset_m;
  out[1].maneuver = YieldManeuver::RIGHT;
  out[1].trajectory = lateral(nominal, -1.0);
  out[1].lateral_offset_m = config_.lateral_offset_m;
  out[2].maneuver = YieldManeuver::SLOW;
  out[2].trajectory = retime(nominal, config_.slow_time_scale);
  out[3].maneuver = YieldManeuver::FAST;
  out[3].trajectory = retime(nominal, config_.fast_time_scale);
  for (YieldCandidate& candidate : out)
  {
    candidate.arrival_time_delta_s = candidate.trajectory.duration - nominal.duration;
    candidate.feasible = validate(candidate.trajectory, peers,
                                  candidate.minimum_clearance_m,
                                  candidate.rejection_reason);
  }
  return out;
}

bool YieldCandidateGenerator::chooseBest(
    const std::vector<YieldCandidate>& candidates,
    YieldCandidate& selected) const
{
  bool found = false;
  for (const YieldCandidate& candidate : candidates)
  {
    if (!candidate.feasible) continue;
    if (!found || candidate.minimum_clearance_m > selected.minimum_clearance_m + 1e-9 ||
        (std::abs(candidate.minimum_clearance_m - selected.minimum_clearance_m) <= 1e-9 &&
         candidate.arrival_time_delta_s < selected.arrival_time_delta_s - 1e-9) ||
        (std::abs(candidate.minimum_clearance_m - selected.minimum_clearance_m) <= 1e-9 &&
         std::abs(candidate.arrival_time_delta_s - selected.arrival_time_delta_s) <= 1e-9 &&
         candidate.lateral_offset_m < selected.lateral_offset_m - 1e-9))
    {
      selected = candidate;
      found = true;
    }
  }
  return found;
}

}  // namespace ego_planner_driver