#include "ego_planner_driver/yield_candidates.h"

#include <algorithm>
#include <cmath>

namespace ego_planner_driver
{

namespace
{
// 邻机轨迹线性插值采样（同刻采样：t_abs 相对 neighbor.start_time）。
Eigen::Vector3d samplePeer(const NeighborTrajectory& nb, double t_abs)
{
  if (nb.pts.empty())
    return Eigen::Vector3d::Zero();
  if (t_abs <= nb.t.front())
    return nb.pts.front();
  if (t_abs >= nb.t.back())
    return nb.pts.back();
  for (size_t i = 0; i + 1 < nb.t.size(); ++i)
  {
    if (t_abs < nb.t[i + 1])
    {
      const double a = (t_abs - nb.t[i]) / (nb.t[i + 1] - nb.t[i]);
      return nb.pts[i] * (1.0 - a) + nb.pts[i + 1] * a;
    }
  }
  return nb.pts.back();
}

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

YieldCandidateGenerator::YieldCandidateGenerator(const YieldCandidateConfig& config)
  : config_(config)
{
}

bool YieldCandidateGenerator::computeMinClearance(
    const TimedTrajectory& traj, const std::vector<NeighborTrajectory>& neighbors,
    double& min_clear, double& conflict_center) const
{
  min_clear = std::numeric_limits<double>::infinity();
  conflict_center = 0.0;
  if (traj.pts.empty() || traj.pts.size() != traj.t.size())
    return false;
  for (size_t i = 0; i < traj.t.size(); ++i)
  {
    const double glb_time = traj.t[i];
    for (const auto& nb : neighbors)
    {
      if (nb.clearance <= 0.0 || nb.pts.size() < 2 || nb.t.size() != nb.pts.size())
        continue;
      const double t_abs = glb_time - nb.start_time;
      if (t_abs < nb.t.front() || t_abs > nb.t.back())
        continue;
      const double d = (traj.pts[i] - samplePeer(nb, t_abs)).norm();
      if (d < min_clear)
      {
        min_clear = d;
        conflict_center = glb_time;
      }
    }
  }
  return std::isfinite(min_clear);
}

bool YieldCandidateGenerator::validate(const TimedTrajectory& traj,
                                       const std::vector<NeighborTrajectory>& neighbors,
                                       double& min_clear) const
{
  double center = 0.0;
  if (!computeMinClearance(traj, neighbors, min_clear, center))
    return false;
  const double gate = config_.clearance_factor * 2.0 * 0.5;  // 1.3 × swarm_clearance×2
  if (min_clear < gate)
    return false;
  // 合法性：finite + 时间单调。动力学由候选构造保证（SLOW 缩放 / LEFT/RIGHT 的
  // 横向机动速度预算 T=lateral·π/0.4），不重新限原轨迹绝对速度（replanOnce
  // feasibility 负责；候选继承原轨迹速度 profile）。
  for (size_t i = 0; i < traj.pts.size(); ++i)
  {
    if (!std::isfinite(traj.t[i]) || !traj.pts[i].allFinite() ||
        (i < traj.vel.size() && !traj.vel[i].allFinite()) ||
        (i < traj.acc.size() && !traj.acc[i].allFinite()))
      return false;
    if (i > 0 && traj.t[i] <= traj.t[i - 1])
      return false;
  }
  return true;
}

TimedTrajectory YieldCandidateGenerator::extractRemainder(const TimedTrajectory& current,
                                                          double t_elapsed)
{
  TimedTrajectory out;
  size_t start = 0;
  for (size_t i = 0; i < current.t.size(); ++i)
  {
    if (current.t[i] >= t_elapsed - 1e-6)
    {
      start = i;
      break;
    }
  }
  for (size_t i = start; i < current.t.size(); ++i)
  {
    out.t.push_back(current.t[i] - t_elapsed);
    out.pts.push_back(current.pts[i]);
    if (i < current.vel.size())
      out.vel.push_back(current.vel[i]);
    if (i < current.acc.size())
      out.acc.push_back(current.acc[i]);
    if (i < current.yaw.size())
      out.yaw.push_back(current.yaw[i]);
  }
  out.duration = out.t.empty() ? 0.0 : out.t.back();
  return out;
}

YieldCandidate YieldCandidateGenerator::makeSlow(
    const TimedTrajectory& remaining, double remaining_nominal,
    const std::vector<NeighborTrajectory>& neighbors) const
{
  // scale 档位从 slow_min_scale 到 1.0（离散尝试，首个可行）。起点速度保持原值
  // （P/V 连续），后续点速度 ×scale。
  for (double scale : {config_.slow_min_scale, 0.5, 0.7, 1.0})
  {
    YieldCandidate cand;
    cand.type = YieldCandidate::Type::SLOW;
    cand.direction = YieldDirection::NONE;
    cand.trajectory = remaining;
    cand.trajectory.t.clear();
    cand.trajectory.duration = 0.0;
    double acc_time = 0.0;
    for (size_t i = 0; i < remaining.t.size(); ++i)
    {
      if (i == 0)
      {
        acc_time = 0.0;
      }
      else
      {
        acc_time += (remaining.t[i] - remaining.t[i - 1]) / scale;
      }
      cand.trajectory.t.push_back(acc_time);
      if (i < remaining.vel.size())
      {
        Eigen::Vector3d v = remaining.vel[i];
        if (i > 0)
          v *= scale;  // 起点速度保持原值（连续）
        cand.trajectory.vel[i] = v;
      }
      if (i < remaining.acc.size())
        cand.trajectory.acc[i] *= (i > 0 ? scale * scale : 1.0);
    }
    cand.trajectory.duration = acc_time;
    cand.time_extension_s = std::max(0.0, cand.trajectory.duration - remaining_nominal);
    cand.cost = remaining_nominal > 1e-6 ? cand.time_extension_s / remaining_nominal
                                         : std::numeric_limits<double>::infinity();
    double min_clear = 0.0;
    if (validate(cand.trajectory, neighbors, min_clear))
    {
      cand.min_clearance_m = min_clear;
      return cand;
    }
  }
  YieldCandidate none;
  return none;
}

YieldCandidate YieldCandidateGenerator::makeLateral(
    const TimedTrajectory& remaining, double remaining_nominal,
    const std::vector<NeighborTrajectory>& neighbors, double conflict_center,
    YieldDirection dir) const
{
  for (double lateral : {0.5, 1.0, 1.5, config_.lateral_max_m})
  {
    // 窗口 T 自适应：横向机动速度 dw = lateral·π/T ≤ 0.4 m/s（预算，考虑本机轨迹
    // 速度峰值可接近 max_vel，叠加后仍 ≤1.5）；窗口覆盖冲突区并 clamp 到轨迹内。
    const double T = std::min(remaining.duration,
                              std::max(2.0, lateral * M_PI / 0.4));
    const double t0 = clamp(conflict_center - T / 2.0, 0.0,
                            std::max(0.0, remaining.duration - T));
    YieldCandidate cand;
    cand.type = YieldCandidate::Type::LEFT;
    cand.direction = dir;
    cand.lateral_dev_m = lateral;
    cand.trajectory = remaining;
    const bool is_left = (dir == YieldDirection::LEFT);
    for (size_t i = 0; i < remaining.t.size(); ++i)
    {
      const double t = remaining.t[i];
      double w = 0.0, dw = 0.0, d2w = 0.0;
      if (t > t0 && t < t0 + T)
      {
        const double tau = (t - t0) / T;
        w = lateral * 0.5 * (1.0 - std::cos(2.0 * M_PI * tau));
        dw = lateral * M_PI * std::sin(2.0 * M_PI * tau) / T;
        d2w = lateral * 2.0 * M_PI * M_PI * std::cos(2.0 * M_PI * tau) / (T * T);
      }
      Eigen::Vector3d n(Eigen::Vector3d::Zero());
      if (i < remaining.vel.size())
      {
        const Eigen::Vector3d tangent(remaining.vel[i].x(), remaining.vel[i].y(), 0.0);
        if (tangent.norm() > 1e-6)
        {
          const Eigen::Vector3d t_hat = tangent / tangent.norm();
          n = is_left ? Eigen::Vector3d(-t_hat.y(), t_hat.x(), 0.0)
                      : Eigen::Vector3d(t_hat.y(), -t_hat.x(), 0.0);
        }
      }
      cand.trajectory.pts[i] += n * w;
      if (i < cand.trajectory.vel.size())
        cand.trajectory.vel[i] += n * dw;
      if (i < cand.trajectory.acc.size())
        cand.trajectory.acc[i] += n * d2w;
    }
    cand.time_extension_s = 0.0;
    cand.cost = lateral / config_.lateral_max_m;  // dev/2.0（2.0 = 最大偏移）
    double min_clear = 0.0;
    if (validate(cand.trajectory, neighbors, min_clear))
    {
      cand.min_clearance_m = min_clear;
      return cand;
    }
  }
  YieldCandidate none;
  return none;
}

std::vector<YieldCandidate> YieldCandidateGenerator::generate(
    const TimedTrajectory& current, double t_elapsed,
    const std::vector<NeighborTrajectory>& neighbors, bool peer_hovering) const
{
  std::vector<YieldCandidate> result;
  if (current.pts.empty())
    return result;
  const TimedTrajectory remaining = extractRemainder(current, t_elapsed);
  if (remaining.pts.size() < 2)
    return result;
  const double remaining_nominal = remaining.duration;
  double conflict_center = 0.0, min_clear = 0.0;
  computeMinClearance(remaining, neighbors, min_clear, conflict_center);

  if (!peer_hovering)
  {
    const YieldCandidate slow = makeSlow(remaining, remaining_nominal, neighbors);
    if (slow.cost < std::numeric_limits<double>::infinity())
      result.push_back(slow);
  }
  const YieldCandidate left = makeLateral(remaining, remaining_nominal, neighbors,
                                          conflict_center, YieldDirection::LEFT);
  if (left.cost < std::numeric_limits<double>::infinity())
    result.push_back(left);
  const YieldCandidate right = makeLateral(remaining, remaining_nominal, neighbors,
                                           conflict_center, YieldDirection::RIGHT);
  if (right.cost < std::numeric_limits<double>::infinity())
    result.push_back(right);

  std::sort(result.begin(), result.end(),
            [](const YieldCandidate& a, const YieldCandidate& b)
            { return a.cost < b.cost; });
  return result;
}

}  // namespace ego_planner_driver
