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

// plateau 包络：过渡区 [s-T, s] 0→1（C2），plateau [s, e] 保持 1，
// 过渡区 [e, e+T] 1→0（C2），外为 0。w/dw/d2w 解析。
void plateauWeight(double t, double s, double e, double T,
                   double& w, double& dw, double& d2w)
{
  w = 0.0;
  dw = 0.0;
  d2w = 0.0;
  if (t < s - T || t > e + T)
    return;
  if (t < s)
  {
    const double tau = (t - (s - T)) / T;  // [0,1]
    w = 0.5 * (1.0 - std::cos(M_PI * tau));
    dw = 0.5 * M_PI * std::sin(M_PI * tau) / T;
    d2w = 0.5 * M_PI * M_PI * std::cos(M_PI * tau) / (T * T);
  }
  else if (t <= e)
  {
    w = 1.0;
  }
  else
  {
    const double tau = (t - e) / T;  // [0,1]
    w = 0.5 * (1.0 + std::cos(M_PI * tau));
    dw = -0.5 * M_PI * std::sin(M_PI * tau) / T;
    d2w = -0.5 * M_PI * M_PI * std::cos(M_PI * tau) / (T * T);
  }
}
}  // namespace

YieldCandidateGenerator::YieldCandidateGenerator(const YieldCandidateConfig& config)
  : config_(config)
{
}

bool YieldCandidateGenerator::computeMinClearance(
    const TimedTrajectory& traj, const std::vector<NeighborTrajectory>& neighbors,
    double& min_clear, double& conflict_center,
    double& overlap_start, double& overlap_end) const
{
  min_clear = std::numeric_limits<double>::infinity();
  conflict_center = 0.0;
  overlap_start = std::numeric_limits<double>::infinity();
  overlap_end = -std::numeric_limits<double>::infinity();
  if (traj.pts.empty() || traj.pts.size() != traj.t.size())
    return false;
  const double gate = config_.clearance_factor * 2.0 * config_.swarm_clearance;
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
      if (d < gate)
      {
        overlap_start = std::min(overlap_start, glb_time);
        overlap_end = std::max(overlap_end, glb_time);
      }
    }
  }
  if (overlap_start > overlap_end)
    overlap_start = overlap_end = conflict_center;
  return std::isfinite(min_clear);
}

bool YieldCandidateGenerator::validate(const TimedTrajectory& traj,
                                       const std::vector<NeighborTrajectory>& neighbors,
                                       double& min_clear) const
{
  double center = 0.0, os = 0.0, oe = 0.0;
  if (!computeMinClearance(traj, neighbors, min_clear, center, os, oe))
    return false;
  const double gate = config_.clearance_factor * 2.0 * config_.swarm_clearance;
  if (min_clear < gate)
    return false;
  // 合法性：finite + 时间单调。动力学由时间轴生成保证（速度 ≤1.5 驱动缩放）。
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

void YieldCandidateGenerator::applyTimeScale(TimedTrajectory& traj, double factor)
{
  if (factor <= 0.0)
    return;
  const std::vector<double> old_t = traj.t;  // 用旧时间轴差分
  double acc_time = 0.0;
  for (size_t i = 0; i < traj.t.size(); ++i)
  {
    if (i == 0)
      acc_time = 0.0;
    else
      acc_time += (old_t[i] - old_t[i - 1]) / factor;
    traj.t[i] = acc_time;
    if (i > 0 && i < traj.vel.size())
      traj.vel[i] *= factor;
    if (i > 0 && i < traj.acc.size())
      traj.acc[i] *= factor * factor;
  }
  traj.duration = acc_time;
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
  // 变速档位 {0.5, 0.8, 1.2}（26082621 迭代）：0.5/0.8 减速等待错开、1.2 加速先过；
  // 穿越时刻相对 peer 移动轨迹错开。最终速度 v_max×factor ≤ max_velocity_mps。
  double v_max = 0.0;
  for (const auto& v : remaining.vel)
    v_max = std::max(v_max, v.norm());
  const double slow_min = std::max(0.05, config_.slow_min_scale);
  for (double scale : {0.5, 0.8, 1.2})
  {
    double factor = std::max(scale, slow_min);
    // 速度约束（含加速档）：加速后不得超 max_velocity_mps。
    if (v_max > 1e-9 && v_max * factor > config_.max_velocity_mps)
      factor = config_.max_velocity_mps / v_max;
    YieldCandidate cand;
    cand.type = YieldCandidate::Type::SLOW;
    cand.direction = YieldDirection::NONE;
    cand.trajectory = remaining;
    applyTimeScale(cand.trajectory, factor);
    // ΔT 取绝对值（减速等待与加速提前都计入代价，26082621 迭代）。
    cand.time_extension_s = std::fabs(cand.trajectory.duration - remaining_nominal);
    cand.cost = remaining_nominal > 1e-6
        ? cand.time_extension_s / remaining_nominal
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
    const std::vector<NeighborTrajectory>& neighbors,
    double overlap_start, double overlap_end, YieldDirection dir) const
{
  // 方向 n_avoid：重叠时段内"本机平均 - peer 平均"的水平方向（远离走廊）；
  // 退化为 0 时用本机切线法向。
  Eigen::Vector3d self_avg = Eigen::Vector3d::Zero();
  Eigen::Vector3d peer_avg = Eigen::Vector3d::Zero();
  int n = 0;
  for (size_t i = 0; i < remaining.t.size(); ++i)
  {
    if (remaining.t[i] < overlap_start || remaining.t[i] > overlap_end)
      continue;
    self_avg += remaining.pts[i];
    for (const auto& nb : neighbors)
    {
      if (nb.clearance <= 0.0)
        continue;
      const double t_abs = remaining.t[i] - nb.start_time;
      if (t_abs < nb.t.front() || t_abs > nb.t.back())
        continue;
      peer_avg += samplePeer(nb, t_abs);
      break;
    }
    ++n;
  }
  // 方向：self-peer 差在本机轨迹切线法向上的分量决定"远离"侧；法向分量≈0
  // （peer 在正前/正后）时用切线法向兜底（横向绕开）。
  Eigen::Vector3d d = (self_avg - peer_avg);
  d.z() = 0.0;
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  for (const auto& v : remaining.vel)
  {
    const Eigen::Vector3d h(v.x(), v.y(), 0.0);
    if (h.norm() > 1e-6)
    {
      tangent += h;
      break;
    }
  }
  if (tangent.norm() > 1e-6)
    tangent.normalize();
  else
    tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Vector3d normal_left(-tangent.y(), tangent.x(), 0.0);
  const double along = d.dot(normal_left);  // self 相对 peer 在法向的位置
  Eigen::Vector3d n_avoid = normal_left;
  if (std::fabs(along) > 1e-3)
    n_avoid = (along > 0.0 ? -1.0 : 1.0) * normal_left;  // 远离 peer 的法向侧
  // LEFT/RIGHT 各自方向（RIGHT 为法向反侧）。
  if (dir == YieldDirection::RIGHT)
    n_avoid = -n_avoid;

  const double span = std::max(0.0, overlap_end - overlap_start);
  const double T_trans = clamp(span / 2.0, 0.5, 1.5);
  for (double lateral : {1.3, 1.8, 2.2})
  {
    for (double scale_wait : {0.7, 0.5})
    {
      YieldCandidate cand;
      cand.type = YieldCandidate::Type::LEFT;
      cand.direction = dir;
      cand.lateral_dev_m = lateral;
      cand.trajectory = remaining;
      double v_max = 0.0;
      for (size_t i = 0; i < remaining.t.size(); ++i)
      {
        double w = 0.0, dw = 0.0, d2w = 0.0;
        plateauWeight(remaining.t[i], overlap_start, overlap_end, T_trans,
                      w, dw, d2w);
        cand.trajectory.pts[i] += n_avoid * (lateral * w);
        if (i < cand.trajectory.vel.size())
        {
          cand.trajectory.vel[i] += n_avoid * (lateral * dw);
          v_max = std::max(v_max, cand.trajectory.vel[i].norm());
        }
        if (i < cand.trajectory.acc.size())
          cand.trajectory.acc[i] += n_avoid * (lateral * d2w);
      }
      const double scale_vel = v_max > config_.max_velocity_mps
          ? config_.max_velocity_mps / v_max : 1.0;
      applyTimeScale(cand.trajectory, scale_wait * scale_vel);
      // ΔT 取绝对值（26082621 迭代）：变速（含加速）时间差计入代价。
      cand.time_extension_s = std::fabs(
          cand.trajectory.duration - remaining_nominal);
      cand.cost = (remaining_nominal > 1e-6
                       ? cand.time_extension_s / remaining_nominal : 0.0)
                  + lateral / 5.0;  // dev 权重降低（26082700：dev/2.0→dev/5.0，横向让路代价更小）
      double min_clear = 0.0;
      if (validate(cand.trajectory, neighbors, min_clear))
      {
        cand.min_clearance_m = min_clear;
        return cand;
      }
    }
  }
  YieldCandidate none;
  return none;
}

std::vector<YieldCandidate> YieldCandidateGenerator::generate(
    const TimedTrajectory& current, double t_elapsed,
    const std::vector<NeighborTrajectory>& neighbors, bool peer_hovering,
    bool slow_only) const
{
  std::vector<YieldCandidate> result;
  if (current.pts.empty())
    return result;
  const TimedTrajectory remaining = extractRemainder(current, t_elapsed);
  if (remaining.pts.size() < 2)
    return result;
  const double remaining_nominal = remaining.duration;
  double min_clear = 0.0, conflict_center = 0.0;
  double overlap_start = 0.0, overlap_end = 0.0;
  computeMinClearance(remaining, neighbors, min_clear, conflict_center,
                      overlap_start, overlap_end);

  // slow_only（26082621 迭代）：STAND_ON 侧只变速（SLOW），不横向偏移。
  if (!peer_hovering)
  {
    const YieldCandidate slow = makeSlow(remaining, remaining_nominal, neighbors);
    if (slow.cost < std::numeric_limits<double>::infinity())
      result.push_back(slow);
  }
  if (!slow_only)
  {
    const YieldCandidate left = makeLateral(remaining, remaining_nominal, neighbors,
                                            overlap_start, overlap_end,
                                            YieldDirection::LEFT);
    if (left.cost < std::numeric_limits<double>::infinity())
      result.push_back(left);
    const YieldCandidate right = makeLateral(remaining, remaining_nominal, neighbors,
                                             overlap_start, overlap_end,
                                             YieldDirection::RIGHT);
    if (right.cost < std::numeric_limits<double>::infinity())
      result.push_back(right);
  }

  // cost 升序（SLOW 无 dev 项 → 自然偏向 SLOW，不强制）。
  std::sort(result.begin(), result.end(),
            [](const YieldCandidate& a, const YieldCandidate& b)
            { return a.cost < b.cost; });
  return result;
}

}  // namespace ego_planner_driver

