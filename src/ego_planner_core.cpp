#include "ego_planner_driver/ego_planner_core.h"
#include "ego_planner_driver/polynomial_traj.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner_driver
{

namespace
{
// Clearance ellipsoid of the reciprocal penalty: symmetric 1.0 m sphere
// (horizontal 1.0 m, vertical 1.0 m).  The vertical axis was reduced from
// 2.0 m to 1.0 m so vertical separation matches horizontal: layered cruise
// (15/12/8 m) no longer penalizes neighbors separated by more than 1 m in z,
// and the planner clearance now agrees with SafetyPredictor (1.0 m 3D).
constexpr double kSwarmA = 1.0;
constexpr double kSwarmB = 1.0;
constexpr double kInvA2 = 1.0 / (kSwarmA * kSwarmA);
constexpr double kInvB2 = 1.0 / (kSwarmB * kSwarmB);
constexpr double kSampleStepS = 0.1;

Eigen::Vector3d sampleNeighbor(const NeighborTrajectory& traj, double t_rel)
{
  if (traj.pts.empty())
    return Eigen::Vector3d::Zero();
  if (t_rel <= traj.t.front())
    return traj.pts.front();
  if (t_rel >= traj.t.back())
    return traj.pts.back();
  for (size_t i = 0; i + 1 < traj.t.size(); ++i)
  {
    if (t_rel < traj.t[i + 1])
    {
      const double a = (t_rel - traj.t[i]) / (traj.t[i + 1] - traj.t[i]);
      return traj.pts[i] * (1.0 - a) + traj.pts[i + 1] * a;
    }
  }
  return traj.pts.back();
}

Eigen::Vector3d sampleNeighborVelocity(const NeighborTrajectory& traj, double t_rel)
{
  if (traj.pts.size() < 2 || traj.t.size() != traj.pts.size())
    return Eigen::Vector3d::Zero();
  const double dt = std::max(1e-6, (traj.t.back() - traj.t.front()) /
      static_cast<double>(traj.pts.size() - 1));
  return (sampleNeighbor(traj, t_rel + dt) - sampleNeighbor(traj, t_rel - dt)) /
         (2.0 * dt);
}

// Uniform densification moved to the Python EgoSwarmDriver layer (plan B):
// the driver builds command-specific layered keypoints and densifies them to
// 2 m before publishing waypoints.  This core only parameterizes whatever
// waypoints it receives (node truncates them to planning_horizon).

// Trapezoidal time estimator for the min-snap segment duration T.
// v0 = start velocity projected onto the segment direction (already clamped).
// Cruise at v_cruise with constant acceleration `a`; the final segment
// decelerates to rest (end velocity projection = 0), cruise segments end at
// v_cruise so the rolling replan joins continuously.
double estimateTrapezoidTime(double dist, double v0, double v_cruise,
                             double a, bool final_segment)
{
  if (dist <= 1e-9)
    return 0.0;
  v0 = std::min(std::max(v0, 0.0), v_cruise);  // clamp onto [0, v_cruise]
  const double end_proj = final_segment ? 0.0 : v_cruise;
  double t_acc = 0.0, d_acc = 0.0;
  if (v0 < v_cruise)
  {
    t_acc = (v_cruise - v0) / a;
    d_acc = v0 * t_acc + 0.5 * a * t_acc * t_acc;
  }
  const double t_dec = (v_cruise - end_proj) / a;
  const double d_dec = 0.5 * (v_cruise + end_proj) * t_dec;
  if (d_acc + d_dec >= dist)
  {
    // Triangular profile (no cruise): accelerate v0 -> v_peak then decelerate
    // v_peak -> end_proj over the whole distance.
    //   v_peak^2 = v0^2 + 2a*d1 = end_proj^2 + 2a*d2,  d1 + d2 = dist
    double v_peak = std::sqrt(0.5 * (v0 * v0 + end_proj * end_proj) + a * dist);
    if (!std::isfinite(v_peak) || v_peak <= v0)
    {
      // Distance too short to keep accelerating from v0: fall back to a
      // linear average-speed estimate (T stays positive and finite).
      return dist / (0.5 * (v0 + end_proj) + 1e-6);
    }
    const double t_dec_tri = (v_peak - end_proj) / a;
    return std::max(0.0, (v_peak - v0) / a + t_dec_tri);
  }
  const double d_cruise = dist - d_acc - d_dec;
  const double t_cruise = d_cruise / std::max(v_cruise, 1e-6);
  return std::max(0.0, t_acc + t_cruise + t_dec);
}

// Samples the min-snap polynomial at `spacing` arc-length intervals.
// A dense polyline is built with a fine time step, then points are taken every
// `spacing` along it (linear interpolation).  `start`/`end` are always the
// first/last samples so the B-spline parameterization anchors the exact
// endpoints.
std::vector<Eigen::Vector3d> samplePolynomialByArcLength(
    const OneSegmentPolynomial& poly, double spacing,
    const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
  std::vector<Eigen::Vector3d> dense;
  const double dt = 0.02;
  for (double t = 0.0; t <= 1.0 + 1e-9; t += dt)
    dense.push_back(poly.evaluate(t));
  dense.back() = end;  // exact endpoint

  std::vector<double> cum(dense.size(), 0.0);
  for (size_t i = 1; i < dense.size(); ++i)
    cum[i] = cum[i - 1] + (dense[i] - dense[i - 1]).norm();
  const double total = cum.back();

  std::vector<Eigen::Vector3d> out;
  out.push_back(start);
  if (spacing <= 0.0 || total <= spacing)
  {
    out.push_back(end);
    return out;
  }
  size_t j = 1;
  double target = spacing;
  while (target < total - 1e-6)
  {
    while (j + 1 < cum.size() && cum[j] < target)
      ++j;
    const double span = cum[j] - cum[j - 1];
    const double a = (span > 1e-12) ? (target - cum[j - 1]) / span : 0.0;
    out.push_back(dense[j - 1] * (1.0 - a) + dense[j] * a);
    target += spacing;
  }
  if ((out.back() - end).norm() > 1e-6)
    out.push_back(end);
  return out;
}

} // namespace

EgoPlannerCore::EgoPlannerCore(const EgoPlannerConfig& config)
  : config_(config)
  , bspline_interval_(config.ts)
  , order_(config.order)
{
}

double EgoPlannerCore::computeSafeClearance(
    const EgoPlannerConfig& config, const NeighborTrajectory& neighbor,
    double peer_t_rel, const Eigen::Vector3d& self_pos,
    const Eigen::Vector3d& self_vel)
{
  if (neighbor.pts.size() < 2 || neighbor.t.size() != neighbor.pts.size())
    return config.swarm_clearance * 2.0;
  if (peer_t_rel < neighbor.t.front() || peer_t_rel > neighbor.t.back())
    return config.swarm_clearance * 2.0;
  const Eigen::Vector3d peer_pos = sampleNeighbor(neighbor, peer_t_rel);
  const Eigen::Vector3d peer_vel = sampleNeighborVelocity(neighbor, peer_t_rel);
  const Eigen::Vector3d r = peer_pos - self_pos;
  const Eigen::Vector3d v_rel = peer_vel - self_vel;
  const double r_norm = r.norm();
  const double closing = r_norm > 1e-9
      ? std::max(0.0, -r.dot(v_rel) / r_norm) : 0.0;
  const double reaction = std::max(0.0, config.safety_control_delay_s) +
      std::max(0.0, config.safety_compute_budget_s) +
      std::max(0.0, neighbor.data_age_s);
  const double decel = std::max(1e-6, config.safety_emergency_decel_mps2);
  const double margin = closing * reaction +
      closing * closing / (2.0 * decel);
  return config.safety_protected_distance_m + margin +
      std::max(0.0, config.safety_emergency_buffer_m);
}

void EgoPlannerCore::combineCost(const double* x, double* grad, double& cost,
                                 const int n)
{
  const int start_id = order_;
  for (int k = 0; k < n; ++k)
  {
    const int col = start_id + k / 3;
    const int row = k % 3;
    cps_.points(row, col) = x[k];
  }
  const Eigen::MatrixXd& q = cps_.points;

  double f_smooth, f_feas, f_swarm;
  Eigen::MatrixXd g_smooth = Eigen::MatrixXd::Zero(3, cps_.size);
  Eigen::MatrixXd g_feas = Eigen::MatrixXd::Zero(3, cps_.size);
  Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);

  calcSmoothnessCost(q, f_smooth, g_smooth);
  calcFeasibilityCost(q, f_feas, g_feas);
  calcSwarmCost(q, f_swarm, g_swarm);
  double f_terminal;
  Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);
  calcTerminalCost(q, f_terminal, g_terminal);

  cost = config_.lambda_smooth * f_smooth +
         config_.lambda_feasibility * f_feas +
         config_.lambda_swarm * f_swarm +
         config_.lambda_terminal * f_terminal;
  const Eigen::MatrixXd gradient = config_.lambda_smooth * g_smooth +
                                   config_.lambda_feasibility * g_feas +
                                   config_.lambda_swarm * g_swarm +
                                   config_.lambda_terminal * g_terminal;

  for (int k = 0; k < n; ++k)
  {
    const int col = start_id + k / 3;
    const int row = k % 3;
    grad[k] = gradient(row, col);
  }
}

void EgoPlannerCore::calcSmoothnessCost(const Eigen::MatrixXd& q, double& cost,
                                        Eigen::MatrixXd& gradient) const
{
  cost = 0.0;
  Eigen::Vector3d jerk, temp_j;
  for (int i = 0; i < q.cols() - 3; ++i)
  {
    jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    gradient.col(i + 0) += -temp_j;
    gradient.col(i + 1) += 3.0 * temp_j;
    gradient.col(i + 2) += -3.0 * temp_j;
    gradient.col(i + 3) += temp_j;
  }
}

void EgoPlannerCore::calcFeasibilityCost(const Eigen::MatrixXd& q,
                                         double& cost,
                                         Eigen::MatrixXd& gradient) const
{
  cost = 0.0;
  const double ts = bspline_interval_;
  const double ts_inv2 = 1.0 / (ts * ts);
  const double ts_inv3 = 1.0 / (ts * ts * ts);
  // 对齐原版 ego-planner-swarm calcFeasibilityCost：三次方惩罚 + demarcation
  // 平滑区。原版二次方惩罚在 max_vel 边界导数不连续，L-BFGS 在边界附近振荡
  // 导致速度抖动（实测 0-1.2 m/s 波动）；三次方惩罚在 max_vel 处一阶导为 0、
  // 二阶衔接，优化器平滑收敛，速度贴住 max_vel 不超。
  const double demarcation = 1.0;  // m/s, m/s^2
  const double ar = 3.0 * demarcation;
  const double br = -3.0 * demarcation * demarcation;
  const double cr = demarcation * demarcation * demarcation;
  const double al = ar, bl = -br, cl = cr;

  /* velocity feasibility */
  for (int i = 0; i < q.cols() - 1; ++i)
  {
    const Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
    for (int j = 0; j < 3; ++j)
    {
      if (vi(j) > config_.max_vel + demarcation)
      {
        const double diff = vi(j) - config_.max_vel;
        cost += (ar * diff * diff + br * diff + cr) * ts_inv3;
        const double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
        gradient(j, i + 0) += -grad;
        gradient(j, i + 1) += grad;
      }
      else if (vi(j) > config_.max_vel)
      {
        const double diff = vi(j) - config_.max_vel;
        cost += std::pow(diff, 3) * ts_inv3;
        const double grad = 3.0 * diff * diff / ts * ts_inv3;
        gradient(j, i + 0) += -grad;
        gradient(j, i + 1) += grad;
      }
      else if (vi(j) < -(config_.max_vel + demarcation))
      {
        const double diff = vi(j) + config_.max_vel;
        cost += (al * diff * diff + bl * diff + cl) * ts_inv3;
        const double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
        gradient(j, i + 0) += -grad;
        gradient(j, i + 1) += grad;
      }
      else if (vi(j) < -config_.max_vel)
      {
        const double diff = vi(j) + config_.max_vel;
        cost += -std::pow(diff, 3) * ts_inv3;
        const double grad = -3.0 * diff * diff / ts * ts_inv3;
        gradient(j, i + 0) += -grad;
        gradient(j, i + 1) += grad;
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.cols() - 2; ++i)
  {
    const Eigen::Vector3d ai =
        (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
    for (int j = 0; j < 3; ++j)
    {
      if (ai(j) > config_.max_acc + demarcation)
      {
        const double diff = ai(j) - config_.max_acc;
        cost += ar * diff * diff + br * diff + cr;
        const double grad = (2.0 * ar * diff + br) * ts_inv2;
        gradient(j, i + 0) += grad;
        gradient(j, i + 1) += -2 * grad;
        gradient(j, i + 2) += grad;
      }
      else if (ai(j) > config_.max_acc)
      {
        const double diff = ai(j) - config_.max_acc;
        cost += std::pow(diff, 3);
        const double grad = 3.0 * diff * diff * ts_inv2;
        gradient(j, i + 0) += grad;
        gradient(j, i + 1) += -2 * grad;
        gradient(j, i + 2) += grad;
      }
      else if (ai(j) < -(config_.max_acc + demarcation))
      {
        const double diff = ai(j) + config_.max_acc;
        cost += al * diff * diff + bl * diff + cl;
        const double grad = (2.0 * al * diff + bl) * ts_inv2;
        gradient(j, i + 0) += grad;
        gradient(j, i + 1) += -2 * grad;
        gradient(j, i + 2) += grad;
      }
      else if (ai(j) < -config_.max_acc)
      {
        const double diff = ai(j) + config_.max_acc;
        cost += -std::pow(diff, 3);
        const double grad = -3.0 * diff * diff * ts_inv2;
        gradient(j, i + 0) += grad;
        gradient(j, i + 1) += -2 * grad;
        gradient(j, i + 2) += grad;
      }
    }
  }
}

void EgoPlannerCore::calcSwarmCost(const Eigen::MatrixXd& q, double& cost,
                                   Eigen::MatrixXd& gradient)
{
  cost = 0.0;
  const int end_idx = q.cols() - order_ -
                      static_cast<int>(static_cast<double>(q.cols() - 2 * order_) / 3.0);
  const double clearance = config_.swarm_clearance * 2.0;
  for (int i = order_; i < end_idx; ++i)
  {
    const double glb_time = (static_cast<double>(order_ - 1) / 2.0 +
                             static_cast<double>(i - order_ + 1)) *
                            bspline_interval_;
    for (const auto& neighbor : neighbors_)
    {
      if (neighbor.clearance <= 0.0 || neighbor.pts.size() < 2 ||
          neighbor.t.size() != neighbor.pts.size())
      {
        continue;
      }
      const double t_abs = glb_time - neighbor.start_time;
      if (t_abs < neighbor.t.front() ||
          t_abs > neighbor.t.back() - 0.1)
      {
        continue;
      }
      const Eigen::Vector3d swarm_prid = sampleNeighbor(neighbor, t_abs);
      const Eigen::Vector3d dist_vec = q.col(i) - swarm_prid;
      const double ellip_dist = std::sqrt(
          dist_vec(2) * dist_vec(2) * kInvA2 +
          (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * kInvB2);
      // 预测式避碰：使用节点按 SafetyPredictor 公式计算的动态安全距离，
      // 使规划在 peer 进入预测冲突区域（同刻交点局部）时提前绕开，而非只
      // 在固定 1m clearance 内才推开。软惩罚目标对动态距离带超调余量
      // （swarm_soft_margin），因为 L-BFGS 收敛总比请求 gate 短约 0.1m；
      // 固定 clearance 场景不加 margin，保持原行为。
      const double gate = neighbor.safe_clearance_m > 0.0
          ? neighbor.safe_clearance_m + config_.swarm_soft_margin
          : clearance;
      const double dist_err = gate - ellip_dist;
      if (dist_err >= 0.0)
      {
        cost += std::pow(dist_err, 2);
        Eigen::Vector3d coeff;
        coeff(0) = -2.0 * (gate / ellip_dist - 1.0) * kInvB2;
        coeff(1) = coeff(0);
        coeff(2) = -2.0 * (gate / ellip_dist - 1.0) * kInvA2;
        gradient.col(i) += (coeff.array() * dist_vec.array()).matrix();
      }
    }
  }
}

void EgoPlannerCore::calcTerminalCost(const Eigen::MatrixXd& q, double& cost,
                                       Eigen::MatrixXd& gradient)
{
  cost = 0.0;
  const int n = q.cols();
  const Eigen::Vector3d q_3 = q.col(n - 3);
  const Eigen::Vector3d q_2 = q.col(n - 2);
  const Eigen::Vector3d q_1 = q.col(n - 1);
  const Eigen::Vector3d dq = (q_3 + 4.0 * q_2 + q_1) / 6.0 - local_target_pt_;
  cost += dq.squaredNorm();
  gradient.col(n - 3) += 2.0 * dq * (1.0 / 6.0);
  gradient.col(n - 2) += 2.0 * dq * (4.0 / 6.0);
  gradient.col(n - 1) += 2.0 * dq * (1.0 / 6.0);
}

double EgoPlannerCore::costFunction(void* data, const double* x, double* grad,
                                    const int n)
{
  EgoPlannerCore* core = static_cast<EgoPlannerCore*>(data);
  double cost = 0.0;
  core->combineCost(x, grad, cost, n);
  return cost;
}

std::vector<NeighborTrajectory> EgoPlannerCore::filterByRange(
    const std::vector<NeighborTrajectory>& neighbors,
    const Eigen::Vector3d& local_position, double planning_range)
{
  std::vector<NeighborTrajectory> kept;
  for (const auto& neighbor : neighbors)
  {
    if (neighbor.pts.empty())
      continue;
    const double distance = (neighbor.pts.front() - local_position).norm();
    if (distance <= planning_range)
      kept.push_back(neighbor);
  }
  return kept;
}

Eigen::Vector3d EgoPlannerCore::waypointTangent(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& pos)
{
  if (waypoints.empty())
    return Eigen::Vector3d::Zero();
  size_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    const double d = (waypoints[i] - pos).squaredNorm();
    if (d < best_d)
    {
      best_d = d;
      best = i;
    }
  }
  const size_t prev = best > 0 ? best - 1 : best;
  const size_t next = best + 1 < waypoints.size() ? best + 1 : best;
  Eigen::Vector3d tangent = waypoints[next] - waypoints[prev];
  if (tangent.norm() < 1e-9)
    tangent = waypoints[best] - pos;
  if (tangent.norm() < 1e-9)
    return Eigen::Vector3d::Zero();
  return tangent.normalized();
}

bool EgoPlannerCore::reboundFromNeighbors(
    const std::vector<NeighborTrajectory>& neighbors,
    const std::vector<Eigen::Vector3d>& waypoints, double clearance)
{
  if (neighbors.empty())
    return false;
  const double ts = bspline_interval_;
  const int order = order_;
  const int end_idx = static_cast<int>(cps_.size) - order_ -
                      static_cast<int>(static_cast<double>(cps_.size - 2 * order_) / 3.0);
  bool rebounded = false;
  for (int i = order_; i < end_idx; ++i)
  {
    const double glb_time = (static_cast<double>(order_ - 1) / 2.0 +
                             static_cast<double>(i - order_ + 1)) * ts;
    const Eigen::Vector3d ctrl = cps_.points.col(i);
    const Eigen::Vector3d tangent = waypointTangent(waypoints, ctrl);
    Eigen::Vector3d moved = ctrl;
    for (const auto& neighbor : neighbors)
    {
      if (neighbor.clearance <= 0.0 || neighbor.pts.size() < 2 ||
          neighbor.t.size() != neighbor.pts.size())
      {
        continue;
      }
      const double t_abs = glb_time - neighbor.start_time;
      if (t_abs < neighbor.t.front() || t_abs > neighbor.t.back())
        continue;
      const Eigen::Vector3d swarm_pt = sampleNeighbor(neighbor, t_abs);
      const Eigen::Vector3d dist_vec = moved - swarm_pt;
      const double ellip = std::sqrt(
          dist_vec(2) * dist_vec(2) * kInvA2 +
          (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * kInvB2);
      if (ellip >= clearance)
        continue;
      // Push direction: ellipsoid-normalized direction with the waypoint
      // tangent component removed (deviate normal to the route, not along it).
      Eigen::Vector3d dir(dist_vec(0) * kInvB2, dist_vec(1) * kInvB2,
                          dist_vec(2) * kInvA2);
      if (tangent.norm() > 1e-6)
        dir = dir - tangent * dir.dot(tangent);
      if (dir.norm() < 1e-6)
      {
        // Neighbor directly ahead along the route (normal component ~0):
        // push sideways using the waypoint horizontal normal (tangent x z),
        // otherwise the spline stays on the route line and cannot deviate.
        dir = Eigen::Vector3d(-tangent.y(), tangent.x(), 0.0);
        if (dir.norm() < 1e-6)
          dir = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
      dir.normalize();
      // 推离到 clearance 外并留裕量（B-spline 曲线在控制点凸包内，
      // 曲线到邻居距离会小于控制点推离量，裕量保证曲线也满足
      // clearance 硬校验）。
      moved = swarm_pt + dir * (clearance * 3.0);
      rebounded = true;
    }
    if (rebounded)
      cps_.points.col(i) = moved;
  }
  return rebounded;
}

bool EgoPlannerCore::optimizeControls(double& final_cost)
{
  const int start_id = order_;
  const int n = variable_num_;
  std::vector<double> q(n);
  for (int k = 0; k < n; ++k)
  {
    const int col = start_id + k / 3;
    const int row = k % 3;
    q[k] = cps_.points(row, col);
  }
  lbfgs::lbfgs_parameter_t params;
  lbfgs::lbfgs_load_default_parameters(&params);
  params.mem_size = config_.mem_size;
  params.max_iterations = config_.max_iterations;
  params.g_epsilon = config_.g_epsilon;
  const int result =
      lbfgs::lbfgs_optimize(n, q.data(), &final_cost,
                            EgoPlannerCore::costFunction, nullptr, nullptr,
                            this, &params);
  if (result != lbfgs::LBFGS_CONVERGENCE &&
      result != lbfgs::LBFGSERR_MAXIMUMITERATION &&
      result != lbfgs::LBFGS_ALREADY_MINIMIZED &&
      result != lbfgs::LBFGS_STOP)
  {
    return false;
  }
  for (int k = 0; k < n; ++k)
  {
    const int col = start_id + k / 3;
    const int row = k % 3;
    cps_.points(row, col) = q[k];
  }
  return true;
}

bool EgoPlannerCore::planOnce(const GoalPoint& start,
                              const std::vector<WayPoint>& sparse_waypoints,
                              const std::vector<NeighborTrajectory>& neighbors,
                              TimedTrajectory& out, double& final_cost,
                              const Eigen::Vector3d& start_vel,
                              const Eigen::Vector3d& start_acc,
                              bool final_segment)
{
  if (sparse_waypoints.empty())
  {
    out.t.assign(1, 0.0);
    out.pts.assign(1, start.position);
    out.yaw.assign(1, start.yaw);
    final_cost = 0.0;
    return true;
  }
  // Window end point: the last waypoint of this plan.  The node already
  // truncated sparse_waypoints to planning_horizon; when final_segment the
  // waypoints are the full remaining route and end_vel must be zero (static
  // arrival).  Cruise segments keep end_vel = tangent * min(v_cruise, max_vel)
  // so the rolling replan's next segment joins velocity-continuously.
  const Eigen::Vector3d end_pt = sparse_waypoints.back().position;
  Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
  if (!final_segment)
  {
    Eigen::Vector3d tangent;
    if (sparse_waypoints.size() >= 2)
      tangent = end_pt - sparse_waypoints[sparse_waypoints.size() - 2].position;
    else
      tangent = end_pt - start.position;
    const double len = tangent.norm();
    if (len > 1e-9)
      end_vel = tangent / len * std::min(config_.cruise_velocity, config_.max_vel);
  }
  const Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();

  // Trapezoidal segment duration T (v0 projection, accel-cruise-decel).
  const Eigen::Vector3d dir = end_pt - start.position;
  const double dist = dir.norm();
  double v0_proj = 0.0;
  if (dist > 1e-9)
    v0_proj = start_vel.dot(dir / dist);
  const double v_cruise = std::max(0.0,
      std::min(config_.cruise_velocity, config_.max_vel));
  const double a = std::max(0.1, config_.acceleration_limit);
  const double segment_time = estimateTrapezoidTime(
      dist, v0_proj, v_cruise, a, final_segment);

  // min-snap single segment (one_segment_traj_gen): a quintic closed-form
  // polynomial that interpolates start P/V/A -> end P/V/A.  Sample it at
  // 0.4 m arc-length intervals to build the B-spline parameterization input
  // (the 2 m driver waypoints never enter the B-spline directly).
  const OneSegmentPolynomial poly = oneSegmentTrajGen(
      start.position, start_vel, start_acc, end_pt, end_vel, end_acc,
      std::max(segment_time, 1e-3));
  std::vector<Eigen::Vector3d> point_set = samplePolynomialByArcLength(
      poly, 0.4, start.position, end_pt);
  // Short-window fallback only: the parameterization needs >= 5 points so the
  // optimizer keeps at least one free control point; the min-snap 0.4 m
  // sampling already provides this for normal windows.
  while (point_set.size() < 5)
  {
    std::vector<Eigen::Vector3d> subdivided;
    subdivided.reserve(point_set.size() * 2);
    for (size_t i = 0; i + 1 < point_set.size(); ++i)
    {
      subdivided.push_back(point_set[i]);
      subdivided.push_back((point_set[i] + point_set[i + 1]) * 0.5);
    }
    subdivided.push_back(point_set.back());
    point_set.swap(subdivided);
  }

  std::vector<Eigen::Vector3d> start_end_derivative(4, Eigen::Vector3d::Zero());
  // upstream order: [0]=vel_start, [1]=vel_end, [2]=acc_start, [3]=acc_end
  start_end_derivative[0] = start_vel;  // start velocity (rolling continuity)
  start_end_derivative[1] = end_vel;    // end velocity (tangent cruise / stop)
  start_end_derivative[2] = start_acc;  // start acceleration (rolling continuity)
  start_end_derivative[3] = end_acc;    // end acceleration (zero)
  Eigen::MatrixXd ctrl_pts;
  ego_planner::UniformBspline::parameterizeToBspline(bspline_interval_, point_set,
                                        start_end_derivative, ctrl_pts);
  if (ctrl_pts.cols() < order_ + 1)
    return false;

  local_target_pt_ = point_set.back();
  cps_.clearance = config_.swarm_clearance;
  cps_.size = static_cast<int>(ctrl_pts.cols());
  cps_.points = ctrl_pts;
  neighbors_ = filterByRange(neighbors, start.position, config_.planning_range);

  // Pin the last order_ control points as a hard terminal constraint so jerk
  // smoothing cannot pull the free end away from the goal (original kernel
  // uses end_id = size - order_, the "Fixed end" mode).
  const int start_id = order_;
  const int end_id = cps_.size;
  variable_num_ = 3 * (end_id - start_id);
  // Exact-anchor the leading control points so the spline start pose is
  // exactly start.position.  The B-spline uses uniform (non-clamped) knots,
  // so at t=0 the de Boor average B(0)=(P0+4P1+P2)/6 holds and the velocity
  // row of parameterizeToBspline reads (P2-P0)/(2*ts) = start_vel.  The
  // underlying parameterizeToBspline is an over-determined least-squares
  // solve (K+2 controls vs K+4 constraints) and dilutes these boundary rows
  // (measured start velocity came out ~8 m/s regardless of the requested
  // derivative); re-solving P0..P2 here pins them exactly:
  //   P0 = pos - vel*ts + acc*ts^2/3
  //   P1 = pos - acc*ts^2/6
  //   P2 = pos + vel*ts + acc*ts^2/3
  // cols 0..2 are never optimized (start_id=3) nor rebound (rebound starts
  // at i=order_).
  if (cps_.points.cols() >= 3)
  {
    const double ts = bspline_interval_;
    const double t2 = ts * ts;
    const Eigen::Vector3d pos = start.position;
    cps_.points.col(0) = pos - start_vel * ts + start_acc * (t2 / 3.0);
    cps_.points.col(1) = pos - start_acc * (t2 / 6.0);
    cps_.points.col(2) = pos + start_vel * ts + start_acc * (t2 / 3.0);
  }
  if (!optimizeControls(final_cost))
    return false;
  // Symmetric exact-anchor of the trailing control points AFTER the optimizer
  // (the last order_=3 columns participate in L-BFGS; re-solving them here
  // restores the exact end_pt / end_vel / end_acc boundary constraints that
  // the smoothness cost would otherwise relax):
  //   B(1)     = (P[n-3] + 4P[n-2] + P[n-1])/6 = end_pt
  //   v(1)     = (P[n-1] - P[n-3])/(2*ts)      = end_vel
  //   a(1)     = (P[n-3] - 2P[n-2] + P[n-1])/ts^2 = end_acc
  if (cps_.points.cols() >= 3)
  {
    const double ts = bspline_interval_;
    const double t2 = ts * ts;
    const int n = static_cast<int>(cps_.points.cols());
    cps_.points.col(n - 3) = end_pt - end_vel * ts + end_acc * (t2 / 3.0);
    cps_.points.col(n - 2) = end_pt - end_acc * (t2 / 6.0);
    cps_.points.col(n - 1) = end_pt + end_vel * ts + end_acc * (t2 / 3.0);
  }

  // Swarm rebound hard push is DISABLED by default (min-snap refactor):
  // inter-UAV safety is owned by the execution-layer runtime distance gate
  // (EgoSwarmDriver), not the optimizer.  Re-enable via enable_rebound for
  // A/B comparison or the backup configuration.
  if (config_.enable_rebound)
    reboundFromNeighbors(neighbors_, point_set, config_.swarm_clearance * 2.0);

  ego_planner::UniformBspline traj(cps_.points, order_, bspline_interval_);
  // Physical-limit check (diagnostics only when time reparameterization is
  // disabled).  Time reparameterization is DISABLED by default: the min-snap
  // 0.4 m arc-length point set with ts = 0.4 s already yields ~1 m/s control
  // spacing, and lengthening the time axis destabilizes the speed profile
  // under 1 s rolling replan (see implementation_plan_26082415.md).
  traj.setPhysicalLimits(config_.max_vel, config_.max_acc,
                         config_.feasibility_tolerance);
  double fea_ratio = 1.0;
  const bool feasible = traj.checkFeasibility(fea_ratio);
  // Rebound changes geometry after optimization and can create large control
  // point derivatives. Never release an actively avoided trajectory that
  // violates the configured physical limits. Reparameterize rebound output
  // even when generic rolling-segment reparameterization remains disabled;
  // the final clearance check below evaluates the resulting time axis again.
  if (!feasible && (config_.enable_time_reparam || config_.enable_rebound))
    traj.lengthenTime(fea_ratio);
  double tm, tmp;
  traj.getTimeSpan(tm, tmp);
  const double duration = std::max(0.0, tmp - tm);
  out.duration = duration;
  ego_planner::UniformBspline dtraj = traj.getDerivative();
  ego_planner::UniformBspline ddtraj = dtraj.getDerivative();
  double du0 = 0.0, dup = 0.0;
  // The derivative B-splines drop one knot per order, so their time spans are
  // shorter than the position trajectory; clamp derivative samples to them.
  const double t_der_max =
      (dtraj.getTimeSpan(du0, dup)) ? std::max(0.0, dup - du0 - 1e-3) : duration;
  double duu0 = 0.0, duup = 0.0;
  const double t_acc_max =
      (ddtraj.getTimeSpan(duu0, duup)) ? std::max(0.0, duup - duu0 - 1e-3) : t_der_max;
  out.t.clear();
  out.pts.clear();
  out.vel.clear();
  out.acc.clear();
  out.yaw.clear();

  const double step = std::max(kSampleStepS, bspline_interval_ * 0.25);
  const int samples = static_cast<int>(std::floor(duration / step)) + 1;
  out.t.reserve(samples);
  out.pts.reserve(samples);
  out.vel.reserve(samples);
  out.acc.reserve(samples);
  out.yaw.reserve(samples);

  for (int i = 0; i < samples; ++i)
  {
    const double t_rel = std::min(duration, static_cast<double>(i) * step);
    out.t.push_back(t_rel);
    out.pts.push_back(traj.evaluateDeBoorT(t_rel));
    out.vel.push_back(Eigen::Vector3d(dtraj.evaluateDeBoorT(std::min(t_rel, t_der_max))));
    out.acc.push_back(Eigen::Vector3d(ddtraj.evaluateDeBoorT(std::min(t_rel, t_acc_max))));
    out.yaw.push_back(start.yaw);
  }
  // The fixed sampling step rarely divides the exact B-spline duration. The
  // intent contract requires t.back() == duration so neighbors can predict the
  // entire execution interval rather than losing the fractional final sample.
  if (!out.t.empty() && duration - out.t.back() > 1e-9)
  {
    out.t.push_back(duration);
    out.pts.push_back(traj.evaluateDeBoorT(duration));
    out.vel.push_back(Eigen::Vector3d(
        dtraj.evaluateDeBoorT(std::min(duration, t_der_max))));
    out.acc.push_back(Eigen::Vector3d(
        ddtraj.evaluateDeBoorT(std::min(duration, t_acc_max))));
    out.yaw.push_back(start.yaw);
  }
  if (out.t.empty())
  {
    out.t.push_back(0.0);
    out.pts.push_back(start.position);
    out.vel.push_back(Eigen::Vector3d::Zero());
    out.acc.push_back(Eigen::Vector3d::Zero());
    out.yaw.push_back(start.yaw);
  }
  // Anchor the sampled endpoints to the exact start position and the final
  // key point so the timed trajectory strictly respects the sparse way
  // point semantics regardless of the de Boor knot-interval definition.
  if (!out.pts.empty())
  {
    out.pts.front() = start.position;
    out.pts.back() = point_set.back();
  }
  if (!checkClearance(out, neighbors_, config_.swarm_clearance * 2.0))
    return false;
  return true;
}

bool EgoPlannerCore::checkClearance(const TimedTrajectory& trajectory,
                                    const std::vector<NeighborTrajectory>& neighbors,
                                    double clearance) const
{
  if (trajectory.pts.empty())
    return true;
  for (size_t i = 0; i < trajectory.t.size(); ++i)
  {
    const double glb_time = trajectory.t[i];
    for (const auto& neighbor : neighbors)
    {
      if (neighbor.clearance <= 0.0 || neighbor.pts.size() < 2)
        continue;
      const double t_abs = glb_time - neighbor.start_time;
      if (t_abs < neighbor.t.front() || t_abs > neighbor.t.back())
        continue;
      const Eigen::Vector3d swarm_prid = sampleNeighbor(neighbor, t_abs);
      const Eigen::Vector3d dist_vec = trajectory.pts[i] - swarm_prid;
      const double ellip_dist = std::sqrt(
          dist_vec(2) * dist_vec(2) * kInvA2 +
          (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * kInvB2);
      // 硬校验保持固定 swarm clearance（物理安全门，1 m），不随监督动态
      // 距离收紧：SITL 实测动态距离（含反应+制动余量，正交交叉时 ≈4 m）
      // 作为硬门会让规划在交叉走廊内拒绝一切穿越轨迹 → EGO_PLAN_FAILED
      // （比监督制动兜底更差）。动态距离只用于 calcSwarmCost 软惩罚激励
      // 绕开，运行时由 SafetyPredictor/监督层兜底。
      if (ellip_dist < clearance)
        return false;
    }
  }
  return true;
}

} // namespace ego_planner_driver
