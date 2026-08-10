#include "ego_planner_driver/ego_planner_core.h"

#include <algorithm>
#include <cmath>

namespace ego_planner_driver
{

namespace
{
// Ellipsoid shape of the reciprocal penalty from the original kernel:
// horizontal axes 1.0 m, vertical axis 2.0 m (paper Eq. 5 style).
constexpr double kSwarmA = 2.0;
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
} // namespace

EgoPlannerCore::EgoPlannerCore(const EgoPlannerConfig& config)
  : config_(config)
  , bspline_interval_(config.ts)
  , order_(config.order)
{
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

  for (int i = 0; i < q.cols() - 1; ++i)
  {
    const Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
    for (int j = 0; j < 3; ++j)
    {
      if (vi(j) > config_.max_vel)
      {
        cost += std::pow(vi(j) - config_.max_vel, 2) * ts_inv2;
        gradient(j, i + 0) += -2 * (vi(j) - config_.max_vel) / ts * ts_inv2;
        gradient(j, i + 1) += 2 * (vi(j) - config_.max_vel) / ts * ts_inv2;
      }
      else if (vi(j) < -config_.max_vel)
      {
        cost += std::pow(vi(j) + config_.max_vel, 2) * ts_inv2;
        gradient(j, i + 0) += -2 * (vi(j) + config_.max_vel) / ts * ts_inv2;
        gradient(j, i + 1) += 2 * (vi(j) + config_.max_vel) / ts * ts_inv2;
      }
    }
  }

  for (int i = 0; i < q.cols() - 2; ++i)
  {
    const Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;
    for (int j = 0; j < 3; ++j)
    {
      if (ai(j) > config_.max_acc)
      {
        cost += std::pow(ai(j) - config_.max_acc, 2);
        gradient(j, i + 0) += 2 * (ai(j) - config_.max_acc) * ts_inv2;
        gradient(j, i + 1) += -4 * (ai(j) - config_.max_acc) * ts_inv2;
        gradient(j, i + 2) += 2 * (ai(j) - config_.max_acc) * ts_inv2;
      }
      else if (ai(j) < -config_.max_acc)
      {
        cost += std::pow(ai(j) + config_.max_acc, 2);
        gradient(j, i + 0) += 2 * (ai(j) + config_.max_acc) * ts_inv2;
        gradient(j, i + 1) += -4 * (ai(j) + config_.max_acc) * ts_inv2;
        gradient(j, i + 2) += 2 * (ai(j) + config_.max_acc) * ts_inv2;
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
      const double dist_err = clearance - ellip_dist;
      if (dist_err >= 0.0)
      {
        cost += std::pow(dist_err, 2);
        Eigen::Vector3d coeff;
        coeff(0) = -2.0 * (clearance / ellip_dist - 1.0) * kInvB2;
        coeff(1) = coeff(0);
        coeff(2) = -2.0 * (clearance / ellip_dist - 1.0) * kInvA2;
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

bool EgoPlannerCore::planOnce(const GoalPoint& start,
                              const std::vector<WayPoint>& sparse_waypoints,
                              const std::vector<NeighborTrajectory>& neighbors,
                              TimedTrajectory& out, double& final_cost)
{
  if (sparse_waypoints.empty())
  {
    out.t.assign(1, 0.0);
    out.pts.assign(1, start.position);
    out.yaw.assign(1, start.yaw);
    final_cost = 0.0;
    return true;
  }
  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(sparse_waypoints.size() + 1);
  point_set.push_back(start.position);
  for (const auto& waypoint : sparse_waypoints)
    point_set.push_back(waypoint.position);

  // The parameterization yields (K+2) control points and the optimizer pins
  // the first and last order_ (=3) columns, so K must be at least 5 to keep
  // at least one free control point (variable_num_ = 3*(K-4)).
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
  std::vector<double> q(variable_num_);
  for (int k = 0; k < variable_num_; ++k)
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
      lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost,
                            EgoPlannerCore::costFunction, nullptr, nullptr,
                            this, &params);
  if (result != lbfgs::LBFGS_CONVERGENCE &&
      result != lbfgs::LBFGSERR_MAXIMUMITERATION &&
      result != lbfgs::LBFGS_ALREADY_MINIMIZED &&
      result != lbfgs::LBFGS_STOP)
  {
    return false;
  }

  for (int k = 0; k < variable_num_; ++k)
  {
    const int col = start_id + k / 3;
    const int row = k % 3;
    cps_.points(row, col) = q[k];
  }

  ego_planner::UniformBspline traj(cps_.points, order_, bspline_interval_);
  double tm, tmp;
  traj.getTimeSpan(tm, tmp);
  const double duration = std::max(0.0, tmp - tm);
  out.duration = duration;
  out.t.clear();
  out.pts.clear();
  out.yaw.clear();

  const double step = std::max(kSampleStepS, bspline_interval_ * 0.25);
  const int samples = static_cast<int>(std::floor(duration / step)) + 1;
  out.t.reserve(samples);
  out.pts.reserve(samples);
  out.yaw.reserve(samples);

  for (int i = 0; i < samples; ++i)
  {
    const double t_rel = std::min(duration, static_cast<double>(i) * step);
    out.t.push_back(t_rel);
    out.pts.push_back(traj.evaluateDeBoorT(t_rel));
    out.yaw.push_back(start.yaw);
  }
  if (out.t.empty())
  {
    out.t.push_back(0.0);
    out.pts.push_back(start.position);
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
      if (ellip_dist < clearance)
        return false;
    }
  }
  return true;
}

} // namespace ego_planner_driver