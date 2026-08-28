#ifndef EGO_PLANNER_CORE_H_
#define EGO_PLANNER_CORE_H_

#include <Eigen/Eigen>
#include <cstddef>
#include <vector>

#include "ego_planner_driver/lbfgs.hpp"
#include "ego_planner_driver/uniform_bspline.h"

namespace ego_planner_driver
{

// Limits a 3-vector's Euclidean norm to `limit` without touching its direction.
// Zero vectors stay zero (never NaN, unlike v.normalized()).
inline void clampVectorNorm(Eigen::Vector3d& v, double limit)
{
  const double n = v.norm();
  if (n > limit && n > 1e-9)
    v *= limit / n;
}

// 内部规划结果分类（implementation_plan_26082722 §5）。这些值只供 node 分流
// 使用，不进入 ROS /exec_state，不改变 executor/backend 协议。
enum class PlanResult
{
  SUCCESS,              // 完整轨迹生成成功
  DEGENERATE_SEGMENT,   // planned start 到当前 target 方向退化，不生成新段
  RETRYABLE_FAILURE,    // 普通有限数值输入无法形成合法轨迹（可重试）
  RESOURCE_LIMIT        // 资源上限/checked arithmetic/bad_alloc，立即收口
};

// 编译期绝对 hard limits（资源配置不得超过；见 §11.3 fail-fast）。
constexpr std::size_t kMaxArcSamplesAbsHard = 32768;              // 细折线采样绝对上限
constexpr std::size_t kMaxParameterizationPointsAbsHard = 1024;   // B-spline 输入点集绝对上限
constexpr std::size_t kMaxTrajectorySamplesAbsHard = 65536;       // TimedTrajectory 样本绝对上限
constexpr std::size_t kMaxNeighborIntentSamplesAbsHard = 4096;    // 邻机 intent 样本绝对上限

// Extracted, no-ROS numerical core of the ego-planner-swarm kernel.
// Keeps three cost terms only: jerk smoothness, dynamic feasibility and
// reciprocal swarm avoidance.  Static obstacle / moving object / fitness
// terms are intentionally excluded (this scenario has no static obstacles).
struct EgoPlannerConfig
{
  int order = 3;
  double ts = 0.4;              // B-spline knot span (s)
  double max_vel = 8.0;         // m/s
  double max_acc = 6.0;         // m/s^2
  // 资源安全边界（implementation_plan_26082722 §11）。node 启动时 fail-fast
  // 校验：所有 count limit > 0 且不超过相应编译期绝对 hard limit。
  std::size_t max_arc_samples = 4096;              // §6 细折线采样上限（0.02s 步长）
  std::size_t max_parameterization_points = 256;   // §6.3 弧长重采样送入 B-spline 的点数上限
  std::size_t max_trajectory_samples = 4096;       // §7 最终 TimedTrajectory 离散样本上限
  std::size_t max_neighbor_intent_samples = 256;   // §10 每条邻机 intent 保留样本上限
  double segment_direction_epsilon_m = 0.05;       // §5.2 方向有效性阈值（非任务到达容差）
  // Permitted ratio by which vel/acc may exceed limits before time reparam
  // triggers (upstream fsm: manager/feasibility_tolerance = 0.05).
  double feasibility_tolerance = 0.05;
  // Receding-horizon planning range [m]: only the waypoints ahead of the
  // current pose within planning_horizon are parameterized per plan, aligned
  // with upstream ego-planner-swarm. <= 0 plans the full route at once
  // (global-planning regression mode).
  double planning_horizon = 0.0;
  // Per-aircraft clearance radius. Reciprocal checks use twice this value;
  // 0.5 m therefore enforces 1.0 m centre distance in every axis (symmetric
  // clearance sphere; the vertical axis was reduced from 2.0 m to 1.0 m).
  // Fixed clearance: prediction ability comes from same-instant sampling of
  // GPS-bias-corrected neighbor intents, not from a data-age dynamic gate.
  double swarm_clearance = 0.5;
  double planning_range = 30.0; // neighbor trajectories beyond this are ignored
  double lambda_smooth = 1.0;
  double lambda_swarm = 1.0;
  double lambda_feasibility = 1.0;
  double lambda_terminal = 1.0;
  // Rebound hard push: after L-BFGS, control points that intrude a neighbor's
  // clearance are pushed out along the waypoint-normal direction (waypoints
  // act as the safe-path reference, replacing the upstream A* path).
  // Disabled by default (min-snap refactor): inter-UAV safety is handled by
  // the runtime distance gate in the execution layer, not the optimizer.
  bool enable_rebound = false;
  // Time reparameterization (setPhysicalLimits/checkFeasibility/lengthenTime)
  // is disabled by default: the min-snap 0.4 m arc-length point set already
  // yields ~ts-consistent control-point spacing.  When disabled the limits are
  // still checked for diagnostics but the time axis is never lengthened.
  bool enable_time_reparam = false;
  // Cruise speed used for the segment end velocity (end_vel = tangent x
  // min(v_cruise, max_vel)).  Only the final segment stops (end_vel = 0).
  double cruise_velocity = 1.0;
  // Acceleration limit for the trapezoidal time estimator (T = f(v0, dist, a)).
  double acceleration_limit = 2.0;
  int max_iterations = 200;
  int mem_size = 16;
  double g_epsilon = 0.01;
};

struct GoalPoint
{
  Eigen::Vector3d position;
  double yaw = 0.0;
};

struct WayPoint
{
  Eigen::Vector3d position;
  double yaw = 0.0;
};

// A neighbor time-stamped trajectory sampled at discrete instants.
// t[i] is relative to start_time (global seconds).
struct NeighborTrajectory
{
  int drone_id = -1;
  double start_time = 0.0;
  double duration = 0.0;
  double clearance = 0.0;   // <=0 means this neighbor does not participate
  // Data age of the intent at the snapshot instant [s]. Retained for the
  // supervisor diagnostic only; the planner consumes a fixed clearance and no
  // longer reacts to intent age (implementation_plan_26082602).
  double data_age_s = 0.0;
  std::vector<double> t;
  std::vector<Eigen::Vector3d> pts;
};

// One-tick trajectory sample: position/velocity/acceleration/yaw read from the
// same trajectory at the same relative time (PX4 raw local setpoint contract).
struct TrajectorySample
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

struct TimedTrajectory
{
  double duration = 0.0;
  std::vector<double> t;                 // relative seconds, dense samples
  std::vector<Eigen::Vector3d> pts;      // global ENU, same length as t
  std::vector<Eigen::Vector3d> vel;      // B-spline first derivative, same length
  std::vector<Eigen::Vector3d> acc;      // B-spline second derivative, same length
  std::vector<double> yaw;               // same length as t

  bool sample(double t_rel, Eigen::Vector3d& out) const
  {
    if (pts.empty())
      return false;
    if (t_rel <= t.front())
    {
      out = pts.front();
      return true;
    }
    if (t_rel >= t.back())
    {
      out = pts.back();
      return true;
    }
    for (size_t i = 0; i + 1 < t.size(); ++i)
    {
      if (t_rel < t[i + 1])
      {
        double a = (t_rel - t[i]) / (t[i + 1] - t[i]);
        out = pts[i] * (1.0 - a) + pts[i + 1] * a;
        return true;
      }
    }
    return false;
  }

  // Samples position/velocity/acceleration/yaw from the same trajectory at the
  // same relative time so the published PositionTarget carries internally
  // consistent derivatives (no cross-frame numeric differentiation).
  bool samplePVA(double t_rel, TrajectorySample& out) const
  {
    if (t.empty() || pts.size() != t.size() || vel.size() != t.size() ||
        acc.size() != t.size() || yaw.size() != t.size())
      return false;
    if (t.size() == 1)
    {
      out.pos = pts.front();
      out.vel = vel.front();
      out.acc = acc.front();
      out.yaw = yaw.front();
      return true;
    }
    if (t_rel <= t.front())
    {
      out.pos = pts.front();
      out.vel = vel.front();
      out.acc = acc.front();
      out.yaw = yaw.front();
      return true;
    }
    if (t_rel >= t.back())
    {
      out.pos = pts.back();
      out.vel = vel.back();
      out.acc = acc.back();
      out.yaw = yaw.back();
      return true;
    }
    size_t lo = 0;
    if (t_rel > t.front())
    {
      for (size_t i = 0; i + 1 < t.size(); ++i)
      {
        if (t_rel < t[i + 1])
        {
          lo = i;
          break;
        }
      }
    }
    const size_t hi = lo + 1;
    const double span = t[hi] - t[lo];
    const double a = (span > 1e-12) ? (t_rel - t[lo]) / span : 0.0;
    out.pos = pts[lo] * (1.0 - a) + pts[hi] * a;
    out.vel = vel[lo] * (1.0 - a) + vel[hi] * a;
    out.acc = acc[lo] * (1.0 - a) + acc[hi] * a;
    out.yaw = yaw[lo] * (1.0 - a) + yaw[hi] * a;
    return true;
  }
};

class EgoPlannerCore
{
public:
  explicit EgoPlannerCore(const EgoPlannerConfig& config);

  // Prepares an initial B-spline through the sparse key points, optimizes it
  // with the reciprocal swarm + smoothness + feasibility costs and samples
  // the resulting timed trajectory.  start is the current pose and is
  // implicitly the first way point.  start_vel/start_acc are the current
  // trajectory derivatives (zero for the first segment) so rolling segments
  // join velocity- and acceleration-continuously (upstream FSM semantics).
  // The initial point_set is generated from a single-segment min-snap
  // polynomial (one_segment_traj_gen) sampled at 0.4 m arc-length intervals
  // (min-snap unified single-segment refactor).  final_segment == true makes
  // the segment end velocity zero (static arrival); otherwise end_vel follows
  // the end-point tangent at min(cruise_velocity, max_vel) with end_acc = 0.
  PlanResult planOnce(const GoalPoint& start,
                      const std::vector<WayPoint>& sparse_waypoints,
                      const std::vector<NeighborTrajectory>& neighbors,
                      TimedTrajectory& out, double& final_cost,
                      const Eigen::Vector3d& start_vel = Eigen::Vector3d::Zero(),
                      const Eigen::Vector3d& start_acc = Eigen::Vector3d::Zero(),
                      bool final_segment = false);

  // Exposes the reciprocal clearance check for tests and gates.
  bool checkClearance(const TimedTrajectory& trajectory,
                      const std::vector<NeighborTrajectory>& neighbors,
                      double clearance) const;

  EgoPlannerConfig config() const { return config_; }
  // 规划失败时让路候选（SLOW）临时降低巡航速度重试规划。
  void setCruiseVelocity(double v) { config_.cruise_velocity = v; }

  static std::vector<NeighborTrajectory> filterByRange(
      const std::vector<NeighborTrajectory>& neighbors,
      const Eigen::Vector3d& local_position, double planning_range);

private:
  struct ControlPoints
  {
    double clearance = 0.0;
    int size = 0;
    Eigen::MatrixXd points; // 3 x size, columns are control points
  };

  void combineCost(const double* x, double* grad, double& cost, int n);
  void calcSmoothnessCost(const Eigen::MatrixXd& q, double& cost,
                          Eigen::MatrixXd& gradient) const;
  void calcFeasibilityCost(const Eigen::MatrixXd& q, double& cost,
                           Eigen::MatrixXd& gradient) const;
  void calcTerminalCost(const Eigen::MatrixXd& q, double& cost,
                        Eigen::MatrixXd& gradient);
  void calcSwarmCost(const Eigen::MatrixXd& q, double& cost,
                     Eigen::MatrixXd& gradient);
  static double costFunction(void* data, const double* x, double* grad,
                             const int n);
  // Rebound: push control points that intrude a neighbor's clearance out along
  // the waypoint-normal direction. waypoints (densified keypoints) act as the
  // safe-path reference in place of the upstream A* path.
  bool reboundFromNeighbors(const std::vector<NeighborTrajectory>& neighbors,
                            const std::vector<Eigen::Vector3d>& waypoints,
                            double clearance);
  static Eigen::Vector3d waypointTangent(
      const std::vector<Eigen::Vector3d>& waypoints,
      const Eigen::Vector3d& pos);
  // Run one L-BFGS pass over the current control points (used both by planOnce
  // and by the rebound re-optimization loop so the spline re-adheres to the
  // rebound control points).
  bool optimizeControls(double& final_cost);

  EgoPlannerConfig config_;
  double bspline_interval_ = 0.4;
  int order_ = 3;
  int variable_num_ = 0;
  ControlPoints cps_;
  std::vector<NeighborTrajectory> neighbors_;
  Eigen::Vector3d local_target_pt_ = Eigen::Vector3d::Zero();
};

} // namespace ego_planner_driver

#endif // EGO_PLANNER_CORE_H_
