#ifndef EGO_PLANNER_CORE_H_
#define EGO_PLANNER_CORE_H_

#include <Eigen/Eigen>
#include <vector>

#include "ego_planner_driver/lbfgs.hpp"
#include "ego_planner_driver/uniform_bspline.h"

namespace ego_planner_driver
{

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
  // Per-aircraft clearance radius. Reciprocal checks use twice this value;
  // 0.5 m therefore enforces 1.0 m horizontal centre distance and, with the
  // existing 2:1 ellipsoid, 2.0 m vertical centre distance.
  double swarm_clearance = 0.5;
  double planning_range = 30.0; // neighbor trajectories beyond this are ignored
  double lambda_smooth = 1.0;
  double lambda_swarm = 1.0;
  double lambda_feasibility = 1.0;
  double lambda_terminal = 1.0;
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
  std::vector<double> t;
  std::vector<Eigen::Vector3d> pts;
};

struct TimedTrajectory
{
  double duration = 0.0;
  std::vector<double> t;                 // relative seconds, dense samples
  std::vector<Eigen::Vector3d> pts;      // global ENU, same length as t
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
};

class EgoPlannerCore
{
public:
  explicit EgoPlannerCore(const EgoPlannerConfig& config);

  // Prepares an initial B-spline through the sparse key points, optimizes it
  // with the reciprocal swarm + smoothness + feasibility costs and samples
  // the resulting timed trajectory.  start is the current pose and is
  // implicitly the first way point.
  bool planOnce(const GoalPoint& start,
                const std::vector<WayPoint>& sparse_waypoints,
                const std::vector<NeighborTrajectory>& neighbors,
                TimedTrajectory& out, double& final_cost);

  // Exposes the reciprocal clearance check for tests and gates.
  bool checkClearance(const TimedTrajectory& trajectory,
                      const std::vector<NeighborTrajectory>& neighbors,
                      double clearance) const;

  EgoPlannerConfig config() const { return config_; }

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
