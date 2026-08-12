#ifndef EGO_PLANNER_DRIVER_EGO_PLANNER_DRIVER_NODE_H_
#define EGO_PLANNER_DRIVER_EGO_PLANNER_DRIVER_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <swarm_uav_interfaces/UavTrajectoryIntent.h>
#include <mavros_msgs/State.h>
#include <boost/optional.hpp>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "ego_planner_driver/ego_planner_core.h"

using ego_planner_driver::EgoPlannerConfig;
using ego_planner_driver::EgoPlannerCore;
using ego_planner_driver::GoalPoint;
using ego_planner_driver::NeighborTrajectory;
using ego_planner_driver::TimedTrajectory;
using ego_planner_driver::WayPoint;

namespace ego_planner_driver
{
// 状态字符串与 /exec_state 发布值一一对应
static constexpr const char* kStateIdle = "IDLE";
static constexpr const char* kStateExecuting = "EXECUTING";
static constexpr const char* kStateCompleted = "COMPLETED";
static constexpr const char* kStateHold = "HOLD";
static constexpr const char* kStatePoseStale = "POSE_STALE";
static constexpr const char* kStatePlanFailed = "EGO_PLAN_FAILED";
static constexpr const char* kStateExecTimeout = "EGO_EXEC_TIMEOUT";
static constexpr const char* kStateTakeoff = "TAKEOFF";

constexpr double kSetpointRateHz = 30.0;
constexpr double kNeighborStaleS = 5.0;
constexpr double kReplanPeriodS = 0.5;
constexpr double kIdleHoverRefreshHz = 1.0;   // IDLE hover 目标每秒刷新一次
constexpr double kTerminalToIdleS = 2.0;      // terminal(COMPLETED/HOLD) → IDLE 自动转换窗口

struct HoldTarget
{
  Eigen::Vector3d pos;
  double yaw;
};

class EgoPlannerDriverNode
{
public:
  EgoPlannerDriverNode();
  ~EgoPlannerDriverNode();

  // 可编程时钟注入（单测用）；默认 ros::Time::now()。
  using ClockFn = std::function<ros::Time()>;
  void setClockSource(ClockFn fn);

  // --- 测试/外部注入接口（线程安全） ---
  void injectLocalPose(const geometry_msgs::PoseStamped& pose);
  void injectMavrosState(bool armed, bool offboard);
  void sendGoal(double x, double y, double z);
  void sendHold();
  bool replanOnce();
  void advanceHoldHandling();
  void publishSetpointTick();
  std::string execState() const;
  boost::optional<HoldTarget> steadyTarget() const;
  boost::optional<Eigen::Vector3d> idleHoverTarget() const;
  ros::Time idleHoverUpdatedAt() const;
  geometry_msgs::PoseStamped lastSetpoint() const;
  size_t setpointCount() const;

private:
  void onGoal(const geometry_msgs::PointStamped::ConstPtr& msg);
  void onGoalYaw(const std_msgs::Float64::ConstPtr& msg);
  void onWaypoints(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  void onLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void onNeighborIntent(const swarm_uav_interfaces::UavTrajectoryIntent::ConstPtr& msg);
  void onHold(const std_msgs::Empty::ConstPtr& msg);
  void onMavrosState(const mavros_msgs::State::ConstPtr& msg);

  ros::Time nowFunc() const { return clock_source_ ? clock_source_() : ros::Time::now(); }
  void resetRun();
  void setState(const std::string& state);
  void publishPose(double x, double y, double z, double yaw);
  static double yawFromQuat(const geometry_msgs::Quaternion& q);
  void publishSetpoint(const ros::TimerEvent& event);
  void refreshIdleHoverTarget(const ros::Time& now);
  void scheduleTerminalToIdle(const std::string& terminal_state);
  void checkArrival(const Eigen::Vector3d& target, double yaw, const ros::Time& now);
  void processHoldLocked();
  void handleMavrosStateEdgeLocked();
  void publishIntent(const TimedTrajectory& traj, const ros::Time& base);
  static void appendSample(swarm_uav_interfaces::UavTrajectoryIntent& msg,
                           const TimedTrajectory& traj, size_t i);
  void replanLoop();

  std::unique_ptr<EgoPlannerCore> core_;
  std::string frame_id_;
  std::string uav_id_;
  std::string exec_target_;
  double setpoint_rate_ = kSetpointRateHz;
  double pose_timeout_ = 1.0;
  double pos_tol_ = 0.2;
  double stable_s_ = 1.0;
  double timeout_margin_ = 5.0;
  double terminal_to_idle_s_ = kTerminalToIdleS;
  double idle_hover_refresh_hz_ = kIdleHoverRefreshHz;

  // --- TAKEOFF 状态相关 ---
  bool armed_ = false;
  bool offboard_ = false;
  bool prev_arm_offboard_ = false;  // 上一次的 arm&&offboard 状态（边沿检测）
  double takeoff_height_ = 5.0;        // 起飞高度（相对当前 Z 增量，m）
  double takeoff_ascend_speed_ = 1.0;  // 上升速度（m/s）
  bool takeoff_active_ = false;        // 是否已进入 arm+OFFBOARD 触发
  double takeoff_start_z_ = 0.0;       // 起飞起始 Z
  ros::Time takeoff_start_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher setpoint_pub_;
  ros::Publisher intent_pub_;
  ros::Publisher state_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber goal_yaw_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber intent_sub_;
  ros::Subscriber hold_sub_;
  ros::Subscriber mavros_state_sub_;
  ros::Timer timer_;

  mutable std::mutex mutex_;
  std::thread replan_thread_;
  bool stop_ = false;

  double goal_yaw_ = 0.0;
  boost::optional<GoalPoint> pending_goal_;
  std::vector<WayPoint> waypoints_;
  boost::optional<geometry_msgs::PoseStamped> local_pose_;
  boost::optional<ros::Time> local_pose_stamp_;
  bool planning_requested_ = false;
  bool hold_requested_ = false;

  std::shared_ptr<TimedTrajectory> current_;
  ros::Time plan_start_;
  int32_t traj_id_ = 0;
  std::string exec_state_;
  boost::optional<HoldTarget> steady_target_;
  boost::optional<ros::Time> arrival_since_;

  // IDLE hover 目标（每秒更新为当前位姿，避免切 OFFBOARD 时 setpoint 跳变）
  boost::optional<Eigen::Vector3d> idle_hover_target_;
  ros::Time idle_hover_updated_at_;

  // terminal(COMPLETED/HOLD) → IDLE 自动转换窗口；"" 表示不调度
  std::string scheduled_revert_state_;
  ros::Time terminal_entered_at_;

  size_t setpoint_count_ = 0;
  boost::optional<geometry_msgs::PoseStamped> last_setpoint_;

  ClockFn clock_source_;
  std::map<std::string, int32_t> latest_traj_id_;
  std::map<std::string, std::pair<NeighborTrajectory, ros::Time>> inbox_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_EGO_PLANNER_DRIVER_NODE_H_