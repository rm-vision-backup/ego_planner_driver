#ifndef EGO_PLANNER_DRIVER_EGO_PLANNER_DRIVER_NODE_H_
#define EGO_PLANNER_DRIVER_EGO_PLANNER_DRIVER_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <swarm_uav_interfaces/UavTrajectoryIntent.h>
#include <mavros_msgs/PositionTarget.h>
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
#include "ego_planner_driver/emergency_brake.h"
#include "ego_planner_driver/safety_logger.h"
#include "ego_planner_driver/safety_predictor.h"
#include "ego_planner_driver/yield_candidates.h"

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
static constexpr const char* kStateEmergencyBrake = "EMERGENCY_BRAKE";
static constexpr const char* kStateBrakeHold = "BRAKE_HOLD";
static constexpr const char* kStateNegotiating = "NEGOTIATING";

constexpr double kSetpointRateHz = 30.0;
constexpr double kNeighborStaleS = 5.0;
constexpr double kReplanPeriodS = 0.5;

// PositionTarget type masks (mavros_msgs constants):
//   EGO trajectory: position + velocity + acceleration + yaw, yaw rate ignored.
//   HOLD/TAKEOFF/steady: position + yaw only, velocity/acceleration ignored.
constexpr uint16_t kMaskEgoTrajectory =
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;  // 2048
constexpr uint16_t kMaskHold =
    mavros_msgs::PositionTarget::IGNORE_VX |
    mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;  // 2552

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
  // 测试注入：设置滚动局部规划范围（等价 launch 的 ~planning_horizon）。
  void setPlanningHorizon(double horizon);
  // 测试注入：直接控制（follower PI）激活门控（等价 ~direct_control_active 消息）。
  void setDirectControlActive(bool active);

  // --- 测试/外部注入接口（线程安全） ---
  void injectLocalPose(const geometry_msgs::PoseStamped& pose);
  // Returns true only when the neighbor intent satisfies the wire/time
  // contract and supersedes the last accepted trajectory from that UAV.
  bool injectNeighborIntent(const swarm_uav_interfaces::UavTrajectoryIntent& intent);
  void injectMavrosState(bool armed, bool offboard);
  void sendGoal(double x, double y, double z);
  void sendWaypoints(const std::vector<geometry_msgs::Point32>& points);
  void sendHold();
  bool replanOnce();
  void advanceHoldHandling();
  void publishSetpointTick();
  void safetySupervisorTick();
  void setSafetySupervisorModeForTest(const std::string& mode);
  void setIntentNegotiationWaitForTest(double seconds);
  void setIdentityForTest(const std::string& uav_id,
                          const std::string& exec_target);
  SafetyPrediction lastSafetyPrediction() const;
  bool yieldCommittedForTest() const;
  std::string execState() const;
  boost::optional<HoldTarget> steadyTarget() const;
  mavros_msgs::PositionTarget lastSetpoint() const;
  size_t setpointCount() const;
  boost::optional<TimedTrajectory> currentTrajectory() const;
  ros::Time planStart() const;
  // 测试断言：时间触发重规划是否已请求。
  bool planningRequested() const;
  // 测试用：禁用 replanLoop 线程（单测手动 replanOnce 驱动时避免线程抢占）。
  void setReplanLoopEnabled(bool enabled);

private:
  void onGoal(const geometry_msgs::PointStamped::ConstPtr& msg);
  void onGoalYaw(const std_msgs::Float64::ConstPtr& msg);
  void onWaypoints(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  void onLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void onNeighborIntent(const swarm_uav_interfaces::UavTrajectoryIntent::ConstPtr& msg);
  bool acceptNeighborIntentLocked(
      const swarm_uav_interfaces::UavTrajectoryIntent& msg, const ros::Time& now);
  void onHold(const std_msgs::Empty::ConstPtr& msg);
  void onMavrosState(const mavros_msgs::State::ConstPtr& msg);
  void onDirectControl(const std_msgs::Bool::ConstPtr& msg);
  // Receding-horizon: from `waypoints_[consumed_..]`, keep only the points
  // ahead of start within horizon (window slides with the UAV's real arc
  // progress, not with the previous planning segment).
  std::vector<WayPoint> truncateToHorizon(
      const std::vector<WayPoint>& waypoints, double horizon,
      const Eigen::Vector3d& start, size_t consumed) const;
  // Advance `consumed` to the first waypoint whose arc-length projection is
  // still ahead of start (drop points the UAV has already passed).
  size_t advanceConsumed(const std::vector<WayPoint>& waypoints,
                         const Eigen::Vector3d& start,
                         size_t consumed) const;
  // Samples the full position/velocity/acceleration/yaw of the current
  // trajectory at replan time (clamped to its end) so a rolling segment starts
  // from the same trajectory state it replaces (upstream FSM semantics)
  // instead of mixing live position with stale derivatives.  Returns false
  // when no usable trajectory exists (first plan / new task).
  bool sampleCurrentState(Eigen::Vector3d& pos, Eigen::Vector3d& vel,
                          Eigen::Vector3d& acc, double& yaw) const;
  // Caps the rolling segment's start derivatives to the planner bounds by
  // Euclidean norm so a pathological sampled derivative cannot spread the
  // pinned control points and amplify into a large setpoint overshoot.
  void clampStartDerivatives(Eigen::Vector3d& vel, Eigen::Vector3d& acc) const;
  // Convert absolute neighbor activation epochs onto a candidate trajectory's
  // relative axis (candidate activation == 0). This must be repeated after
  // optimization because planning latency advances every moving neighbor.
  static std::vector<NeighborTrajectory> rebaseNeighbors(
      const std::vector<NeighborTrajectory>& absolute_neighbors,
      const ros::Time& candidate_activation);
  std::vector<NeighborTrajectory> freshAbsoluteNeighborsLocked(
      const ros::Time& now) const;

  ros::Time nowFunc() const { return clock_source_ ? clock_source_() : ros::Time::now(); }
  void resetRun();
  void setState(const std::string& state);
  bool captureHoldTargetLocked(const ros::Time& now);
  // Unified command construction/publish: business fields stay ROS ENU, MAVROS
  // converts them to NED for the PX4 raw local setpoint.  velocity/acceleration
  // are only meaningful for kMaskEgoTrajectory (ignored by HOLD masks).
  void publishPositionTarget(double x, double y, double z, double yaw,
                             uint16_t type_mask,
                             const Eigen::Vector3d& vel = Eigen::Vector3d::Zero(),
                             const Eigen::Vector3d& acc = Eigen::Vector3d::Zero());
  static double yawFromQuat(const geometry_msgs::Quaternion& q);
  void publishSetpoint(const ros::TimerEvent& event);
  void runSafetySupervisor(const ros::TimerEvent& event);
  void checkArrival(const Eigen::Vector3d& target, double yaw, const ros::Time& now);
  void processHoldLocked();
  void handleMavrosStateEdgeLocked();
  void publishIntent(const TimedTrajectory& traj, const ros::Time& base);
  void publishIntent(const TimedTrajectory& traj, const ros::Time& base,
                     uint8_t phase);
  bool activateTentativeLocked(const ros::Time& now);
  static void appendSample(swarm_uav_interfaces::UavTrajectoryIntent& msg,
                           const TimedTrajectory& traj, size_t i);
  void replanLoop();
  std::vector<PeerTrajectoryInput> safetyPeersLocked() const;
  std::string formatSafetyLog(const SafetyPrediction& prediction,
                              const ros::Time& now,
                              double self_odom_dev_m) const;
  // 缺陷 2：已提交让路候选后，仅当邻机 intent 版本有更新或当前候选已恶化到
  // emergency 边界内时才重新评估；否则保持以避免 25Hz 重复切轨迹。
  bool peerVersionsUnchangedLocked() const;
  std::map<std::string, int32_t> peerVersionsSnapshotLocked() const;

  std::unique_ptr<EgoPlannerCore> core_;
  std::string frame_id_;
  std::string uav_id_;
  std::string exec_target_;
  double setpoint_rate_ = kSetpointRateHz;
  double pose_timeout_ = 1.0;
  double pos_tol_ = 0.2;
  double stable_s_ = 1.0;
  double timeout_margin_ = 5.0;
  double planning_horizon_ = 0.0;
  double replan_period_s_ = 1.0;
  double no_replan_thresh_m_ = 1.0;
  int activation_replan_attempts_ = 3;
  bool direct_control_active_ = false;
  volatile bool replan_loop_enabled_ = true;
  std::string safety_supervisor_mode_ = "off";
  double safety_supervisor_rate_hz_ = 25.0;
  double episode_release_hysteresis_s_ = 1.0;
  SafetyPredictorConfig safety_config_;
  std::unique_ptr<SafetyPredictor> safety_predictor_;
  std::unique_ptr<EmergencyBrakeGenerator> emergency_brake_generator_;
  std::unique_ptr<YieldCandidateGenerator> yield_candidate_generator_;
  bool enable_yield_candidates_ = true;
  boost::optional<YieldManeuver> episode_maneuver_;
  bool yield_committed_ = false;
  // 提交让路候选时记录的邻机 intent 版本（uav_id -> traj_id），用于缺陷 2
  // 的"peer 更新才重新评估"判断。
  std::map<std::string, int32_t> yield_peer_versions_;
  SafetyLogger safety_logger_;
  ConflictEpisode conflict_episode_;
  SafetyPrediction last_safety_prediction_;
  uint64_t safety_generation_ = 0;
  bool emergency_braking_ = false;
  double intent_negotiation_wait_s_ = 1.0;
  std::shared_ptr<TimedTrajectory> tentative_;
  ros::Time tentative_created_;
  uint64_t tentative_generation_ = 0;

  bool armed_ = false;
  bool offboard_ = false;
  bool prev_arm_offboard_ = false;
  double takeoff_height_ = 5.0;
  double takeoff_ascend_speed_ = 1.0;
  bool takeoff_active_ = false;
  double takeoff_start_z_ = 0.0;
  ros::Time takeoff_start_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string setpoint_topic_;
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
  ros::Subscriber direct_control_sub_;
  ros::Timer timer_;
  ros::Timer safety_timer_;

  mutable std::mutex mutex_;
  std::thread replan_thread_;
  bool stop_ = false;

  double goal_yaw_ = 0.0;
  boost::optional<GoalPoint> pending_goal_;
  std::vector<WayPoint> waypoints_;
  // Index into waypoints_ of the first waypoint still ahead of the UAV
  // (receding-horizon window start).  Advanced from the UAV's real position
  // each replan, never by erasing previously planned segments, so the window
  // cannot outrun the executed trajectory.
  size_t waypoint_consumed_ = 0;
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

  size_t setpoint_count_ = 0;
  boost::optional<mavros_msgs::PositionTarget> last_setpoint_;

  ClockFn clock_source_;
  // Last accepted (epoch, sequence) per neighbor. A newer epoch establishes a
  // new sequence generation, allowing process restarts and int32 wraparound.
  std::map<std::string, std::pair<double, int32_t>> latest_neighbor_version_;
  std::map<std::string, std::pair<NeighborTrajectory, ros::Time>> inbox_;
  std::map<std::string, uint8_t> neighbor_phase_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_EGO_PLANNER_DRIVER_NODE_H_
