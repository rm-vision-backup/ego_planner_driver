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
#include <condition_variable>
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
static constexpr const char* kStatePlanFailed = "EGO_PLAN_FAILED";
static constexpr const char* kStateExecTimeout = "EGO_EXEC_TIMEOUT";
static constexpr const char* kStateTakeoff = "TAKEOFF";
static constexpr const char* kStateEmergencyBrake = "EMERGENCY_BRAKE";
static constexpr const char* kStateBrakeHold = "BRAKE_HOLD";

constexpr double kSetpointRateHz = 30.0;
// Intent freshness bound: peers keep their intent within [now-30s, now+30s]
// of the trajectory epoch. The 5.0s bound was too tight for long rolling
// trajectories (>5s execution) refreshed periodically with a fixed epoch —
// peers rejected the refresh as stale and the dynamic safe clearance kept
// the old (large) intent age.
constexpr double kNeighborStaleS = 30.0;

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
  // 测试注入：直接控制（follower PI）激活门控（等价 ~direct_control_active 消息）。
  void setDirectControlActive(bool active);

  // --- 测试/外部注入接口（线程安全） ---
  void injectLocalPose(const geometry_msgs::PoseStamped& pose);
  // Returns true only when the neighbor intent satisfies the wire/time
  // contract and supersedes the last accepted trajectory from that UAV.
  bool injectNeighborIntent(const swarm_uav_interfaces::UavTrajectoryIntent& intent);
  // GPS 时钟偏置注入（单测）：own = 本机 bias，peer = 邻机 bias（秒）。
  void injectOwnBias(double bias);
  void injectPeerBias(const std::string& uav_id, double bias);
  // 读取邻机已修正的激活时刻（本机时间轴，单测验证）。
  double neighborStartTimeForTest(const std::string& uav_id) const;
  void injectMavrosState(bool armed, bool offboard);
  void sendGoal(double x, double y, double z);
  void sendWaypoints(const std::vector<geometry_msgs::Point32>& points);
  void sendHold();
  bool replanOnce(bool timed_trigger = false);
  void advanceHoldHandling();
  void publishSetpointTick();
  void safetySupervisorTick();
  void setSafetySupervisorModeForTest(const std::string& mode);
  void setIdentityForTest(const std::string& uav_id,
                          const std::string& exec_target);
  SafetyPrediction lastSafetyPrediction() const;
  std::string execState() const;
  boost::optional<HoldTarget> steadyTarget() const;
  mavros_msgs::PositionTarget lastSetpoint() const;
  size_t setpointCount() const;
  boost::optional<TimedTrajectory> currentTrajectory() const;
  ros::Time planStart() const;
  // 测试断言：时间触发重规划是否已请求。
  bool planningRequested() const;
  // 测试/外部注入接口：执行一次周期碰撞检查（等价 collision_check_rate_hz tick），
  // 本机 current_ 与 fresh 邻机轨迹 < 固定 clearance → 置 planning_requested_（单一入口）。
  void runCollisionCheckTick();
  // 测试/外部注入接口：执行一次到达/推进检查（等价 publishSetpoint 每 tick 分支 A/B）。
  void arrivalCheckTick();
  // 首轮排序（SEQUENTIAL_START）测试接口：设置等待超时 / 判定首轮是否就绪
  // （等价 replanLoop 首轮检查，leader/已收前驱/超时 → true）。
  void setFirstPlanWaitTimeoutForTest(double timeout_s);
  bool firstPlanReadyForTest();
  // 让路候选测试接口：启用 / 查询提交状态。
  void setYieldCandidatesEnabledForTest(bool enabled);
  bool yieldCommittedForTest() const;
  // 测试用：禁用 replanLoop 线程（单测手动 replanOnce 驱动时避免线程抢占）。
  void setReplanLoopEnabled(bool enabled);
  // 软着陆测试：查询首次失败时刻（isZero = 未在软着陆失败中）。
  ros::Time planFailSinceForTest() const;

private:
  void onGoal(const geometry_msgs::PointStamped::ConstPtr& msg);
  void onGoalYaw(const std_msgs::Float64::ConstPtr& msg);
  void onWaypoints(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  void onLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void onNeighborIntent(const swarm_uav_interfaces::UavTrajectoryIntent::ConstPtr& msg);
  // GPS 时钟偏置：本机 /gps_bias 与邻机 /UAVn/gps_bias（bridge 转发）。
  void onOwnBias(const std_msgs::Float64::ConstPtr& msg);
  void onNeighborBias(const std_msgs::Float64::ConstPtr& msg,
                      const std::string& uav_id);
  // 首次收到某邻机 intent 时订阅其 /UAVn/gps_bias（bridge 转发到本机 master）。
  void ensureNeighborBiasSubscribed(const std::string& uav_id);
  // 邻机 intent 时间戳修正量：T_own = T_other - bias_other + bias_own。
  // 本机与邻机 bias 都新鲜（bias_valid_s_ 内）才修正，否则回退 0（相对时钟）。
  double timeCorrectionFor(const std::string& uav_id) const;
  bool acceptNeighborIntentLocked(
      const swarm_uav_interfaces::UavTrajectoryIntent& msg, const ros::Time& now);
  void onHold(const std_msgs::Empty::ConstPtr& msg);
  void onMavrosState(const mavros_msgs::State::ConstPtr& msg);
  void onDirectControl(const std_msgs::Bool::ConstPtr& msg);
  // Receding-horizon 窗口滚动已删除（implementation_plan_26082621）：plan-exec
  // 循环对齐原版 A/B/C 分支，任务航点 = 相邻下一个 keypoints（waypoint_consumed_）。
  // Advance the current target keypoint past keypoints the UAV has already
  // reached (< reach_thresh_m_).  Too-close targets are discarded; the new
  // target never exceeds max_advance_dist_m_ from the live pose.  Returns true
  // when the target index moved (caller requests a replan).
  bool advanceTargetLocked(const Eigen::Vector3d& pos);
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
  // 周期碰撞检查（collision_check_rate_hz）：本机 current_ 与全部 fresh 邻机
  // 轨迹同刻距离 < 固定 clearance → planning_requested_（implementation_plan_26082602）。
  void collisionCheckTick(const ros::TimerEvent& event);
  // 提交候选让路轨迹（替换 current_ + 发布 intent）。调用方持 mutex_。
  void commitYieldCandidate(const YieldCandidate& cand);
  // 判定 primary peer 是否近似悬停（末段速度 < hovering_velocity_mps）。
  bool isPeerHovering(const NeighborTrajectory& nb) const;
  // 方案 A（§10 扩展）：replanOnce 规划失败（safe_candidate=false）时尝试让路候选：
  // SLOW 降 cruise_velocity / LEFT/RIGHT waypoints 横向偏移后重试 planOnce，
  // 验证 min_clear ≥ 1.2×clearance，cost 最小者提交。成功返回 true。
  struct ReplanSnapshot
  {
    GoalPoint start;
    std::vector<WayPoint> waypoints;
    std::vector<NeighborTrajectory> neighbors;
    ros::Time now;
    Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
    bool final_segment = false;
    uint64_t safety_generation = 0;
  };
  bool tryYieldReplan(const ReplanSnapshot& snap);
  void checkArrival(const Eigen::Vector3d& target, double yaw, const ros::Time& now);
  // 方案 Y：EMERGENCY 且 ttc 紧急时生成并提交受约束制动轨迹（replan 线程内调用，
  // 持 mutex_）。返回 true 表示已提交制动/fallback；false 表示不满足制动条件
  // （ttc 充足 → 调用方 PLAN_FAILED 收口）。
  bool tryEmergencyBrakeLocked();
  void processHoldLocked();
  void handleMavrosStateEdgeLocked();
  void publishIntent(const TimedTrajectory& traj, const ros::Time& base);
  void publishIntent(const TimedTrajectory& traj, const ros::Time& base,
                     uint8_t phase);
  void appendSample(swarm_uav_interfaces::UavTrajectoryIntent& msg,
                    const TimedTrajectory& traj, size_t i);
  void replanLoop();
  std::vector<PeerTrajectoryInput> safetyPeersLocked() const;
  std::string formatSafetyLog(const SafetyPrediction& prediction,
                              const ros::Time& now,
                              double self_odom_dev_m) const;

  std::unique_ptr<EgoPlannerCore> core_;
  std::string frame_id_;
  std::string uav_id_;
  std::string exec_target_;
  double setpoint_rate_ = kSetpointRateHz;
  double pose_timeout_ = 1.0;
  double stable_s_ = 1.0;
  double timeout_margin_ = 5.0;
  double replan_period_s_ = 1.0;
  double no_replan_thresh_m_ = 1.0;
  // plan-exec 循环 A/B/C 分支阈值（implementation_plan_26082621）：
  double reach_thresh_m_ = 0.5;         // 到当前目标 <0.5m → 切下一关键点
  double max_advance_dist_m_ = 3.5;     // 推进最远使用 3.5m 远的 keypoint
  // 到达最终关键点阈值 0.2m（26082621 复盘放宽）：PX4 SITL 稳态位置偏差实测
  // 0.13-0.17m，0.1m 阈值导致长距离任务末段停稳后永不触发 COMPLETED → 5s 超时
  // HOLD；0.2m 匹配 executor position_tolerance_m=0.2 惯例，COMPLETED 可靠触发。
  double arrival_reach_thresh_m_ = 0.2;
  int activation_replan_attempts_ = 3;
  bool direct_control_active_ = false;
  volatile bool replan_loop_enabled_ = true;
  std::string safety_supervisor_mode_ = "active";
  double safety_supervisor_rate_hz_ = 25.0;
  double episode_release_hysteresis_s_ = 1.0;
  SafetyPredictorConfig safety_config_;
  std::unique_ptr<SafetyPredictor> safety_predictor_;
  std::unique_ptr<EmergencyBrakeGenerator> emergency_brake_generator_;
  SafetyLogger safety_logger_;
  ConflictEpisode conflict_episode_;
  SafetyPrediction last_safety_prediction_;
  uint64_t safety_generation_ = 0;
  bool emergency_braking_ = false;
  // 对齐原版 EGOReplanFSM：EMERGENCY 时先尝试立即重规划（planFromCurrentTraj
  // 语义）；重规划失败且预测越界时间 < emergency_replan_time_s 才触发受约束
  // 制动，否则收口 PLAN_FAILED（对齐原版 emergency_time=1.0s）。
  double emergency_replan_time_s_ = 1.0;
  // 方案 Y（implementation_plan_26082621）：监督层检测到 EMERGENCY 只置位
  // emergency_pending_（记录 primary peer + ttc）并唤醒 replanLoop；制动由
  // replan 线程在"常规 replan 失败且 ttc < emergency_replan_time_s_"时提交
  // （提交唯一化，监督回调不再直接改轨迹）。
  bool emergency_pending_ = false;
  std::string emergency_peer_id_;
  double emergency_peer_ttc_ = std::numeric_limits<double>::infinity();
  // replanOnce 失败时是否置 PLAN_FAILED+HOLD 收口（常规 replan 为 true）；
  bool replan_failure_fatal_ = true;
  // 软着陆（26082700 EGO_PLAN_FAILED 修复）：常规 replan 失败不立即 PLAN_FAILED，
  // 保持当前轨迹（执行完自然悬停），由 1s 定时/事件继续重试；连续失败超过
  // plan_fail_timeout_s_（10s）才 EGO_PLAN_FAILED 收口（executor 收口后经
  // backend 对全组活动机发 HOLD）。
  double plan_fail_timeout_s_ = 10.0;
  ros::Time plan_fail_since_;  // 首次失败时刻（isZero = 当前不在软着陆失败中）
  // 机间让路候选（SLOW/LEFT/RIGHT + right-of-way，implementation_plan_26082602 §10）：
  // collisionCheckTick 检测冲突（min_clear < 1.2×clearance，yield_clearance_factor=1.2）
  // → 本机 YIELD（ID 小于 primary peer）生成候选，cost 最小者提交发布；STAND_ON 交常规 replan。
  YieldCandidateConfig yield_config_;
  std::unique_ptr<YieldCandidateGenerator> yield_generator_;
  bool enable_yield_candidates_ = false;
  bool yield_committed_ = false;
  YieldDirection yield_direction_ = YieldDirection::NONE;
  int32_t yield_peer_traj_id_ = -1;  // 锁存的 primary peer 轨迹版本（防重复提交）
  // GPS 时钟偏置（秒）：own = 本机 /gps_bias，peer = 邻机 /UAVn/gps_bias。
  // 新鲜度由各自接收时刻判定；超 bias_valid_s_ 视为失效回退相对时钟。
  double own_bias_s_ = 0.0;
  ros::Time own_bias_received_;
  std::map<std::string, std::pair<double, ros::Time>> peer_bias_;
  double bias_valid_s_ = 30.0;
  std::map<std::string, ros::Subscriber> neighbor_bias_subs_;

  bool armed_ = false;
  bool offboard_ = false;
  bool prev_arm_offboard_ = false;
  double takeoff_height_ = 5.0;
  double takeoff_ascend_speed_ = 1.0;
  bool takeoff_active_ = false;
  double takeoff_start_z_ = 0.0;
  ros::Time takeoff_start_time_;

  // 对齐原版 SEQUENTIAL_START：首轮（current_ 为空）规划前等待前驱无人机
  // intent（drone_id 顺序，前机轨迹到达才首轮规划）。leader(id<=1) 或已收到
  // 前驱或超时 first_plan_wait_timeout_s 后放行；否则 replanLoop 跳过本轮。
  double first_plan_wait_timeout_s_ = 10.0;
  bool prev_intent_received_ = false;
  ros::Time first_plan_requested_at_;
  bool firstPlanReadyLocked(const ros::Time& now) const;

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
  ros::Subscriber own_bias_sub_;
  ros::Timer timer_;
  ros::Timer safety_timer_;
  // Periodic collision check (collision_check_rate_hz): replan trigger aligned
  // with upstream's independent safety timer. Peers refresh intent on replan
  // commit only (publish-on-commit); the check uses the buffered neighbor
  // trajectories so a stopped peer stream is still detected.
  ros::Timer collision_check_timer_;
  double collision_check_rate_hz_ = 10.0;
  // Serializes traj_id_ increment + intent_pub_ publish across the replan
  // thread and the safety-timer emergency-brake commit.
  std::mutex intent_pub_mutex_;

  mutable std::mutex mutex_;
  // 事件唤醒 replanLoop（单一 replan 入口）：planning_requested_ /
  // emergency_pending_ 置位即通知；常态 1s 定时由 wait_for 超时兜底。
  std::condition_variable replan_cv_;
  std::thread replan_thread_;
  bool stop_ = false;

  double goal_yaw_ = 0.0;
  boost::optional<GoalPoint> pending_goal_;
  std::vector<WayPoint> waypoints_;
  // Index into waypoints_ of the current target keypoint (adjacent-next
  // keypoint, plan-exec branch A).  Initialized past the start keypoint
  // (waypoints_[0]); advanced by advanceTargetLocked when the UAV reaches the
  // current target within reach_thresh_m_.  Never erased with the previous
  // planning segment, so the target cannot outrun the executed trajectory.
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
