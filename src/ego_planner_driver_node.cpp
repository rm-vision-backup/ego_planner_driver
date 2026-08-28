#include "ego_planner_driver/ego_planner_driver_node.h"

#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace ego_planner_driver
{
namespace
{
// 邻机轨迹线性插值采样（同刻采样，node 内复用于碰撞检测）。
Eigen::Vector3d sampleNeighborForCollision(const NeighborTrajectory& nb, double t_abs)
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

// 轨迹 vs 邻机全时域同刻最小距离（候选验证 1.3m 门槛）。
double minClearanceWith(const TimedTrajectory& traj,
                        const std::vector<NeighborTrajectory>& neighbors)
{
  double mc = std::numeric_limits<double>::infinity();
  if (traj.pts.empty() || traj.pts.size() != traj.t.size())
    return mc;
  for (size_t i = 0; i < traj.t.size(); ++i)
    for (const auto& nb : neighbors)
    {
      if (nb.clearance <= 0.0 || nb.pts.size() < 2 || nb.t.size() != nb.pts.size())
        continue;
      const double t_abs = traj.t[i] - nb.start_time;
      if (t_abs < nb.t.front() || t_abs > nb.t.back())
        continue;
      mc = std::min(mc, (traj.pts[i] - sampleNeighborForCollision(nb, t_abs)).norm());
    }
  return mc;
}

// 点 p 到线段 a-b 的最小距离。
double pointSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                            const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < 1e-12)
    return (p - a).norm();
  const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / len2));
  return (p - (a + ab * t)).norm();
}
}  // namespace

EgoPlannerDriverNode::EgoPlannerDriverNode()
    : nh_(), pnh_("~")
{
  EgoPlannerConfig config;
  config.ts = pnh_.param("ts", 0.4);
  config.max_vel = pnh_.param("max_vel", 1.0);
  config.max_acc = pnh_.param("max_acc", 6.0);
  config.swarm_clearance = pnh_.param("swarm_clearance", 0.5);
  config.planning_range = pnh_.param("planning_range", 30.0);
  config.lambda_smooth = pnh_.param("lambda_smooth", 1.0);
  config.lambda_swarm = pnh_.param("lambda_swarm", 1.0);
  config.enable_rebound = pnh_.param("enable_rebound", false);
  config.enable_time_reparam = pnh_.param("enable_time_reparam", false);
  config.cruise_velocity = pnh_.param("cruise_velocity", 1.0);
  config.acceleration_limit = pnh_.param("acceleration_limit", 2.0);
  config.lambda_feasibility = pnh_.param("lambda_feasibility", 1.0);
  config.max_iterations = pnh_.param("max_iterations", 200);
  config.mem_size = pnh_.param("mem_size", 16);
  config.g_epsilon = pnh_.param("g_epsilon", 0.01);
  // 资源安全边界（implementation_plan_26082722 §11）：与 launch 同步的五项
  // 参数。fail-fast：count limit > 0 且不超过编译期绝对 hard limit，
  // segment_direction_epsilon_m finite 且 > 0；危险配置拒绝启动不静默 clamp。
  config.max_arc_samples = static_cast<std::size_t>(
      pnh_.param("max_arc_samples", static_cast<int>(config.max_arc_samples)));
  config.max_parameterization_points = static_cast<std::size_t>(
      pnh_.param("max_parameterization_points",
                 static_cast<int>(config.max_parameterization_points)));
  config.max_trajectory_samples = static_cast<std::size_t>(
      pnh_.param("max_trajectory_samples",
                 static_cast<int>(config.max_trajectory_samples)));
  config.max_neighbor_intent_samples = static_cast<std::size_t>(
      pnh_.param("max_neighbor_intent_samples",
                 static_cast<int>(config.max_neighbor_intent_samples)));
  config.segment_direction_epsilon_m = pnh_.param(
      "segment_direction_epsilon_m", config.segment_direction_epsilon_m);
  max_arc_samples_ = config.max_arc_samples;
  max_parameterization_points_ = config.max_parameterization_points;
  max_trajectory_samples_ = config.max_trajectory_samples;
  max_neighbor_intent_samples_ = config.max_neighbor_intent_samples;
  segment_direction_epsilon_m_ = config.segment_direction_epsilon_m;
  if (!(config.segment_direction_epsilon_m > 0.0) ||
      !std::isfinite(config.segment_direction_epsilon_m))
  {
    ROS_FATAL("[%s] invalid segment_direction_epsilon_m=%f; refusing to start",
              uav_id_.c_str(), config.segment_direction_epsilon_m);
    ros::shutdown();
    return;
  }
  if (config.max_arc_samples == 0 ||
      config.max_arc_samples > kMaxArcSamplesAbsHard ||
      config.max_parameterization_points == 0 ||
      config.max_parameterization_points > kMaxParameterizationPointsAbsHard ||
      config.max_trajectory_samples == 0 ||
      config.max_trajectory_samples > kMaxTrajectorySamplesAbsHard ||
      config.max_neighbor_intent_samples == 0 ||
      config.max_neighbor_intent_samples > kMaxNeighborIntentSamplesAbsHard)
  {
    ROS_FATAL("[%s] resource limits outside [1, absolute hard]: "
              "max_arc=%zu max_pts=%zu max_traj=%zu max_intent=%zu; refusing to start",
              uav_id_.c_str(), config.max_arc_samples,
              config.max_parameterization_points, config.max_trajectory_samples,
              config.max_neighbor_intent_samples);
    ros::shutdown();
    return;
  }
  // 规划层避碰门槛回固定 clearance（implementation_plan_26082602）：不再把
  // SafetyPredictor 动态距离参数同步给规划核心；监督层参数读入 safety_config_
  // （下方），规划层只消费 swarm_clearance×2 固定 gate。
  bias_valid_s_ = pnh_.param("bias_valid_s", 120.0);
  collision_check_rate_hz_ = pnh_.param("collision_check_rate_hz", 10.0);
  core_.reset(new EgoPlannerCore(config));
  replan_period_s_ = pnh_.param("replan_period_s", 1.0);
  no_replan_thresh_m_ = pnh_.param("no_replan_thresh_m", 1.0);
  reach_thresh_m_ = pnh_.param("reach_thresh_m", 0.5);
  max_advance_dist_m_ = pnh_.param("max_advance_dist_m", 3.5);
  arrival_reach_thresh_m_ = pnh_.param("arrival_reach_thresh_m", 0.5);
  activation_replan_attempts_ = std::max(
      1, pnh_.param("activation_replan_attempts", 3));
  setpoint_topic_ = pnh_.param("setpoint_out_topic", std::string("setpoint/ego"));
  frame_id_ = pnh_.param("frame_id", std::string("map"));
  // Identity parameters live in the public namespace (/UAV1/*) so the
  // Python driver and topology can discover them; planner tuning stays
  // private.
  uav_id_ = nh_.param("uav_id", std::string("UAV1"));
  exec_target_ = nh_.param("exec_target", std::string("UAV1"));
  setpoint_rate_ = pnh_.param("setpoint_rate_hz", kSetpointRateHz);
  pose_timeout_ = pnh_.param("local_pose_timeout_s", 1.0);
  stable_s_ = pnh_.param("arrival_stable_s", 1.0);
  timeout_margin_ = pnh_.param("exec_timeout_margin_s", 5.0);
  takeoff_height_ = pnh_.param("takeoff_height_m", 5.0);
  takeoff_ascend_speed_ = pnh_.param("takeoff_ascend_speed", 1.0);

  safety_supervisor_mode_ = pnh_.param(
      "safety_supervisor_mode", std::string("active"));
  safety_supervisor_rate_hz_ = pnh_.param("safety_supervisor_rate_hz", 25.0);
  safety_config_.protected_distance_m = pnh_.param("protected_distance_m", 1.0);
  safety_config_.prediction_horizon_s = pnh_.param("safety_prediction_horizon_s", 5.0);
  safety_config_.sample_period_s = pnh_.param("safety_sample_period_s", 0.05);
  safety_config_.control_delay_s = pnh_.param("safety_control_delay_s", 0.20);
  safety_config_.local_compute_budget_s = pnh_.param("safety_compute_budget_s", 0.05);
  safety_config_.peer_data_age_cap_s = pnh_.param("peer_data_age_cap_s", 2.0);
  safety_config_.emergency_decel_mps2 = pnh_.param("emergency_decel_mps2", 2.0);
  safety_config_.warning_buffer_m = pnh_.param("warning_buffer_m", 0.35);
  safety_config_.emergency_buffer_m = pnh_.param("emergency_buffer_m", 0.10);
  safety_config_.warning_ttc_s = pnh_.param("warning_ttc_s", 3.0);
  safety_config_.emergency_ttc_s = pnh_.param("emergency_ttc_s", 1.5);
  safety_config_.minimum_prediction_window_s = pnh_.param(
      "minimum_prediction_window_s", 0.25);
  episode_release_hysteresis_s_ = pnh_.param(
      "safety_episode_release_s", 1.0);
  emergency_replan_time_s_ = pnh_.param("emergency_replan_time_s", 1.0);
  first_plan_wait_timeout_s_ = pnh_.param("first_plan_wait_timeout_s", 10.0);
  // 机间让路候选（SLOW/LEFT/RIGHT + right-of-way，26082621 迭代）。
  enable_yield_candidates_ = pnh_.param("enable_yield_candidates", false);
  yield_config_.clearance_factor = pnh_.param("yield_clearance_factor", 1.2);
  yield_config_.lateral_max_m = pnh_.param("yield_lateral_max_m", 2.2);
  yield_config_.slow_min_scale = pnh_.param("yield_slow_min_scale", 0.3);
  // 最大速度限制（26082621 迭代）：候选/加速档允许的轨迹速度上限，独立于制动
  // （emergency_max_velocity_mps 1.5 保持）。SLOW 1.2 加速档 + PX4 执行超调实测
  // 1.49-1.54，原 1.5 上限过紧；上限 2.5 与测试判定一致（写在本驱动，不靠验证脚本）。
  yield_config_.max_velocity_mps = pnh_.param("yield_max_velocity_mps", 2.5);
  yield_config_.max_acceleration_mps2 = config.max_acc;
  yield_generator_.reset(new YieldCandidateGenerator(yield_config_));
  safety_predictor_.reset(new SafetyPredictor(safety_config_));
  EmergencyBrakeConfig brake_config;
  brake_config.max_velocity_mps = pnh_.param("emergency_max_velocity_mps", 1.5);
  brake_config.max_acceleration_mps2 = config.max_acc;
  brake_config.max_jerk_mps3 = pnh_.param("emergency_jerk_mps3", 8.0);
  brake_config.nominal_deceleration_mps2 = safety_config_.emergency_decel_mps2;
  brake_config.sample_period_s = pnh_.param("emergency_sample_period_s", 0.02);
  emergency_brake_generator_.reset(new EmergencyBrakeGenerator(brake_config));
  if (safety_supervisor_mode_ != "off" && safety_supervisor_mode_ != "shadow" &&
      safety_supervisor_mode_ != "active")
  {
    ROS_ERROR("[%s] invalid safety_supervisor_mode=%s; disabling supervisor",
              uav_id_.c_str(), safety_supervisor_mode_.c_str());
    safety_supervisor_mode_ = "off";
  }
  if (safety_supervisor_mode_ != "off")
  {
    std::string identity = exec_target_;
    if (identity.empty())
    {
      char hostname[256] = {};
      if (::gethostname(hostname, sizeof(hostname) - 1) == 0) identity = hostname;
    }
    const std::string log_dir = pnh_.param(
        "diagnostic_log_dir",
        std::string("/home/ub20tg/catkin_swarm6-2/runtime_logs/ego_planner"));
    const int queue_capacity = std::max(1, pnh_.param("diagnostic_log_queue_size", 2048));
    if (!safety_logger_.start(log_dir, identity, static_cast<size_t>(queue_capacity)))
      ROS_WARN("[%s] safety diagnostic file unavailable; ROS logging remains active",
               uav_id_.c_str());
  }

  // Interface topics live under the public namespace (/UAV1/*) so the
  // Python EgoSwarmDriver and topology bridge can reach them without
  // knowing the private roslaunch node name. Parameters stay private.
  setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(setpoint_topic_, 10);
  intent_pub_ = nh_.advertise<swarm_uav_interfaces::UavTrajectoryIntent>("trajectory_intent", 10);
  state_pub_ = nh_.advertise<std_msgs::String>("exec_state", 10, true);
  goal_sub_ = nh_.subscribe("goal", 1, &EgoPlannerDriverNode::onGoal, this);
  goal_yaw_sub_ = nh_.subscribe("goal_yaw", 1, &EgoPlannerDriverNode::onGoalYaw, this);
  waypoints_sub_ = nh_.subscribe("waypoints", 1, &EgoPlannerDriverNode::onWaypoints, this);
  pose_sub_ = nh_.subscribe("local_pose", 1, &EgoPlannerDriverNode::onLocalPose, this);
  intent_sub_ = nh_.subscribe("neighbor_intent", 10, &EgoPlannerDriverNode::onNeighborIntent, this);
  own_bias_sub_ = nh_.subscribe("gps_bias", 1, &EgoPlannerDriverNode::onOwnBias, this);
  hold_sub_ = nh_.subscribe("hold", 1, &EgoPlannerDriverNode::onHold, this);
  mavros_state_sub_ = nh_.subscribe("/mavros/state", 1, &EgoPlannerDriverNode::onMavrosState, this);
  direct_control_sub_ = nh_.subscribe("direct_control_active", 1,
                                      &EgoPlannerDriverNode::onDirectControl, this);
  setState(kStateHold);
  timer_ = nh_.createTimer(ros::Duration(1.0 / setpoint_rate_),
                           &EgoPlannerDriverNode::publishSetpoint, this);
  if (safety_supervisor_mode_ != "off" && safety_supervisor_rate_hz_ > 0.0)
    safety_timer_ = nh_.createTimer(ros::Duration(1.0 / safety_supervisor_rate_hz_),
        &EgoPlannerDriverNode::runSafetySupervisor, this);
  if (collision_check_rate_hz_ > 0.0)
    collision_check_timer_ = nh_.createTimer(
        ros::Duration(1.0 / collision_check_rate_hz_),
        &EgoPlannerDriverNode::collisionCheckTick, this);
  replan_thread_ = std::thread(&EgoPlannerDriverNode::replanLoop, this);
}

EgoPlannerDriverNode::~EgoPlannerDriverNode()
{
  stop_ = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    replan_cv_.notify_all();
  }
  if (replan_thread_.joinable())
    replan_thread_.join();
  safety_logger_.stop();
}

void EgoPlannerDriverNode::setClockSource(ClockFn fn)
{
  std::lock_guard<std::mutex> lock(mutex_);
  clock_source_ = std::move(fn);
}

void EgoPlannerDriverNode::injectLocalPose(const geometry_msgs::PoseStamped& pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  local_pose_ = pose;
  local_pose_stamp_ = pose.header.stamp;
  if (exec_state_ == kStateHold && !steady_target_ && !direct_control_active_)
    captureHoldTargetLocked(nowFunc());
}

bool EgoPlannerDriverNode::injectNeighborIntent(
    const swarm_uav_interfaces::UavTrajectoryIntent& intent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return acceptNeighborIntentLocked(intent, nowFunc());
}

void EgoPlannerDriverNode::injectOwnBias(double bias)
{
  std::lock_guard<std::mutex> lock(mutex_);
  own_bias_s_ = bias;
  own_bias_received_ = nowFunc();
}

void EgoPlannerDriverNode::injectPeerBias(const std::string& uav_id, double bias)
{
  std::lock_guard<std::mutex> lock(mutex_);
  peer_bias_[uav_id] = std::make_pair(bias, nowFunc());
}

double EgoPlannerDriverNode::neighborStartTimeForTest(
    const std::string& uav_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = inbox_.find(uav_id);
  return it == inbox_.end() ? -1.0 : it->second.first.start_time;
}

void EgoPlannerDriverNode::injectMavrosState(bool armed, bool offboard)
{
  std::lock_guard<std::mutex> lock(mutex_);
  armed_ = armed;
  offboard_ = offboard;
  handleMavrosStateEdgeLocked();
}

void EgoPlannerDriverNode::sendGoal(double x, double y, double z)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = nowFunc();
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;
  onGoal(boost::make_shared<geometry_msgs::PointStamped>(msg));
}

void EgoPlannerDriverNode::sendHold()
{
  onHold(boost::make_shared<std_msgs::Empty>());
}

void EgoPlannerDriverNode::sendWaypoints(
    const std::vector<geometry_msgs::Point32>& points)
{
  geometry_msgs::PolygonStamped msg;
  msg.header.stamp = nowFunc();
  msg.polygon.points = points;
  onWaypoints(boost::make_shared<geometry_msgs::PolygonStamped>(msg));
}

bool EgoPlannerDriverNode::replanOnce(bool timed_trigger)
{
  boost::optional<ReplanSnapshot> snap;
  ros::Time completed_at;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hold_requested_)
    {
      processHoldLocked();
      return true;
    }
    if (direct_control_active_)
      return false;
    if (exec_state_ == kStateTakeoff)
      return false;
    // 单一 replan 入口（方案 A）：planning_requested_（1s 定时/10Hz 冲突/推进）
    // 或 emergency_pending_（监督 EMERGENCY 事件）任一置位即执行一轮。
    // timed_trigger=true：replanLoop 1s 定时超时（软着陆失败重试的固定节奏）。
    // steady_target_ 拦截例外：软着陆失败中（plan_fail_since_ 非零）即使 HOLD 捕获
    // 已锁存 steady_target_ 也放行定时重试（否则 HOLD 状态首轮失败无法恢复/无法
    // 在 10s 超时后收口）；成功提交后 plan_fail_since_ 清空恢复拦截语义。
    if ((!timed_trigger && !planning_requested_ && !emergency_pending_) ||
        !pending_goal_ ||
        // §3.1：退化等待中（arrival_pending_）steady_target_ 可能被 HOLD 捕获
        // 锁存，必须放行定时补判，否则同点目标永远停在 HOLD。
        (steady_target_ && plan_fail_since_.isZero() && !arrival_pending_))
      return false;
    planning_requested_ = false;
    const ros::Time now = nowFunc();
    if (!local_pose_ || !local_pose_stamp_ ||
        (now - *local_pose_stamp_).toSec() > pose_timeout_)
      return false;  // pose 过期跳过本轮（无 POSE_STALE；pose 恢复后周期触发重试）
    // §3.1（260828 修复）：snapshot 前先用 fresh odometry 处理 target 推进与
    // 最终到达（覆盖无 current_ 的首轮规划/退化等待场景）。真实到达最终关键点
    // → COMPLETED 收口；稳定窗口未满则等待 1s 定时补判。
    if (handleArrivalBeforeSnapshotLocked(now))
      return true;
    ReplanSnapshot data;
    data.now = now;
    data.safety_generation = safety_generation_;
    data.start.yaw = goal_yaw_;
    // 滚动重规划：从旧轨迹同一时刻取完整 P/V/A/yaw 作为新段起点（原版 FSM
    // 语义），避免"实际位置 + 旧导数"组合在切段时产生位置跳变与速度纹波。
    // 首次规划/新任务（current_ 被 resetRun 清空）才用实际位置 + 零导数。
    if (!sampleCurrentState(data.start.position, data.start_vel, data.start_acc,
                            data.start.yaw))
    {
      data.start.position = Eigen::Vector3d(local_pose_->pose.position.x,
                                            local_pose_->pose.position.y,
                                            local_pose_->pose.position.z);
      data.start.yaw = goal_yaw_;
      data.start_vel = Eigen::Vector3d::Zero();
      data.start_acc = Eigen::Vector3d::Zero();
    }
    clampStartDerivatives(data.start_vel, data.start_acc);
    // 终点 = 当前任务航点（相邻下一个 keypoints，plan-exec 分支 A/B）：
    // waypoint_consumed_ 即目标索引（onWaypoints 初始化为跳过 start 关键点）。
    // 单点退化（onGoal/单点航点）时目标 = waypoints_[0]。
    data.waypoints.clear();
    if (!waypoints_.empty())
    {
      const size_t idx = std::min(waypoint_consumed_, waypoints_.size() - 1);
      data.waypoints.push_back(waypoints_[idx]);
      data.final_segment = (idx == waypoints_.size() - 1);
    }
    else
    {
      data.final_segment = false;
    }
    data.neighbors = freshAbsoluteNeighborsLocked(now);
    snap = ReplanSnapshot(std::move(data));
  }
  if (!snap)
    return false;
  TimedTrajectory traj;
  double cost = 0.0;
  bool safe_candidate = false;
  ros::Time assumed_activation = snap->now;
  std::vector<NeighborTrajectory> activation_neighbors;
  for (int attempt = 0; attempt < activation_replan_attempts_; ++attempt)
  {
    std::vector<NeighborTrajectory> absolute_neighbors;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      absolute_neighbors = freshAbsoluteNeighborsLocked(nowFunc());
    }
    const std::vector<NeighborTrajectory> planning_neighbors =
        rebaseNeighbors(absolute_neighbors, assumed_activation);
    const PlanResult plan_result = core_->planOnce(
        snap->start, snap->waypoints, planning_neighbors, traj, cost,
        snap->start_vel, snap->start_acc, snap->final_segment);
    if (plan_result == PlanResult::RESOURCE_LIMIT)
    {
      // §9.6：资源失败同一事务内不 activation retry、不 yield，立即安全收口
      // 为 EGO_PLAN_FAILED（executor → HOLD → backend ALL HOLD）。
      std::lock_guard<std::mutex> lock(mutex_);
      captureHoldTargetLocked(nowFunc());
      setState(kStatePlanFailed);
      plan_fail_since_ = ros::Time();
      ROS_ERROR("[%s] resource limit during replan; closing as EGO_PLAN_FAILED",
                uav_id_.c_str());
      return true;
    }
    if (plan_result == PlanResult::DEGENERATE_SEGMENT)
    {
      // §5.2/§9.4 首次退化（无失败计时）：planned start 到当前 target 方向退化。
      // 不提交新轨迹、不清失败计时、不改状态；保留现有 current_ 及末端 P/V/A，
      // 由 snapshot 前的 fresh odometry（handleArrivalBeforeSnapshotLocked）推进/
      // 到达。返回 false 且不启动 plan_fail_since_。
      // §3.1（260828 修复）：已有 plan_fail_since_ 后持续退化必须走普通可重试
      // 失败路径（soft-landing 计时），否则失败计时既不清除也不收口，任务无法
      // 按 plan_fail_timeout_s_ 终止。
      bool failing = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        failing = !plan_fail_since_.isZero();
      }
      if (!failing)
      {
        ROS_DEBUG_THROTTLE(1.0,
                           "[%s] degenerate segment; keeping current trajectory",
                           uav_id_.c_str());
        return false;
      }
      ROS_WARN_THROTTLE(1.0,
                        "[%s] degenerate segment while replan failing; "
                        "falling through to soft-landing closure",
                        uav_id_.c_str());
      break;  // 进入 safe_candidate=false 软着陆失败路径（10s 收口）
    }
    if (plan_result == PlanResult::RETRYABLE_FAILURE)
    {
      assumed_activation = nowFunc();
      continue;
    }
    completed_at = nowFunc();
    activation_neighbors = rebaseNeighbors(absolute_neighbors, completed_at);
    if (core_->checkClearance(traj, activation_neighbors,
                              core_->config().swarm_clearance * 2.0))
    {
      safe_candidate = true;
      break;
    }
    assumed_activation = completed_at;
  }
  if (!safe_candidate)
  {
    // 方案 Y：EMERGENCY 尝试路径失败不收口，由 Y 方案判定制动/PLAN_FAILED
    // （跳过候选——ttc 紧时直接判制动，不做 SLOW/LEFT/RIGHT 浪费时间）。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (emergency_pending_)
      {
        if (tryEmergencyBrakeLocked())
          return true;
        captureHoldTargetLocked(nowFunc());
        setState(kStatePlanFailed);
        return false;
      }
    }
    // 常规 replan 失败 → 候选（SLOW/LEFT/RIGHT + right-of-way，replan 失败分支）。
    if (snap)
    {
      const YieldResult yield_result = tryYieldReplan(*snap);
      if (yield_result == YieldResult::COMMITTED)
        return true;
      if (yield_result == YieldResult::RESOURCE_LIMIT)
      {
        // §9.6：候选生成内部资源失败 → 同一事务内立即收口，不进入软着陆。
        std::lock_guard<std::mutex> lock(mutex_);
        captureHoldTargetLocked(nowFunc());
        setState(kStatePlanFailed);
        plan_fail_since_ = ros::Time();
        ROS_ERROR("[%s] resource limit during yield replan; closing as EGO_PLAN_FAILED",
                  uav_id_.c_str());
        return true;
      }
      // NOT_AVAILABLE：继续软着陆。
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // 软着陆（26082700）：常规 replan 全失败不立即 PLAN_FAILED。保持当前轨迹
    // （继续执行到末端自然悬停），由 replanLoop 1s 定时/冲突事件继续重试；
    // 连续失败超过 plan_fail_timeout_s_（10s）才收口 EGO_PLAN_FAILED（executor
    // 收口后经 backend 对全组活动机发 HOLD）。EMERGENCY 分支保持立即收口
    // （安全关键路径不变）。
    const ros::Time now = nowFunc();
    if (plan_fail_since_.isZero())
      plan_fail_since_ = now;
    if (replan_failure_fatal_ &&
        (now - plan_fail_since_).toSec() >= plan_fail_timeout_s_)
    {
      captureHoldTargetLocked(now);
      setState(kStatePlanFailed);
      plan_fail_since_ = ros::Time();
      ROS_WARN("[%s] replan failed for %.1fs, closing as EGO_PLAN_FAILED",
               uav_id_.c_str(), plan_fail_timeout_s_);
    }
    else
    {
      ROS_WARN_THROTTLE(1.0,
                        "[%s] replan failed, soft-landing (retry on 1s timer/event, "
                        "%.1fs until EGO_PLAN_FAILED)",
                        uav_id_.c_str(),
                        plan_fail_since_.isZero()
                            ? plan_fail_timeout_s_
                            : (plan_fail_timeout_s_ -
                               (now - plan_fail_since_).toSec()));
    }
    return false;
  }
  // B（26082621 迭代）：常规规划"成功"但裕度不足（min_clear < clearance_factor×1m）
  // → 也生成候选替换成更安全的轨迹（防止"规划通过但实际间距偏小"——SITL Run5 类
  // min_3d 0.605m）。候选失败则保持原轨迹（仍有 1m 硬门裕度），不 PLAN_FAILED。
  if (snap)
  {
    const double gate =
        yield_config_.clearance_factor * core_->config().swarm_clearance * 2.0;
    if (minClearanceWith(traj, activation_neighbors) < gate)
    {
      const YieldResult yield_result = tryYieldReplan(*snap);
      if (yield_result == YieldResult::COMMITTED)
        return true;
      if (yield_result == YieldResult::RESOURCE_LIMIT)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        captureHoldTargetLocked(nowFunc());
        setState(kStatePlanFailed);
        plan_fail_since_ = ros::Time();
        ROS_ERROR("[%s] resource limit during margin yield replan; closing as EGO_PLAN_FAILED",
                  uav_id_.c_str());
        return true;
      }
      // 候选失败保持原轨迹（仍有 1m 硬门裕度），不 PLAN_FAILED。
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hold_requested_)
      return true;
    if (emergency_braking_ || snap->safety_generation != safety_generation_)
      return false;
    const std::vector<NeighborTrajectory> commit_neighbors = rebaseNeighbors(
        freshAbsoluteNeighborsLocked(completed_at), completed_at);
    if (!core_->checkClearance(traj, commit_neighbors,
                               core_->config().swarm_clearance * 2.0))
    {
      // 提交时最新邻机轨迹使本段不安全：EMERGENCY 优先制动，否则 PLAN_FAILED。
      if (emergency_pending_ && tryEmergencyBrakeLocked())
        return true;
      captureHoldTargetLocked(completed_at);
      setState(kStatePlanFailed);
      return false;
    }
    // The freshly generated trajectory owns a relative time axis beginning at
    // zero. Planning latency is represented exactly once by this activation
    // epoch; it must never be added to relative samples or duration.
    current_.reset(new TimedTrajectory(traj));
    plan_start_ = completed_at;
    arrival_since_.reset();
    steady_target_.reset();
    plan_fail_since_ = ros::Time();  // 常规 replan 提交 = 软着陆恢复（退出失败计时）
    // 常规 replan 成功 → 清除 EMERGENCY 事件（监督层对提交后新轨迹重新评估）。
    emergency_pending_ = false;
    emergency_peer_id_.clear();
    setState(kStateExecuting);
    publishIntent(traj, completed_at);
  }
  return true;
}

// plan-exec 分支 A：推进。当前目标（waypoints_[waypoint_consumed_]）已到达
// （距当前位置 < reach_thresh_m_ 0.5m）→ 切换到相邻下一个 keypoint 为目标。
// 距当前位置 <0.5m 的过近关键点被丢弃（目标距当前位置 <0.5m 时丢弃推进），
// 推进后的目标最远使用 max_advance_dist_m_（3.5m）远的 keypoint。
bool EgoPlannerDriverNode::advanceTargetLocked(const Eigen::Vector3d& pos)
{
  if (waypoints_.size() < 2 || waypoint_consumed_ + 1 >= waypoints_.size())
    return false;  // 无下一个关键点（单点目标/最终关键点不推进）
  bool advanced = false;
  while (waypoint_consumed_ + 1 < waypoints_.size())
  {
    const double d_cur = (pos - waypoints_[waypoint_consumed_].position).norm();
    if (d_cur >= reach_thresh_m_)
      break;  // 当前目标未到达（≥0.5m）
    const double d_next =
        (pos - waypoints_[waypoint_consumed_ + 1].position).norm();
    if (d_next > max_advance_dist_m_ && !advanced)
      break;  // 最远使用 3.5m 远的 keypoint：不再向前跳
    ++waypoint_consumed_;
    advanced = true;
    if (d_next >= reach_thresh_m_)
      break;  // 新目标有效（≥0.5m）
    // d_next < 0.5m：丢弃该推进，继续向后找（多个关键点挤在 0.5m 内时）。
  }
  return advanced;
}

// §3.1（260828 修复）：replan snapshot 前用 fresh odometry 处理 target 推进与
// 最终到达。覆盖两处原有缺陷：
//   1. 无 current_（首轮/退化等待）时 checkArrival 直接退出 → 同点目标持续 HOLD；
//   2. replanOnce 只检查 pose freshness 不推进 → 真实位置已过目标仍规划旧目标。
// 返回 true = 本轮已收口（无 current_ 且真实到达最终关键点 → COMPLETED），
// 不需要构建 snapshot；返回 false = 继续正常 snapshot 规划（推进后目标已更新，
// 或当前处于 EXECUTING 由每 tick checkArrival 处理到达/推进）。
bool EgoPlannerDriverNode::handleArrivalBeforeSnapshotLocked(const ros::Time& now)
{
  // §3.1：steady_target_ 非空时若 arrival_pending_ 未置（正常 HOLD/空闲捕获）
  // 返回 false；若 arrival_pending_ 已置（退化等待中，steady_target_ 可能是
  // 等待期间被 pose 捕获锁存）必须忽略并继续判定到达，否则同点目标永远停在
  // HOLD。
  if (current_ || emergency_braking_ || hold_requested_ ||
      (steady_target_ && !arrival_pending_))
    return false;  // 有轨迹时分支 A/B 由 publishSetpoint tick 的 checkArrival 处理
  if (!local_pose_ || !local_pose_stamp_ ||
      (now - *local_pose_stamp_).toSec() > pose_timeout_)
  {
    arrival_since_.reset();
    return false;
  }
  const Eigen::Vector3d cur(local_pose_->pose.position.x,
                            local_pose_->pose.position.y,
                            local_pose_->pose.position.z);
  // 分支 A：真实位置已越过当前目标 → 推进到相邻下一个 keypoint（返回 false，
  // 让本轮 snapshot 用推进后的新目标规划；推进目标最远 3.5m 规则不变）。
  if (advanceTargetLocked(cur))
  {
    planning_requested_ = true;
    replan_cv_.notify_all();
    return false;
  }
  // 分支 B：无 current_ 且真实位置已在最终关键点附近 → 稳定窗口后 COMPLETED。
  if (!waypoints_.empty() &&
      (cur - waypoints_.back().position).norm() <= arrival_reach_thresh_m_)
  {
    if (!arrival_since_)
      arrival_since_ = now;
    if ((now - *arrival_since_).toSec() >= stable_s_)
    {
      steady_target_ = HoldTarget{waypoints_.back().position, goal_yaw_};
      current_.reset();
      arrival_since_.reset();
      arrival_pending_ = false;
      plan_start_ = ros::Time(0);
      emergency_pending_ = false;
      setState(kStateCompleted);
      ROS_INFO("[%s] real arrival at final keypoint (no current_) -> COMPLETED",
               uav_id_.c_str());
      return true;
    }
    arrival_pending_ = true;  // 稳定窗口未满：replanLoop 1s 定时补判，不重复规划
    ROS_DEBUG_THROTTLE(1.0, "[%s] at final keypoint, waiting %.1fs stable",
                       uav_id_.c_str(),
                       stable_s_ - (now - *arrival_since_).toSec());
    return true;  // 已由 fresh odometry 收口判定，本轮不构建 snapshot
  }
  arrival_since_.reset();
  arrival_pending_ = false;
  return false;
}

// 方案 Y：EMERGENCY 且 ttc 紧急（< emergency_replan_time_s_）时生成并提交受约束
// 制动轨迹（EMERGENCY_BRAKE → 轨迹耗尽 → BRAKE_HOLD 锁存）。持 mutex_。
// 返回 true = 已提交制动/fallback；false = 不制动（ttc 充足 → 调用方 PLAN_FAILED）。
bool EgoPlannerDriverNode::tryEmergencyBrakeLocked()
{
  if (!emergency_pending_)
    return false;
  emergency_pending_ = false;
  const std::string peer = emergency_peer_id_;
  emergency_peer_id_.clear();
  if (!(emergency_peer_ttc_ < emergency_replan_time_s_))
    return false;  // ttc 充足：由调用方收口（PLAN_FAILED / 等待）

  TrajectorySample start;
  if (current_ &&
      current_->samplePVA(std::max(0.0, (nowFunc() - plan_start_).toSec()), start))
  {
    TimedTrajectory braking;
    if (emergency_brake_generator_ &&
        emergency_brake_generator_->generate(start, braking))
    {
      ++safety_generation_;
      current_.reset(new TimedTrajectory(braking));
      plan_start_ = nowFunc();
      arrival_since_.reset();
      steady_target_.reset();
      emergency_braking_ = true;
      plan_fail_since_ = ros::Time();  // 制动提交 = 软着陆结束
      setState(kStateEmergencyBrake);
      publishIntent(braking, plan_start_,
                    swarm_uav_interfaces::UavTrajectoryIntent::PHASE_BRAKING);
      ROS_ERROR("[%s] emergency brake committed for peer=%s (scheme Y)",
                uav_id_.c_str(), peer.empty() ? "unknown" : peer.c_str());
      return true;
    }
    // 生成器失败：立即中止 nominal，在当前样本位置锁存 fallback BRAKE_HOLD。
    ++safety_generation_;
    current_.reset();
    plan_start_ = ros::Time(0);
    emergency_braking_ = false;
    plan_fail_since_ = ros::Time();  // fallback BRAKE_HOLD = 软着陆结束
    steady_target_ = HoldTarget{start.pos, start.yaw};
    setState(kStateBrakeHold);
    ROS_ERROR("[%s] emergency brake generation failed; nominal aborted to "
              "fallback BRAKE_HOLD", uav_id_.c_str());
    return true;
  }
  return false;  // 当前轨迹不可采样：调用方 PLAN_FAILED 收口。
}

bool EgoPlannerDriverNode::sampleCurrentState(Eigen::Vector3d& pos,
                                              Eigen::Vector3d& vel,
                                              Eigen::Vector3d& acc,
                                              double& yaw) const
{
  if (!current_ || current_->pts.size() < 3)
    return false;
  const double elapsed = (nowFunc() - plan_start_).toSec();
  const double t_end = current_->t.empty() ? 0.0 : current_->t.back();
  // 段间衔接：periodic replan 可能在当前段执行完（elapsed>duration，UAV 在
  // 段末悬停或滑行）后才快照。此时若返回全零 vel，新段从 0 起步重新加速，
  // 速度 0~max 震荡。clamp 到轨迹末端采样，保持速度/位置连续。
  const double t_clamp = std::min(elapsed, std::max(0.0, t_end - 1e-4));
  TrajectorySample s;
  if (!current_->samplePVA(t_clamp, s))
    return false;
  pos = s.pos;
  vel = s.vel;
  acc = s.acc;
  yaw = s.yaw;
  return true;
}

void EgoPlannerDriverNode::clampStartDerivatives(Eigen::Vector3d& vel,
                                                 Eigen::Vector3d& acc) const
{
  const EgoPlannerConfig cfg = core_->config();
  // Euclidean-norm caps: max_vel/max_acc bound the magnitude, so a per-axis cap
  // (which can grow the magnitude up to sqrt(3)x) is the wrong metric here.
  clampVectorNorm(vel, cfg.max_vel);
  clampVectorNorm(acc, cfg.max_acc);
}

void EgoPlannerDriverNode::advanceHoldHandling()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (hold_requested_)
    processHoldLocked();
}

void EgoPlannerDriverNode::publishSetpointTick()
{
  publishSetpoint(ros::TimerEvent());
}

void EgoPlannerDriverNode::safetySupervisorTick()
{
  runSafetySupervisor(ros::TimerEvent());
}

void EgoPlannerDriverNode::arrivalCheckTick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_ || current_->pts.empty())
    return;
  checkArrival(current_->pts.back(), current_->yaw.back(), nowFunc());
}

void EgoPlannerDriverNode::setSafetySupervisorModeForTest(const std::string& mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  safety_supervisor_mode_ = (mode == "shadow" || mode == "active") ? mode : "off";
}

void EgoPlannerDriverNode::setIdentityForTest(const std::string& uav_id,
                                              const std::string& exec_target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  uav_id_ = uav_id;
  exec_target_ = exec_target;
}

SafetyPrediction EgoPlannerDriverNode::lastSafetyPrediction() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_safety_prediction_;
}

std::string EgoPlannerDriverNode::execState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return exec_state_;
}

boost::optional<HoldTarget> EgoPlannerDriverNode::steadyTarget() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return steady_target_;
}

mavros_msgs::PositionTarget EgoPlannerDriverNode::lastSetpoint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_setpoint_ ? *last_setpoint_ : mavros_msgs::PositionTarget();
}

size_t EgoPlannerDriverNode::setpointCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return setpoint_count_;
}

void EgoPlannerDriverNode::onGoal(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_goal_ = GoalPoint{Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z), goal_yaw_};
  waypoints_.assign(1, WayPoint{pending_goal_->position, goal_yaw_});
  // 单点目标：waypoints_[0] 即目标（无 start 关键点）。
  waypoint_consumed_ = 0;
  resetRun();
  planning_requested_ = true;
  replan_cv_.notify_all();
}

void EgoPlannerDriverNode::onGoalYaw(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goal_yaw_ = msg->data;
  if (pending_goal_)
    pending_goal_->yaw = goal_yaw_;
}

void EgoPlannerDriverNode::onWaypoints(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<WayPoint> wp;
  for (const auto& p : msg->polygon.points)
    wp.push_back(WayPoint{Eigen::Vector3d(p.x, p.y, p.z), goal_yaw_});
  if (wp.empty())
    return;
  pending_goal_ = GoalPoint{wp.back().position, goal_yaw_};
  waypoints_ = wp;
  // plan-exec 相邻目标推进：waypoints_[0] 为 start 关键点（已到达），初始目标
  // = 相邻下一个 keypoint（waypoints_[1]）；单点退化时目标 = waypoints_[0]。
  waypoint_consumed_ = (wp.size() >= 2) ? 1 : 0;
  resetRun();
  planning_requested_ = true;
  replan_cv_.notify_all();
}

void EgoPlannerDriverNode::onLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  local_pose_ = *msg;
  local_pose_stamp_ = msg->header.stamp;
}

void EgoPlannerDriverNode::onNeighborIntent(
    const swarm_uav_interfaces::UavTrajectoryIntent::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  acceptNeighborIntentLocked(*msg, nowFunc());
}

void EgoPlannerDriverNode::onOwnBias(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  own_bias_s_ = msg->data;
  own_bias_received_ = nowFunc();
}

void EgoPlannerDriverNode::onNeighborBias(const std_msgs::Float64::ConstPtr& msg,
                                          const std::string& uav_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  peer_bias_[uav_id] = std::make_pair(msg->data, nowFunc());
}

void EgoPlannerDriverNode::ensureNeighborBiasSubscribed(const std::string& uav_id)
{
  if (neighbor_bias_subs_.count(uav_id))
    return;
  // 本机 master 上邻机 gps_bias 由 topology bridge 转发为 /UAVn/gps_bias。
  neighbor_bias_subs_[uav_id] = nh_.subscribe<std_msgs::Float64>(
      "/" + uav_id + "/gps_bias", 1,
      boost::bind(&EgoPlannerDriverNode::onNeighborBias, this, _1, uav_id));
}

double EgoPlannerDriverNode::timeCorrectionFor(const std::string& uav_id) const
{
  const ros::Time now = nowFunc();
  if (own_bias_received_.isZero() ||
      (now - own_bias_received_).toSec() > bias_valid_s_)
    return 0.0;
  const auto it = peer_bias_.find(uav_id);
  if (it == peer_bias_.end() ||
      (now - it->second.second).toSec() > bias_valid_s_)
    return 0.0;
  return own_bias_s_ - it->second.first;
}

bool EgoPlannerDriverNode::acceptNeighborIntentLocked(
    const swarm_uav_interfaces::UavTrajectoryIntent& msg, const ros::Time& now)
{
  const bool legacy = msg.protocol_version == "1.0";
  const bool current_protocol = msg.protocol_version == "2.0";
  const bool valid_phase = legacy ||
      msg.phase == swarm_uav_interfaces::UavTrajectoryIntent::PHASE_TENTATIVE ||
      msg.phase == swarm_uav_interfaces::UavTrajectoryIntent::PHASE_ACTIVE ||
      msg.phase == swarm_uav_interfaces::UavTrajectoryIntent::PHASE_BRAKING;
  if ((!legacy && !current_protocol) || !valid_phase || msg.uav_id.empty() ||
      msg.uav_id == uav_id_ || msg.exec_target.empty() ||
      msg.frame_id != frame_id_ || !std::isfinite(msg.stamp) ||
      !std::isfinite(msg.clearance) || msg.clearance <= 0.0 ||
      msg.t.size() != msg.sampled_traj.size() || msg.t.size() < 2 ||
      msg.t.size() > max_neighbor_intent_samples_ ||
      msg.t.size() > kMaxNeighborIntentSamplesAbsHard)
    return false;

  // A trajectory epoch is its activation time. Reject already stale epochs and
  // implausibly future epochs rather than retaining unusable or unsynchronised
  // trajectories in the reciprocal planner.
  // GPS 时钟偏置修正：把邻机意图激活时刻修正到本机时间轴
  //   T_own = T_other - bias_other + bias_own（本机与邻机 bias 均新鲜时）
  // 修正后的 epoch 用于 stale 检查、版本比较、start_time 与 data_age 对齐。
  const double correction = timeCorrectionFor(msg.uav_id);
  const double epoch = msg.stamp + correction;
  ensureNeighborBiasSubscribed(msg.uav_id);
  const double age = now.toSec() - epoch;
  if (age > kNeighborStaleS || age < -kNeighborStaleS)
    return false;

  constexpr double kTimeEpsilon = 1e-9;
  if (!std::isfinite(msg.t.front()) || std::abs(msg.t.front()) > kTimeEpsilon)
    return false;
  for (size_t i = 0; i < msg.t.size(); ++i)
  {
    const geometry_msgs::Point& p = msg.sampled_traj[i].position;
    if (!std::isfinite(msg.t[i]) || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z) ||
        (i > 0 && msg.t[i] <= msg.t[i - 1]))
      return false;
  }

  auto version = latest_neighbor_version_.find(msg.uav_id);
  if (version != latest_neighbor_version_.end())
  {
    const double previous_epoch = version->second.first;
    if (epoch < previous_epoch - kTimeEpsilon ||
        (std::abs(epoch - previous_epoch) <= kTimeEpsilon &&
         msg.traj_id <= version->second.second))
      return false;
  }

  const size_t n = msg.t.size();
  NeighborTrajectory nb;
  nb.start_time = epoch;  // absolute epoch (own time axis); converted at planning snapshot
  nb.clearance = msg.clearance;
  nb.duration = msg.t.back();
  nb.t.reserve(n);
  nb.pts.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    nb.t.push_back(msg.t[i]);
    nb.pts.emplace_back(msg.sampled_traj[i].position.x,
                        msg.sampled_traj[i].position.y,
                        msg.sampled_traj[i].position.z);
  }
  inbox_[msg.uav_id] = std::make_pair(nb, now);
  neighbor_phase_[msg.uav_id] = legacy
      ? swarm_uav_interfaces::UavTrajectoryIntent::PHASE_ACTIVE : msg.phase;
  latest_neighbor_version_[msg.uav_id] = std::make_pair(epoch, msg.traj_id);
  // 对齐原版 SEQUENTIAL_START：记录前驱（drone_id-1）intent 已收到 → 首轮
  // 规划可放行（leader 立即，后机按 drone_id 顺序等待前机轨迹）。
  {
    int my_id = 0, peer_id = 0;
    if (SafetyPredictor::parseNumericId(
            !uav_id_.empty() ? uav_id_ : exec_target_, my_id) &&
        my_id > 1 && SafetyPredictor::parseNumericId(msg.uav_id, peer_id) &&
        peer_id == my_id - 1)
    {
      prev_intent_received_ = true;
      replan_cv_.notify_all();  // 前驱轨迹到达 → 事件唤醒首轮规划（SEQUENTIAL_START）
    }
  }
  // 事件驱动 replan 已删除（implementation_plan_26082602 §2.3/§5）：与 10Hz
  // 周期碰撞检查作用重复，周期检查用最新缓冲轨迹，邻机更新后 ≤100ms 即检测
  // 同一碰撞；新 intent 只更新缓冲，由 collisionCheckTick 统一触发 replan。
  return true;
}

std::vector<NeighborTrajectory> EgoPlannerDriverNode::rebaseNeighbors(
    const std::vector<NeighborTrajectory>& absolute_neighbors,
    const ros::Time& candidate_activation)
{
  std::vector<NeighborTrajectory> result = absolute_neighbors;
  for (auto& neighbor : result)
    neighbor.start_time -= candidate_activation.toSec();
  return result;
}

std::vector<NeighborTrajectory>
EgoPlannerDriverNode::freshAbsoluteNeighborsLocked(const ros::Time& now) const
{
  std::vector<NeighborTrajectory> result;
  result.reserve(inbox_.size());
  for (const auto& item : inbox_)
  {
    if ((now - item.second.second).toSec() <= kNeighborStaleS)
    {
      NeighborTrajectory neighbor = item.second.first;
      neighbor.data_age_s = std::max(0.0, (now - item.second.second).toSec());
      result.push_back(neighbor);
    }
  }
  return result;
}

void EgoPlannerDriverNode::onHold(const std_msgs::Empty::ConstPtr&)
{
  std::lock_guard<std::mutex> lock(mutex_);
  hold_requested_ = true;
  planning_requested_ = false;
  emergency_pending_ = false;
  replan_cv_.notify_all();
}

void EgoPlannerDriverNode::onMavrosState(const mavros_msgs::State::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  armed_ = msg->armed;
  offboard_ = (msg->mode == "OFFBOARD");
  handleMavrosStateEdgeLocked();
}

void EgoPlannerDriverNode::setDirectControlActive(bool active)
{
  std::lock_guard<std::mutex> lock(mutex_);
  direct_control_active_ = active;
  if (direct_control_active_)
  {
    planning_requested_ = false;
    emergency_pending_ = false;
    current_.reset();
    steady_target_.reset();
    arrival_since_.reset();
    setState(kStateHold);
  }
}

void EgoPlannerDriverNode::onDirectControl(const std_msgs::Bool::ConstPtr& msg)
{
  setDirectControlActive(msg->data);
}

void EgoPlannerDriverNode::handleMavrosStateEdgeLocked()
{
  const bool current = armed_ && offboard_;
  if (current && !prev_arm_offboard_)
  {
    prev_arm_offboard_ = true;
    if (exec_state_ == kStateHold || exec_state_ == kStateIdle)
    {
      takeoff_active_ = false;
      ROS_INFO("[%s] arm+OFFBOARD detected, entering TAKEOFF (soft takeoff)",
               uav_id_.c_str());
      setState(kStateTakeoff);
    }
    return;
  }
  if (!current && prev_arm_offboard_)
  {
    prev_arm_offboard_ = false;
    if (exec_state_ == kStateTakeoff)
    {
      takeoff_active_ = false;
      ROS_WARN("[%s] arm/OFFBOARD lost during TAKEOFF, reverting to HOLD",
               uav_id_.c_str());
      captureHoldTargetLocked(nowFunc());
    }
  }
}

void EgoPlannerDriverNode::resetRun()
{
  steady_target_.reset();
  arrival_since_.reset();
  hold_requested_ = false;
  // A new goal/waypoint/hold abandons the previous trajectory: the next plan
  // must start from the live pose with zero derivatives, never from the old
  // trajectory state (sampleCurrentState guards on current_).
  current_.reset();
  plan_start_ = ros::Time(0);
  emergency_braking_ = false;
  emergency_pending_ = false;
  emergency_peer_id_.clear();
  yield_committed_ = false;
  yield_peer_traj_id_ = -1;
}

void EgoPlannerDriverNode::setState(const std::string& state)
{
  if (exec_state_ == state)
    return;
  exec_state_ = state;
  std_msgs::String msg;
  msg.data = state;
  state_pub_.publish(msg);
}

bool EgoPlannerDriverNode::captureHoldTargetLocked(const ros::Time& now)
{
  // §3.1：退化到达等待中（arrival_pending_）禁止 HOLD 捕获锁存——否则会清除
  // arrival_since_ 重置稳定窗口并锁存真实位置，使同点目标永远停在 HOLD。
  if (arrival_pending_)
    return false;
  // HOLD 锁存：进入 HOLD 时读一次当前 pose 冻结 setpoint（implementation_plan_
  // 26082621 删 POSE_STALE），后续不随 pose 漂移；悬停目标建立后也不因 pose
  // 过期丢弃目标（relay 抖动/短暂丢帧不会把无人机打出稳定悬停停发 setpoint）。
  if (steady_target_)
  {
    setState(kStateHold);
    return true;
  }
  if (!local_pose_ || !local_pose_stamp_ ||
      (now - *local_pose_stamp_).toSec() > pose_timeout_)
  {
    // 无 fresh pose 无法锁存：保持 HOLD 但不发布无效 setpoint（pose 恢复后
    // publishSetpoint 的 HOLD 捕获路径自愈），不进入 POSE_STALE。
    return false;
  }

  steady_target_ = HoldTarget{
      Eigen::Vector3d(local_pose_->pose.position.x,
                      local_pose_->pose.position.y,
                      local_pose_->pose.position.z),
      yawFromQuat(local_pose_->pose.orientation)};
  current_.reset();
  arrival_since_.reset();
  plan_start_ = ros::Time(0);
  emergency_pending_ = false;
  emergency_peer_id_.clear();
  setState(kStateHold);
  return true;
}

void EgoPlannerDriverNode::publishPositionTarget(double x, double y, double z,
                                                 double yaw, uint16_t type_mask,
                                                 const Eigen::Vector3d& vel,
                                                 const Eigen::Vector3d& acc)
{
  mavros_msgs::PositionTarget m;
  m.header.stamp = nowFunc();
  m.header.frame_id = frame_id_;
  // MAVROS setpoint_raw converts the business ENU fields to NED before sending
  // the SET_POSITION_TARGET_LOCAL_NED to PX4; never re-transform here.
  m.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  m.type_mask = type_mask;
  m.position.x = x;
  m.position.y = y;
  m.position.z = z;
  m.velocity.x = vel.x();
  m.velocity.y = vel.y();
  m.velocity.z = vel.z();
  m.acceleration_or_force.x = acc.x();
  m.acceleration_or_force.y = acc.y();
  m.acceleration_or_force.z = acc.z();
  m.yaw = yaw;
  m.yaw_rate = 0.0;
  setpoint_pub_.publish(m);
  ++setpoint_count_;
  last_setpoint_ = m;
}

double EgoPlannerDriverNode::yawFromQuat(const geometry_msgs::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void EgoPlannerDriverNode::publishSetpoint(const ros::TimerEvent&)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::Time now = nowFunc();

  if (direct_control_active_)
    return;

  if (exec_state_ == kStateTakeoff)
  {
    if (!armed_ || !offboard_)
    {
      captureHoldTargetLocked(now);
      return;
    }
    if (!takeoff_active_)
    {
      takeoff_active_ = true;
      takeoff_start_z_ = local_pose_ ? local_pose_->pose.position.z : 0.0;
      takeoff_start_time_ = now;
      ROS_INFO("[%s] TAKEOFF started from z=%.2f, target z=%.2f",
               uav_id_.c_str(), takeoff_start_z_, takeoff_start_z_ + takeoff_height_);
    }
    const double elapsed = (now - takeoff_start_time_).toSec();
    const double target_z = takeoff_start_z_ + takeoff_height_;
    const double current_z = takeoff_start_z_ +
                             std::min(elapsed * takeoff_ascend_speed_, takeoff_height_);
    const double yaw = local_pose_ ? yawFromQuat(local_pose_->pose.orientation) : 0.0;
    const double x = local_pose_ ? local_pose_->pose.position.x : 0.0;
    const double y = local_pose_ ? local_pose_->pose.position.y : 0.0;
    publishPositionTarget(x, y, current_z, yaw, kMaskHold);
    if (current_z >= target_z - 0.1)
    {
      ROS_INFO("[%s] TAKEOFF complete at z=%.2f, switching to HOLD",
               uav_id_.c_str(), target_z);
      steady_target_ = HoldTarget{Eigen::Vector3d(x, y, target_z), yaw};
      takeoff_active_ = false;
      setState(kStateHold);
    }
    return;
  }

  if (steady_target_)
  {
    publishPositionTarget(steady_target_->pos.x(), steady_target_->pos.y(),
                          steady_target_->pos.z(), steady_target_->yaw, kMaskHold);
    return;
  }
  if (!current_ || current_->pts.empty())
  {
    // HOLD：steady_target_ 为空时等待 fresh pose 捕获锁存位置（删 POSE_STALE，
    // 不再发布无效 setpoint）。有待处理 goal（planning_requested_）时禁止重新
    // 捕获 steady_target_，否则会与 replanLoop 的 steady_target_ 检查竞争，
    // 导致新目标永不规划。
    if (exec_state_ == kStateHold && !planning_requested_)
      captureHoldTargetLocked(now);
    return;
  }
  const double elapsed = (now - plan_start_).toSec();
  if (elapsed < current_->duration)
  {
    // Trajectory segment: publish position + velocity + acceleration + yaw from
    // the same sample tick so PX4 receives the planning derivatives directly
    // (no position-only reset at rolling replan boundaries).
    TrajectorySample sample;
    if (current_->samplePVA(elapsed, sample))
    {
      publishPositionTarget(sample.pos.x(), sample.pos.y(), sample.pos.z(),
                            sample.yaw, kMaskEgoTrajectory, sample.vel, sample.acc);
    }
    else
    {
      // Defensive: sampling failed -> freeze the trajectory end as a HOLD.
      publishPositionTarget(current_->pts.back().x(), current_->pts.back().y(),
                            current_->pts.back().z(), current_->yaw.back(), kMaskHold);
    }
  }
  else
  {
    // Trajectory exhausted: switch to a fixed position/yaw HOLD with zero
    // velocity/acceleration before the arrival state transition, so the last
    // segment's nonzero derivatives never leak into the hold.
    publishPositionTarget(current_->pts.back().x(), current_->pts.back().y(),
                          current_->pts.back().z(), current_->yaw.back(), kMaskHold);
  }
  // 制动轨迹耗尽 → BRAKE_HOLD 锁存制动终点（方案 Y 由 replan 线程提交）。
  if (elapsed >= current_->duration && emergency_braking_)
  {
    steady_target_ = HoldTarget{current_->pts.back(), current_->yaw.back()};
    current_.reset();
    plan_start_ = ros::Time(0);
    emergency_braking_ = false;
    setState(kStateBrakeHold);
    return;
  }
  // plan-exec 分支 A/B（每 tick）：推进（距当前目标 <0.5m 切下一 keypoint）与
  // 到达（距最终关键点 <0.5m → COMPLETED）。推进时置 planning_requested_ 并
  // 事件唤醒 replanLoop（单一入口）。
  if (exec_state_ == kStateExecuting && !emergency_braking_)
    checkArrival(current_->pts.back(), current_->yaw.back(), now);
  if (!current_)
    return;  // checkArrival 已提交 COMPLETED（清空 current_）
  // 分支 C：定时 replan。elapsed >= replan_period_s（以段时长 85% 为上限，保证
  // replanLoop 快照仍能采到有效速度）→ 置位唤醒；进入最终目标 no_replan_thresh_m
  // 后停止时间触发，让最后一段轨迹自然走完触发 COMPLETED。
  const bool near_final = !waypoints_.empty() && local_pose_ &&
      (Eigen::Vector3d(local_pose_->pose.position.x, local_pose_->pose.position.y,
                       local_pose_->pose.position.z) -
       waypoints_.back().position).norm() <= no_replan_thresh_m_;
  if (!planning_requested_ && !hold_requested_ && !near_final && !emergency_braking_)
  {
    const double replan_trigger = std::min(replan_period_s_,
                                           current_->duration * 0.85);
    if (elapsed >= replan_trigger)
    {
      planning_requested_ = true;
      replan_cv_.notify_all();
    }
  }
}

std::vector<PeerTrajectoryInput> EgoPlannerDriverNode::safetyPeersLocked() const
{
  std::vector<PeerTrajectoryInput> peers;
  peers.reserve(inbox_.size());
  for (const auto& item : inbox_)
  {
    PeerTrajectoryInput peer;
    peer.uav_id = item.first;
    peer.traj_id = latest_neighbor_version_.count(item.first)
        ? latest_neighbor_version_.at(item.first).second : 0;
    peer.activation_epoch_s = item.second.first.start_time;
    peer.receive_epoch_s = item.second.second.toSec();
    peer.trajectory.duration = item.second.first.duration;
    peer.trajectory.t = item.second.first.t;
    peer.trajectory.pts = item.second.first.pts;
    // Neighbor intents currently carry position samples only. Finite-difference
    // velocity fallback in SafetyPredictor preserves same-epoch comparison.
    // Keep derivative arrays empty so samplePVA rejects them and predictor uses
    // finite differences. Filling zeros here would silently erase peer motion.
    peers.push_back(peer);
  }
  return peers;
}

std::string EgoPlannerDriverNode::formatSafetyLog(
    const SafetyPrediction& prediction, const ros::Time& now,
    double self_odom_dev_m) const
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6)
         << "ros_s=" << now.toSec()
         << " identity=" << SafetyLogger::sanitizeIdentity(exec_target_)
         << " state=" << exec_state_
         << " self_traj_id=" << traj_id_
         << " mode=" << safety_supervisor_mode_
         << " level=" << SafetyPredictor::levelName(prediction.level)
         << " primary_peer=" << prediction.primary_peer_id
         << " episode=" << (conflict_episode_.active() ? "active" : "none")
         << " action=" << (safety_supervisor_mode_ == "active" ? "ACTIVE_GATE" : "SHADOW_ONLY")
         // 缺陷 4 诊断：本机实际 odometry 与 supervisor 所用 current_ 同刻采样
         // 位置的偏差。显著 >0 表示 current_ 与实际执行脱节（时间轴/覆盖疑点）。
         << " self_odom_dev=" << self_odom_dev_m;
  for (const PairPrediction& pair : prediction.pairs)
  {
    stream << " peer={id:" << pair.peer_id
           << ",traj:" << pair.peer_traj_id
           << ",age:" << pair.data_age_s
           << ",act:" << pair.peer_activation_epoch_s
           << ",window:" << pair.common_window_s
           << ",distance:" << pair.current_distance_m
           << ",closing:" << pair.closing_speed_mps
           << ",cpa_t:" << pair.t_cpa_s
           << ",cpa_d:" << pair.cpa_distance_m
           << ",ttc:" << pair.ttc_s
           << ",min_d:" << pair.min_distance_m
           << ",warning_d:" << pair.warning_distance_m
           << ",emergency_d:" << pair.emergency_distance_m
           << ",level:" << SafetyPredictor::levelName(pair.level) << "}";
  }
  return stream.str();
}

void EgoPlannerDriverNode::runSafetySupervisor(const ros::TimerEvent&)
{
  if (safety_supervisor_mode_ == "off" || !safety_predictor_) return;
  std::shared_ptr<TimedTrajectory> self;
  std::vector<PeerTrajectoryInput> peers;
  ros::Time activation;
  ros::Time now;
  std::string self_id;
  double self_odom_dev_m = -1.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_ || plan_start_.isZero()) return;
    self.reset(new TimedTrajectory(*current_));
    peers = safetyPeersLocked();
    activation = plan_start_;
    now = nowFunc();
    self_id = !uav_id_.empty() ? uav_id_ : exec_target_;
    // 缺陷 4 诊断：实际 odometry 与 supervisor 所用 current_ 同刻采样位置偏差。
    if (local_pose_)
    {
      const Eigen::Vector3d odom(local_pose_->pose.position.x,
                                 local_pose_->pose.position.y,
                                 local_pose_->pose.position.z);
      TrajectorySample sample;
      const double elapsed = now.toSec() - activation.toSec();
      if (self->samplePVA(std::max(0.0, elapsed), sample))
        self_odom_dev_m = (odom - sample.pos).norm();
    }
  }
  const SafetyPrediction prediction = safety_predictor_->evaluate(
      *self, activation.toSec(), self_id, peers, now.toSec());
  std::string line;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_safety_prediction_ = prediction;
    conflict_episode_.update(prediction, now.toSec(),
                             episode_release_hysteresis_s_);
    line = formatSafetyLog(prediction, now, self_odom_dev_m);
  }
  safety_logger_.enqueue(line);
  if (safety_supervisor_mode_ != "active")
    return;

  // 监督层只保留 EMERGENCY 检测（L3 精简）：WARNING 级冲突交给规划层
  // （calcSwarmCost 固定 1m 软惩罚绕开）；让路候选（SLOW/LEFT/RIGHT）与
  // YIELD/STAND_ON 角色判定在 replan 失败分支（tryYieldReplan）处理。
  if (prediction.level != SafetyLevel::EMERGENCY)
    return;

  // 方案 Y（implementation_plan_26082621）：EMERGENCY 只置 emergency_pending_
  // （记录 primary peer + ttc）并事件唤醒 replanLoop；不直接 replan/制动/改
  // current_。制动由 replan 线程在"常规 replan 失败且 ttc < emergency_replan_
  // time_s"时提交（提交唯一化，消除监督回调与 replan 线程的多路径竞争）。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (emergency_braking_ || !current_ || plan_start_.isZero() ||
        plan_start_ != activation || hold_requested_)
      return;
    emergency_pending_ = true;
    emergency_peer_id_ = prediction.primary_peer_id;
    emergency_peer_ttc_ = std::numeric_limits<double>::infinity();
    for (const auto& pair : prediction.pairs)
      if (pair.peer_id == prediction.primary_peer_id)
      {
        emergency_peer_ttc_ = pair.ttc_s;
        break;
      }
    replan_cv_.notify_all();
    ROS_WARN("[%s] EMERGENCY detected (peer=%s, ttc=%.2fs); replan thread will handle",
             uav_id_.c_str(), emergency_peer_id_.c_str(), emergency_peer_ttc_);
  }
}

void EgoPlannerDriverNode::checkArrival(const Eigen::Vector3d& target, double yaw,
                                        const ros::Time& now)
{
  if (emergency_braking_ || !current_)
  {
    arrival_since_.reset();
    return;  // 制动期间不推进/不判到达（制动耗尽 → BRAKE_HOLD）
  }
  if (!local_pose_ || !local_pose_stamp_ ||
      (now - *local_pose_stamp_).toSec() > pose_timeout_)
  {
    arrival_since_.reset();
    return;
  }
  const Eigen::Vector3d cur(local_pose_->pose.position.x,
                            local_pose_->pose.position.y,
                            local_pose_->pose.position.z);
  // 分支 B：到达（||pos - 最终 keypoints|| < arrival_reach_thresh_m_ 0.5m）
  // → COMPLETED，冻结最终关键点位姿为 setpoint（executor 判成功；后续 HOLD/
  // 新目标由 onHold/onGoal 正常接管）。
  if (!waypoints_.empty() &&
      (cur - waypoints_.back().position).norm() <= arrival_reach_thresh_m_)
  {
    if (!arrival_since_)
      arrival_since_ = now;
    if ((now - *arrival_since_).toSec() >= stable_s_)
    {
      steady_target_ = HoldTarget{waypoints_.back().position, yaw};
      current_.reset();
      arrival_since_.reset();
      plan_start_ = ros::Time(0);
      emergency_pending_ = false;
      setState(kStateCompleted);
    }
    return;
  }
  arrival_since_.reset();
  // 分支 A：推进（距当前目标 <0.5m → 切相邻下一个 keypoint，置 planning_requested_）。
  if (advanceTargetLocked(cur))
  {
    planning_requested_ = true;
    replan_cv_.notify_all();
    return;
  }
  // 安全超时：轨迹耗尽后仍未推进/到达（跟踪滞后过大）→ HOLD 收口。
  const double elapsed = (now - plan_start_).toSec();
  if (elapsed >= current_->duration + timeout_margin_)
  {
    captureHoldTargetLocked(now);
    setState(kStateHold);
  }
}

void EgoPlannerDriverNode::processHoldLocked()
{
  hold_requested_ = false;
  const ros::Time now = nowFunc();
  captureHoldTargetLocked(now);
}

void EgoPlannerDriverNode::collisionCheckTick(const ros::TimerEvent&)
{
  runCollisionCheckTick();
}

void EgoPlannerDriverNode::runCollisionCheckTick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_ || plan_start_.isZero() || hold_requested_ || emergency_braking_)
    return;
  const ros::Time now = nowFunc();
  const double base = plan_start_.toSec();

  // 冲突检测：current_ vs 全部 fresh 邻机同刻最小距离（门槛 1.2×clearance，
  // yield_clearance_factor=1.2）。
  // 单一 replan 入口（方案 A）：检测到冲突只置 planning_requested_ + 唤醒
  // replanLoop；候选生成/提交/制动全部由 replan 线程统一处理（提交唯一化）。
  double min_clear = std::numeric_limits<double>::infinity();
  const double gate =
      yield_config_.clearance_factor * core_->config().swarm_clearance * 2.0;
  for (const auto& item : inbox_)
  {
    if ((now - item.second.second).toSec() > kNeighborStaleS)
      continue;
    NeighborTrajectory nb = item.second.first;
    nb.start_time -= base;
    if (nb.clearance <= 0.0 || nb.pts.size() < 2 || nb.t.size() != nb.pts.size())
      continue;
    for (size_t i = 0; i < current_->t.size(); ++i)
    {
      const double t_abs = current_->t[i] - nb.start_time;
      if (t_abs < nb.t.front() || t_abs > nb.t.back())
        continue;
      const double d = (current_->pts[i] - sampleNeighborForCollision(nb, t_abs)).norm();
      min_clear = std::min(min_clear, d);
    }
  }
  if (min_clear >= gate)
  {
    yield_committed_ = false;  // 已脱离冲突区，解除候选锁存
    return;
  }
  planning_requested_ = true;
  replan_cv_.notify_all();
}

void EgoPlannerDriverNode::commitYieldCandidate(const YieldCandidate& cand)
{
  ++safety_generation_;
  planning_requested_ = false;
  current_.reset(new TimedTrajectory(cand.trajectory));
  plan_start_ = nowFunc();
  arrival_since_.reset();
  plan_fail_since_ = ros::Time();  // 候选提交 = 软着陆恢复（退出失败计时）
  steady_target_.reset();
  emergency_braking_ = false;
  setState(kStateExecuting);
  publishIntent(*current_, plan_start_,
                swarm_uav_interfaces::UavTrajectoryIntent::PHASE_ACTIVE);
  const char* type = cand.type == YieldCandidate::Type::SLOW ? "SLOW"
                     : cand.direction == YieldDirection::LEFT ? "LEFT" : "RIGHT";
  ROS_WARN("[%s] yield committed: %s min_clear=%.2f cost=%.3f",
           uav_id_.c_str(), type, cand.min_clearance_m, cand.cost);
}

EgoPlannerDriverNode::YieldResult EgoPlannerDriverNode::tryYieldReplan(
    const ReplanSnapshot& snap)
{
  if (!enable_yield_candidates_ || !yield_generator_)
    return YieldResult::NOT_AVAILABLE;

  // 1. right-of-way：找 primary peer（邻机轨迹到 start→goal 线段最近者）。
  std::string primary_peer;
  double best_dist = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d goal = snap.waypoints.empty()
      ? snap.start.position : snap.waypoints.back().position;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time now = nowFunc();
    for (const auto& item : inbox_)
    {
      if ((now - item.second.second).toSec() > kNeighborStaleS)
        continue;
      const NeighborTrajectory& nb = item.second.first;
      for (const auto& p : nb.pts)
      {
        const double d = pointSegmentDistance(p, snap.start.position, goal);
        if (d < best_dist)
        {
          best_dist = d;
          primary_peer = item.first;
        }
      }
    }
  }
  int my_id = 0, peer_id = 0;
  const std::string my_identity = !uav_id_.empty() ? uav_id_ : exec_target_;
  if (!SafetyPredictor::parseNumericId(my_identity, my_id) ||
      !SafetyPredictor::parseNumericId(primary_peer, peer_id))
    return YieldResult::NOT_AVAILABLE;  // UNKNOWN：不生成候选。

  // 非对称（26082621 迭代）：低 ID STAND_ON 只生成 SLOW 变速候选（减速等待/加速
  // 提前错开，不横向偏移——保持 nominal 语义）；高 ID YIELD 生成 SLOW/LEFT/RIGHT
  // 全候选。episode 内锁存角色与方向。
  const bool stand_on = my_id <= peer_id;

  // episode 锁存：已提交候选且 primary peer 轨迹版本未变 → 保持（不重复提交）。
  int32_t primary_traj = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto ver = latest_neighbor_version_.find(primary_peer);
    if (ver != latest_neighbor_version_.end())
      primary_traj = ver->second.second;
    if (yield_committed_ && yield_peer_traj_id_ == primary_traj)
      return YieldResult::NOT_AVAILABLE;
  }

  // 2. 生成候选：基于 nominal（无邻居规划）轨迹由生成器统一产出
  //    SLOW（时间缩放错开）/ LEFT/RIGHT（横向 C2 偏移），验证 1.2m 门槛。
  std::vector<YieldCandidate> candidates;
  TimedTrajectory nominal, traj;
  double plan_cost = 0.0;
  bool nominal_ok = false;
  bool peer_hovering = false;
  const PlanResult nominal_result = core_->planOnce(
      snap.start, snap.waypoints, {}, nominal, plan_cost,
      snap.start_vel, snap.start_acc, snap.final_segment);
  if (nominal_result == PlanResult::RESOURCE_LIMIT)
  {
    // §9.7：nominal/candidate planOnce 返回 RESOURCE_LIMIT 必须向上传播，
    // 禁止降级成"无候选"后继续尝试其他档位或下一轮 activation retry。
    ROS_ERROR("[%s] resource limit in yield nominal planOnce", uav_id_.c_str());
    return YieldResult::RESOURCE_LIMIT;
  }
  if (nominal_result == PlanResult::SUCCESS)
  {
    nominal_ok = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ros::Time now = nowFunc();
      const auto it = inbox_.find(primary_peer);
      if (it != inbox_.end())
      {
        NeighborTrajectory nb = it->second.first;
        nb.start_time -= snap.now.toSec();
        peer_hovering = isPeerHovering(nb);
      }
    }
    // neighbors 是绝对 epoch，需 rebase 到 nominal 轨迹时间轴（snap.now）。
    std::vector<NeighborTrajectory> neighbors = snap.neighbors;
    for (auto& nb : neighbors)
      nb.start_time -= snap.now.toSec();
    candidates = yield_generator_->generate(nominal, 0.0, neighbors,
                                            peer_hovering, stand_on);
  }

  if (candidates.empty())
  {
    // 诊断（26082621 迭代）：STAND_ON/YIELD 候选全无效时记录原因。
    ROS_WARN("[%s] yield replan: no valid candidate (peer=%s stand_on=%d "
             "nominal_ok=%d peer_hovering=%d)", uav_id_.c_str(),
             primary_peer.c_str(), stand_on ? 1 : 0, nominal_ok ? 1 : 0,
             peer_hovering ? 1 : 0);
    return YieldResult::NOT_AVAILABLE;
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const YieldCandidate& a, const YieldCandidate& b)
            { return a.cost < b.cost; });
  commitYieldCandidate(candidates.front());
  yield_committed_ = true;
  yield_direction_ = candidates.front().direction;
  yield_peer_traj_id_ = primary_traj;
  ROS_WARN("[%s] yield replan committed (peer=%s)", uav_id_.c_str(),
           primary_peer.c_str());
  return YieldResult::COMMITTED;
}

bool EgoPlannerDriverNode::isPeerHovering(const NeighborTrajectory& nb) const
{
  if (nb.pts.size() < 2 || nb.t.size() != nb.pts.size())
    return false;
  const size_t n = nb.pts.size();
  const double dt = nb.t.back() - nb.t[n - 2];
  if (dt <= 1e-6)
    return false;
  const double v = (nb.pts.back() - nb.pts[n - 2]).norm() / dt;
  return v < yield_config_.hovering_velocity_mps;
}

bool EgoPlannerDriverNode::firstPlanReadyLocked(const ros::Time& now) const
{
  int my_id = 0;
  // leader（id<=1）或无法解析身份：立即首轮规划（原版 SEQUENTIAL_START
  // `drone_id <= 0` 分支）。
  if (!SafetyPredictor::parseNumericId(
          !uav_id_.empty() ? uav_id_ : exec_target_, my_id) ||
      my_id <= 1)
    return true;
  // 前驱（drone_id-1）intent 已收到 → 放行（基于前机轨迹规划首轮）。
  if (prev_intent_received_)
    return true;
  // 超时兜底：前驱缺失/失联时不永久阻塞首轮规划。
  if (!first_plan_requested_at_.isZero() &&
      (now - first_plan_requested_at_).toSec() >= first_plan_wait_timeout_s_)
    return true;
  return false;
}

void EgoPlannerDriverNode::setFirstPlanWaitTimeoutForTest(double timeout_s)
{
  first_plan_wait_timeout_s_ = timeout_s;
}

bool EgoPlannerDriverNode::firstPlanReadyForTest()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::Time now = nowFunc();
  if (first_plan_requested_at_.isZero())
    first_plan_requested_at_ = now;
  return firstPlanReadyLocked(now);
}

void EgoPlannerDriverNode::setYieldCandidatesEnabledForTest(bool enabled)
{
  enable_yield_candidates_ = enabled;
}

bool EgoPlannerDriverNode::yieldCommittedForTest() const
{
  return yield_committed_;
}

void EgoPlannerDriverNode::publishIntent(const TimedTrajectory& traj, const ros::Time& base)
{
  publishIntent(traj, base, swarm_uav_interfaces::UavTrajectoryIntent::PHASE_ACTIVE);
}

void EgoPlannerDriverNode::publishIntent(const TimedTrajectory& traj,
                                         const ros::Time& base, uint8_t phase)
{
  std::lock_guard<std::mutex> pub_lock(intent_pub_mutex_);
  swarm_uav_interfaces::UavTrajectoryIntent msg;
  msg.protocol_version = "2.0";
  msg.phase = phase;
  msg.uav_id = uav_id_;
  msg.exec_target = exec_target_;
  msg.frame_id = frame_id_;
  msg.traj_id = ++traj_id_;
  msg.stamp = base.toSec();
  msg.clearance = core_->config().swarm_clearance;
  const size_t step = std::max<size_t>(1, traj.pts.size() / 50);
  for (size_t i = 0; i < traj.pts.size(); i += step)
    appendSample(msg, traj, i);
  if (!traj.pts.empty() && (traj.pts.size() - 1) % step != 0)
    appendSample(msg, traj, traj.pts.size() - 1);
  intent_pub_.publish(msg);
}

void EgoPlannerDriverNode::appendSample(swarm_uav_interfaces::UavTrajectoryIntent& msg,
                                        const TimedTrajectory& traj, size_t i)
{
  msg.t.push_back(traj.t[i]);
  geometry_msgs::Pose pose;
  pose.position.x = traj.pts[i].x();
  pose.position.y = traj.pts[i].y();
  pose.position.z = traj.pts[i].z();
  msg.sampled_traj.push_back(pose);
}

bool EgoPlannerDriverNode::planningRequested() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return planning_requested_;
}

boost::optional<TimedTrajectory> EgoPlannerDriverNode::currentTrajectory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_)
    return boost::none;
  return *current_;
}

ros::Time EgoPlannerDriverNode::planStart() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_start_;
}

void EgoPlannerDriverNode::setReplanLoopEnabled(bool enabled)
{
  replan_loop_enabled_ = enabled;
}

ros::Time EgoPlannerDriverNode::planFailSinceForTest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_fail_since_;
}

void EgoPlannerDriverNode::replanLoop()
{
  // 单一 replan 入口（方案 A）：replanLoop 是唯一轨迹提交者。常态 replan_period_s
  // （1.0s）定时由 wait_for 超时兜底；planning_requested_/emergency_pending_ 置位
  // 时事件唤醒立即执行（条件变量）。replanOnce 内部按优先级处理：EMERGENCY 制动
  // 分支 → 常规 replan → 候选（replan 失败分支）。
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_ && replan_loop_enabled_)
  {
    const bool timed_out =
        replan_cv_.wait_for(lock,
                            std::chrono::duration<double>(replan_period_s_)) ==
        std::cv_status::timeout;
    if (stop_ || !replan_loop_enabled_)
      break;
    if (hold_requested_)
    {
      processHoldLocked();
      continue;
    }
    if (exec_state_ == kStateTakeoff)
      continue;
    if (direct_control_active_)
      continue;
    // 对齐原版 SEQUENTIAL_START：首轮（current_ 空 = 新任务未规划）规划前等待
    // 前驱无人机 intent（drone_id 顺序，前机轨迹到达才首轮规划，避免多机同时
    // 盲规划初始轨迹冲突）。leader(id<=1) / 已收到前驱 / 超时
    // first_plan_wait_timeout_s 后放行；否则清除事件等下一个 1s 周期/前驱 intent
    // 事件再试（不 busy-loop）。
    if (!current_)
    {
      const ros::Time now = nowFunc();
      if (!firstPlanReadyLocked(now))
      {
        if (first_plan_requested_at_.isZero())
          first_plan_requested_at_ = now;
        planning_requested_ = false;
        continue;
      }
    }
    // 事件置位或 1s 定时超时（timed_out）都执行一轮 replan。定时超时是软着陆
    // 失败重试的固定节奏（保持悬停，每 1s 尝试恢复）。
    if (!timed_out && !planning_requested_ && !emergency_pending_)
      continue;
    lock.unlock();
    // §9.9：replan 事务最外层窄范围 bad_alloc 边界。core 内部已把分配失败
    // 收口为 RESOURCE_LIMIT；此处兜底 node 侧其他分配（current_ 复制、邻机
    // 缓冲等）。转换为立即 EGO_PLAN_FAILED 安全收口，不允许异常逃出线程。
    bool committed = false;
    try
    {
      committed = replanOnce(timed_out);  // 唯一提交者
    }
    catch (const std::bad_alloc&)
    {
      {
        std::lock_guard<std::mutex> catch_lock(mutex_);
        captureHoldTargetLocked(nowFunc());
        setState(kStatePlanFailed);
        plan_fail_since_ = ros::Time();
      }
      ROS_ERROR("[%s] bad_alloc in replan thread; closing as EGO_PLAN_FAILED",
                uav_id_.c_str());
      committed = true;
    }
    lock.lock();
    if (committed)
      plan_fail_since_ = ros::Time();  // 成功提交（含 HOLD/制动/候选/资源收口）→ 退出软着陆
  }
}

}  // namespace ego_planner_driver
