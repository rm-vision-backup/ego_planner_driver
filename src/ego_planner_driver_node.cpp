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
}  // namespace

EgoPlannerDriverNode::EgoPlannerDriverNode()
    : nh_(), pnh_("~")
{
  EgoPlannerConfig config;
  config.ts = pnh_.param("ts", 0.4);
  config.max_vel = pnh_.param("max_vel", 1.0);
  config.max_acc = pnh_.param("max_acc", 6.0);
  config.planning_horizon = pnh_.param("planning_horizon", 0.0);
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
  // 规划层避碰门槛回固定 clearance（implementation_plan_26082602）：不再把
  // SafetyPredictor 动态距离参数同步给规划核心；监督层参数读入 safety_config_
  // （下方），规划层只消费 swarm_clearance×2 固定 gate。
  bias_valid_s_ = pnh_.param("bias_valid_s", 120.0);
  collision_check_rate_hz_ = pnh_.param("collision_check_rate_hz", 10.0);
  core_.reset(new EgoPlannerCore(config));
  planning_horizon_ = config.planning_horizon;
  replan_period_s_ = pnh_.param("replan_period_s", 1.0);
  no_replan_thresh_m_ = pnh_.param("no_replan_thresh_m", 1.0);
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
  pos_tol_ = pnh_.param("position_tolerance_m", 0.2);
  stable_s_ = pnh_.param("arrival_stable_s", 1.0);
  timeout_margin_ = pnh_.param("exec_timeout_margin_s", 5.0);
  takeoff_height_ = pnh_.param("takeoff_height_m", 5.0);
  takeoff_ascend_speed_ = pnh_.param("takeoff_ascend_speed", 1.0);

  safety_supervisor_mode_ = pnh_.param(
      "safety_supervisor_mode", std::string("off"));
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
  // 机间让路候选（SLOW/LEFT/RIGHT + right-of-way，§10）。
  enable_yield_candidates_ = pnh_.param("enable_yield_candidates", false);
  yield_config_.clearance_factor = pnh_.param("yield_clearance_factor", 1.3);
  yield_config_.lateral_max_m = pnh_.param("yield_lateral_max_m", 2.0);
  yield_config_.slow_min_scale = pnh_.param("yield_slow_min_scale", 0.3);
  yield_config_.max_velocity_mps = pnh_.param("emergency_max_velocity_mps", 1.5);
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
  if (replan_thread_.joinable())
    replan_thread_.join();
  safety_logger_.stop();
}

void EgoPlannerDriverNode::setClockSource(ClockFn fn)
{
  std::lock_guard<std::mutex> lock(mutex_);
  clock_source_ = std::move(fn);
}

void EgoPlannerDriverNode::setPlanningHorizon(double horizon)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planning_horizon_ = horizon;
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

bool EgoPlannerDriverNode::replanOnce()
{
  struct Snapshot
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
  boost::optional<Snapshot> snap;
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
    if (!planning_requested_ || !pending_goal_ || steady_target_)
      return false;
    planning_requested_ = false;
    const ros::Time now = nowFunc();
    if (!local_pose_ || !local_pose_stamp_ ||
        (now - *local_pose_stamp_).toSec() > pose_timeout_)
    {
      setState(kStatePoseStale);
      return false;
    }
    Snapshot data;
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
    data.waypoints = waypoints_;
    // 滚动窗口推进用"实际位置"（真实进度感知），规划起点用旧轨迹状态
    // （连续性）——两者职责分离。若窗口也用旧轨迹期望位置推进，每 1s 换段
    // 时期望位置总在下降段中段，waypoint_consumed_ 会卡住，checkArrival 永不
    // 触发。飞机进入最终目标 no_replan_thresh_m 后停止时间重规划并退化规划
    // 完整剩余航点，保证最后一段轨迹终点=目标，轨迹执行完即触发 COMPLETED。
    bool near_final = false;
    if (planning_horizon_ > 0.0)
    {
      const Eigen::Vector3d live_pos(local_pose_->pose.position.x,
                                     local_pose_->pose.position.y,
                                     local_pose_->pose.position.z);
      near_final = !waypoints_.empty() &&
          (live_pos - waypoints_.back().position).norm() <= no_replan_thresh_m_;
      if (near_final)
      {
        data.waypoints = std::vector<WayPoint>(
            waypoints_.begin() + static_cast<std::ptrdiff_t>(
                std::min(waypoint_consumed_, waypoints_.size())),
            waypoints_.end());
      }
      else
      {
        waypoint_consumed_ = advanceConsumed(
            waypoints_, live_pos, waypoint_consumed_);
        data.waypoints = truncateToHorizon(
            waypoints_, planning_horizon_, live_pos, waypoint_consumed_);
      }
    }
    // 最终段判定：已进入最终目标容差、或全局规划（horizon<=0）、或窗口内
    // 已无更多航点 -> 本段末端静止（end_vel=0）。其余巡航段 end_vel 保持
    // 切线巡航速度，保证滚动段间速度连续。
    data.final_segment = near_final || planning_horizon_ <= 0.0 ||
        waypoint_consumed_ + 1 >= waypoints_.size();
    data.neighbors = freshAbsoluteNeighborsLocked(now);
    snap = Snapshot(std::move(data));
  }
  if (!snap)
    return false;
  TimedTrajectory traj;
  double cost = 0.0;
  bool safe_candidate = false;
  ros::Time assumed_activation = snap->now;
  for (int attempt = 0; attempt < activation_replan_attempts_; ++attempt)
  {
    std::vector<NeighborTrajectory> absolute_neighbors;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      absolute_neighbors = freshAbsoluteNeighborsLocked(nowFunc());
    }
    const std::vector<NeighborTrajectory> planning_neighbors =
        rebaseNeighbors(absolute_neighbors, assumed_activation);
    if (!core_->planOnce(snap->start, snap->waypoints, planning_neighbors, traj, cost,
                         snap->start_vel, snap->start_acc, snap->final_segment))
    {
      assumed_activation = nowFunc();
      continue;
    }
    completed_at = nowFunc();
    const std::vector<NeighborTrajectory> activation_neighbors =
        rebaseNeighbors(absolute_neighbors, completed_at);
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
    std::lock_guard<std::mutex> lock(mutex_);
    if (replan_failure_fatal_)
    {
      captureHoldTargetLocked(nowFunc());
      setState(kStatePlanFailed);
    }
    return false;
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
    setState(kStateExecuting);
    publishIntent(traj, completed_at);
    // 滚动窗口不在此 erase：窗口起点由 UAV 实际位置（advanceConsumed）推进，
    // 避免"窗口消耗快于实际执行"导致后半段截取为空、UAV 停在半途。
  }
  return true;
}

std::vector<WayPoint> EgoPlannerDriverNode::truncateToHorizon(
    const std::vector<WayPoint>& waypoints, double horizon,
    const Eigen::Vector3d& start, size_t consumed) const
{
  if (horizon <= 0.0)
    return waypoints;
  // 窗口起点：UAV 实际位置前方第一个尚未消费的 waypoint。
  if (consumed >= waypoints.size())
    return {};
  std::vector<WayPoint> out;
  out.push_back(waypoints[consumed]);
  double accumulated = (waypoints[consumed].position - start).norm();
  for (size_t i = consumed + 1; i < waypoints.size(); ++i)
  {
    accumulated += (waypoints[i].position - waypoints[i - 1].position).norm();
    out.push_back(waypoints[i]);
    if (accumulated >= horizon)
      break;
  }
  return out;
}

size_t EgoPlannerDriverNode::advanceConsumed(
    const std::vector<WayPoint>& waypoints, const Eigen::Vector3d& start,
    size_t consumed) const
{
  if (consumed >= waypoints.size())
    return consumed;
  while (consumed + 1 < waypoints.size())
  {
    const Eigen::Vector3d& a = waypoints[consumed].position;
    const Eigen::Vector3d& b = waypoints[consumed + 1].position;
    const Eigen::Vector3d seg = b - a;
    const double seg_len = seg.norm();
    if (seg_len < 1e-9)
    {
      ++consumed;
      continue;
    }
    const double t = (start - a).dot(seg) / seg_len;
    if (t < 0.0)
      break;
    if (t >= seg_len)
    {
      ++consumed;
      continue;
    }
    break;
  }
  return consumed;
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
  waypoint_consumed_ = 0;
  resetRun();
  planning_requested_ = true;
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
  waypoint_consumed_ = 0;
  resetRun();
  planning_requested_ = true;
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
      msg.t.size() != msg.sampled_traj.size() || msg.t.size() < 2)
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
      prev_intent_received_ = true;
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
  // fresh 豁免：悬停目标建立后不再因 pose 过期丢弃目标，relay 抖动/
  // 短暂丢帧不会把无人机打出稳定悬停进入 POSE_STALE 停发 setpoint。
  if (steady_target_)
  {
    setState(kStateHold);
    return true;
  }
  if (!local_pose_ || !local_pose_stamp_ ||
      (now - *local_pose_stamp_).toSec() > pose_timeout_)
  {
    setState(kStatePoseStale);
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
    // HOLD/POSE_STALE：steady_target_ 为空时等待 fresh pose，避免发布无效
    // setpoint；POSE_STALE 也在 pose 恢复后自愈回 HOLD（启动时序问题）。
    // 有待处理 goal（planning_requested_）时禁止重新捕获 steady_target_，
    // 否则会与 replanLoop 的 steady_target_ 检查竞争，导致新目标永不规划。
    if ((exec_state_ == kStateHold || exec_state_ == kStatePoseStale) &&
        !planning_requested_)
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
  // Periodic replan (time-triggered): once the current segment has executed
  // for replan_period_s (bounded by 85% of the trajectory duration so the
  // replanLoop snapshot still samples valid velocity for segment continuity)
  // and there are still waypoints ahead, request a replan.  Arrival still
  // triggers the next segment; the final segment publishes COMPLETED.
  // Once the UAV is inside the final target tolerance, time-triggered replan
  // stops so the final (degraded full-remaining) segment can run to its end
  // and trigger checkArrival -> COMPLETED.
  const bool near_final = !waypoints_.empty() && local_pose_ &&
      (Eigen::Vector3d(local_pose_->pose.position.x, local_pose_->pose.position.y,
                       local_pose_->pose.position.z) -
       waypoints_.back().position).norm() <= no_replan_thresh_m_;
  if (planning_horizon_ > 0.0 && !planning_requested_ && !hold_requested_ &&
      waypoint_consumed_ + 1 < waypoints_.size() && !near_final)
  {
    const double replan_trigger = std::min(replan_period_s_,
                                           current_ ? current_->duration * 0.85 : replan_period_s_);
    if (elapsed >= replan_trigger)
      planning_requested_ = true;
  }
  if (elapsed >= current_->duration)
  {
    if (emergency_braking_)
    {
      steady_target_ = HoldTarget{current_->pts.back(), current_->yaw.back()};
      current_.reset();
      plan_start_ = ros::Time(0);
      emergency_braking_ = false;
      setState(kStateBrakeHold);
      return;
    }
    checkArrival(current_->pts.back(), current_->yaw.back(), now);
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

  // 监督层只保留 EMERGENCY 制动兜底（L3 精简）：WARNING 级冲突交给规划层
  // （calcSwarmCost 动态距离软惩罚绕开）；让路候选（SLOW/LEFT/RIGHT/FAST）与
  // YIELD/STAND_ON 角色判定已删除——SITL 实证正交交叉场景单机让路几何不足。
  if (prediction.level != SafetyLevel::EMERGENCY)
    return;

  // 对齐原版 EGOReplanFSM::checkCollisionCallback（planFromCurrentTraj 优先）：
  // EMERGENCY 先尝试立即重规划（replan_failure_fatal_=false，失败不收口），
  // 成功则继续执行；失败再按预测越界时间（primary peer 的 ttc）决定急停或
  // 交常规 replanLoop（≤1s）兜底。data_age 不再放大门槛（邻机时间戳已修正）。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (emergency_braking_ || !current_ || plan_start_ != activation || hold_requested_)
      return;
    planning_requested_ = true;
    replan_failure_fatal_ = false;
  }
  const bool replanned = replanOnce();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    replan_failure_fatal_ = true;
  }
  if (replanned)
  {
    ROS_WARN("[%s] EMERGENCY mitigated by immediate replan (peer=%s)",
             uav_id_.c_str(), prediction.primary_peer_id.c_str());
    return;
  }
  double primary_ttc_s = std::numeric_limits<double>::infinity();
  for (const auto& pair : prediction.pairs)
    if (pair.peer_id == prediction.primary_peer_id)
      primary_ttc_s = pair.ttc_s;
  if (!std::isfinite(primary_ttc_s) || primary_ttc_s > emergency_replan_time_s_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planning_requested_ = true;  // 常规 replanLoop 兜底（≤1s 重试）
    ROS_WARN("[%s] EMERGENCY replan failed; ttc=%.2fs > %.2fs, waiting replanLoop",
             uav_id_.c_str(), primary_ttc_s, emergency_replan_time_s_);
    return;
  }
  ROS_WARN("[%s] EMERGENCY replan failed; ttc=%.2fs <= %.2fs, braking",
           uav_id_.c_str(), primary_ttc_s, emergency_replan_time_s_);

  TrajectorySample start;
  if (!self->samplePVA(std::max(0.0, now.toSec() - activation.toSec()), start))
    return;
  TimedTrajectory braking;
  if (!emergency_brake_generator_ ||
      !emergency_brake_generator_->generate(start, braking))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_ && plan_start_ == activation)
    {
      ++safety_generation_;
      planning_requested_ = false;
      current_.reset();
      plan_start_ = ros::Time(0);
      emergency_braking_ = false;
      steady_target_ = HoldTarget{start.pos, start.yaw};
      setState(kStateBrakeHold);
    }
    ROS_ERROR("[%s] emergency brake generation failed; nominal aborted to fallback HOLD",
              uav_id_.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (emergency_braking_ || !current_ || plan_start_ != activation)
      return;
    ++safety_generation_;
    planning_requested_ = false;
    current_.reset(new TimedTrajectory(braking));
    plan_start_ = now;
    arrival_since_.reset();
    steady_target_.reset();
    emergency_braking_ = true;
    setState(kStateEmergencyBrake);
    publishIntent(braking, now,
                  swarm_uav_interfaces::UavTrajectoryIntent::PHASE_BRAKING);
    ROS_ERROR("[%s] emergency brake committed for peer=%s",
              uav_id_.c_str(), prediction.primary_peer_id.c_str());
  }
}

void EgoPlannerDriverNode::checkArrival(const Eigen::Vector3d& target, double yaw,
                                        const ros::Time& now)
{
  if (!local_pose_ || !local_pose_stamp_ ||
      (now - *local_pose_stamp_).toSec() > pose_timeout_)
  {
    arrival_since_.reset();
    return;
  }
  const Eigen::Vector3d cur(local_pose_->pose.position.x,
                            local_pose_->pose.position.y,
                            local_pose_->pose.position.z);
  const double dist = (cur - target).norm();
  if (dist <= pos_tol_)
  {
    if (!arrival_since_)
      arrival_since_ = now;
    if ((now - *arrival_since_).toSec() >= stable_s_)
    {
      // COMPLETED 判定不依赖 waypoint_consumed_：飞机进入最终目标容差
      // （near_final）即视为到达最终目标。否则滚动窗口的 consumed 会因
      // 密化末段的投影阈值（t<seg_len）卡在 size-2，导致永远滚动、永不
      // COMPLETED（见 implementation_plan 26082322 复测）。
      const bool near_final = !waypoints_.empty() &&
          (cur - waypoints_.back().position).norm() <= pos_tol_;
      if (planning_horizon_ <= 0.0 || near_final ||
          waypoint_consumed_ + 1 >= waypoints_.size())
      {
        // 最终目标已到达：冻结目标位姿并发布 COMPLETED 终态（executor 判定成功）。
        // steady_target_ 非空时持续发布目标 setpoint 悬停；不再单独切 HOLD，
        // 避免 executor 收到 HOLD 判 COMMAND_HELD 失败。后续 HOLD/新目标由
        // onHold/onGoal 正常接管。
        steady_target_ = HoldTarget{target, yaw};
        current_.reset();
        arrival_since_.reset();
        plan_start_ = ros::Time(0);
        setState(kStateCompleted);
      }
      else
      {
        // 还有后续航点：滚动规划下一段（receding-horizon）。
        current_.reset();
        arrival_since_.reset();
        plan_start_ = ros::Time(0);
        planning_requested_ = true;
      }
    }
    return;
  }
  arrival_since_.reset();
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

  // 1. 冲突检测：current_ vs 全部 fresh 邻机同刻最小距离（门槛 1.3×clearance）。
  std::string primary_peer;
  int32_t primary_traj = -1;
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
      if (d < min_clear)
      {
        min_clear = d;
        primary_peer = item.first;
      }
    }
  }
  if (min_clear >= gate)
  {
    yield_committed_ = false;  // 已脱离冲突区，解除锁存
    return;
  }
  const auto ver = latest_neighbor_version_.find(primary_peer);
  if (ver != latest_neighbor_version_.end())
    primary_traj = ver->second.second;

  // 2. right-of-way：低 ID 保持（STAND_ON），高 ID 让路（YIELD）。
  int my_id = 0, peer_id = 0;
  const std::string my_identity = !uav_id_.empty() ? uav_id_ : exec_target_;
  const bool my_ok = SafetyPredictor::parseNumericId(my_identity, my_id);
  const bool peer_ok = SafetyPredictor::parseNumericId(primary_peer, peer_id);
  if (!my_ok || !peer_ok || my_id <= peer_id)
  {
    // 本机 STAND_ON（ID 数字小于等于 primary peer）：保持 nominal，交常规 replan 兜底。
    planning_requested_ = true;
    return;
  }
  // 本机 YIELD。
  if (!enable_yield_candidates_ || !yield_generator_)
  {
    planning_requested_ = true;
    return;
  }

  // 3. episode 锁存：已提交且 primary peer 版本未变 → 保持（不重复生成/提交）。
  if (yield_committed_ && yield_peer_traj_id_ == primary_traj)
    return;

  // 4. 悬停前机判定（跳过 SLOW）。
  bool peer_hovering = false;
  const auto it = inbox_.find(primary_peer);
  if (it != inbox_.end())
  {
    NeighborTrajectory nb = it->second.first;
    nb.start_time -= base;
    peer_hovering = isPeerHovering(nb);
  }

  // 5. 生成候选 + cost 最优 + 提交发布。
  const double t_elapsed = std::max(0.0, (now - plan_start_).toSec());
  const std::vector<NeighborTrajectory> neighbors = rebaseNeighbors(
      freshAbsoluteNeighborsLocked(now), plan_start_);
  const std::vector<YieldCandidate> candidates =
      yield_generator_->generate(*current_, t_elapsed, neighbors, peer_hovering);
  if (candidates.empty())
  {
    planning_requested_ = true;  // 全失败 → 常规 replan。
    return;
  }
  const YieldCandidate& best = candidates.front();
  commitYieldCandidate(best);
  yield_committed_ = true;
  yield_direction_ = best.direction;
  yield_peer_traj_id_ = primary_traj;
}

void EgoPlannerDriverNode::commitYieldCandidate(const YieldCandidate& cand)
{
  ++safety_generation_;
  planning_requested_ = false;
  current_.reset(new TimedTrajectory(cand.trajectory));
  plan_start_ = nowFunc();
  arrival_since_.reset();
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

void EgoPlannerDriverNode::replanLoop()
{
  const ros::Duration period(kReplanPeriodS);
  while (!stop_ && replan_loop_enabled_)
  {
    period.sleep();
    if (ros::isShuttingDown())
      break;
    if (!replan_loop_enabled_)
      break;
    struct Snapshot
    {
      GoalPoint start;
      std::vector<WayPoint> waypoints;
      std::vector<NeighborTrajectory> neighbors;
      ros::Time now;
      Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
      Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
      bool final_segment = false;
    };
    boost::optional<Snapshot> snap;
    ros::Time completed_at;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (hold_requested_)
      {
        processHoldLocked();
        continue;
      }
      if (exec_state_ == kStateTakeoff)
        continue;
      // 缺陷 8：紧急制动期间禁止任何常规 replan（含快照），否则制动轨迹会被
      // 下一个周期覆盖为 nominal，且 emergency_braking_ 遗留导致后续 EMERGENCY
      // 永远无法再次制动（与 replanOnce 的 commit 门禁保持一致）。
      if (!planning_requested_ || !pending_goal_ || steady_target_ ||
          emergency_braking_)
        continue;
      planning_requested_ = false;
      const ros::Time now = nowFunc();
      if (!local_pose_ || !local_pose_stamp_ ||
          (now - *local_pose_stamp_).toSec() > pose_timeout_)
      {
        setState(kStatePoseStale);
        continue;
      }
      // 对齐原版 SEQUENTIAL_START：首轮（current_ 空 = 新任务未规划）规划前
      // 等待前驱无人机 intent（drone_id 顺序，前机轨迹到达才首轮规划，避免
      // 多机同时盲规划初始轨迹冲突）。leader(id<=1) / 已收到前驱 / 超时
      // first_plan_wait_timeout_s 后放行；否则保持待规划并跳过本轮重试。
      if (!current_ && !firstPlanReadyLocked(now))
      {
        if (first_plan_requested_at_.isZero())
          first_plan_requested_at_ = now;
        planning_requested_ = true;
        continue;
      }
      Snapshot data;
      data.now = now;
      data.start.yaw = goal_yaw_;
      // 滚动重规划：从旧轨迹同一时刻取完整 P/V/A/yaw（原版 FSM 语义）；
      // 首次规划/新任务才用实际位置 + 零导数（见 replanOnce 同款逻辑）。
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
      data.waypoints = waypoints_;
      // 滚动窗口推进用"实际位置"（真实进度感知），规划起点用旧轨迹状态
      // （连续性）——两者职责分离（见 replanOnce 同款说明）。一次规划完整
      // 航线（120+ 点）会让 B-spline 参数化把长水平段压缩在起点附近，必须
      // 滚动窗口逐段推进。
      bool near_final = false;
      if (planning_horizon_ > 0.0)
      {
        const Eigen::Vector3d live_pos(local_pose_->pose.position.x,
                                       local_pose_->pose.position.y,
                                       local_pose_->pose.position.z);
        near_final = !waypoints_.empty() &&
            (live_pos - waypoints_.back().position).norm() <= no_replan_thresh_m_;
        if (near_final)
        {
          // 飞机已进入最终目标容差：退化规划完整剩余航点（含目标），并让
          // 重规划停止（publishSetpoint 侧同样用 near_final 关闭时间触发），
          // 保证最后一段轨迹执行完触发 COMPLETED。
          data.waypoints = std::vector<WayPoint>(
              waypoints_.begin() + static_cast<std::ptrdiff_t>(
                  std::min(waypoint_consumed_, waypoints_.size())),
              waypoints_.end());
        }
        else
        {
          waypoint_consumed_ = advanceConsumed(
              waypoints_, live_pos, waypoint_consumed_);
          data.waypoints = truncateToHorizon(
              waypoints_, planning_horizon_, live_pos, waypoint_consumed_);
          if (data.waypoints.size() < 2)
          {
            // 窗口内不足两点（已接近最终目标）：退化一次规划完整剩余航点。
            data.waypoints = std::vector<WayPoint>(
                waypoints_.begin() + static_cast<std::ptrdiff_t>(
                    std::min(waypoint_consumed_, waypoints_.size())),
                waypoints_.end());
          }
        }
      }
      // 最终段判定（同 replanOnce）：近目标 / 全局规划 / 窗口无更多航点。
      data.final_segment = near_final || planning_horizon_ <= 0.0 ||
          waypoint_consumed_ + 1 >= waypoints_.size();
      data.neighbors = freshAbsoluteNeighborsLocked(now);
      snap = Snapshot(std::move(data));
    }
    if (!snap)
      continue;
    TimedTrajectory traj;
    double cost = 0.0;
    bool safe_candidate = false;
    ros::Time assumed_activation = snap->now;
    for (int attempt = 0; attempt < activation_replan_attempts_; ++attempt)
    {
      std::vector<NeighborTrajectory> absolute_neighbors;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        absolute_neighbors = freshAbsoluteNeighborsLocked(nowFunc());
      }
      const std::vector<NeighborTrajectory> planning_neighbors =
          rebaseNeighbors(absolute_neighbors, assumed_activation);
      if (!core_->planOnce(snap->start, snap->waypoints, planning_neighbors, traj, cost,
                           snap->start_vel, snap->start_acc, snap->final_segment))
      {
        assumed_activation = nowFunc();
        continue;
      }
      completed_at = nowFunc();
      const std::vector<NeighborTrajectory> activation_neighbors =
          rebaseNeighbors(absolute_neighbors, completed_at);
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
      std::lock_guard<std::mutex> lock(mutex_);
      captureHoldTargetLocked(nowFunc());
      setState(kStatePlanFailed);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (hold_requested_ || emergency_braking_)
        continue;
      const std::vector<NeighborTrajectory> commit_neighbors = rebaseNeighbors(
          freshAbsoluteNeighborsLocked(completed_at), completed_at);
      if (!core_->checkClearance(traj, commit_neighbors,
                                 core_->config().swarm_clearance * 2.0))
      {
        captureHoldTargetLocked(completed_at);
        setState(kStatePlanFailed);
        continue;
      }
      // Activation epoch accounts for planning latency exactly once; traj.t
      // remains relative and starts at zero.
      current_.reset(new TimedTrajectory(traj));
      plan_start_ = completed_at;
      arrival_since_.reset();
      steady_target_.reset();
      setState(kStateExecuting);
      publishIntent(traj, completed_at);
    }
  }
}

}  // namespace ego_planner_driver
