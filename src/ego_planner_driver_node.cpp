#include "ego_planner_driver/ego_planner_driver_node.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ego_planner_driver
{
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
  config.swarm_rebound = pnh_.param("swarm_rebound", true);
  config.lambda_feasibility = pnh_.param("lambda_feasibility", 1.0);
  config.max_iterations = pnh_.param("max_iterations", 200);
  config.mem_size = pnh_.param("mem_size", 16);
  config.g_epsilon = pnh_.param("g_epsilon", 0.01);
  core_.reset(new EgoPlannerCore(config));
  planning_horizon_ = config.planning_horizon;
  replan_period_s_ = pnh_.param("replan_period_s", 1.0);
  no_replan_thresh_m_ = pnh_.param("no_replan_thresh_m", 1.0);
  setpoint_topic_ = pnh_.param("setpoint_out_topic", std::string("setpoint/ego"));
  frame_id_ = pnh_.param("frame_id", std::string("map"));
  // Identity parameters live in the public namespace (/UAV1/*) so the
  // Python driver and topology can discover them; planner tuning stays
  // private.
  uav_id_ = nh_.param("uav_id", std::string("A01"));
  exec_target_ = nh_.param("exec_target", std::string("UAV1"));
  setpoint_rate_ = pnh_.param("setpoint_rate_hz", kSetpointRateHz);
  pose_timeout_ = pnh_.param("local_pose_timeout_s", 1.0);
  pos_tol_ = pnh_.param("position_tolerance_m", 0.2);
  stable_s_ = pnh_.param("arrival_stable_s", 1.0);
  timeout_margin_ = pnh_.param("exec_timeout_margin_s", 5.0);
  takeoff_height_ = pnh_.param("takeoff_height_m", 5.0);
  takeoff_ascend_speed_ = pnh_.param("takeoff_ascend_speed", 1.0);

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
  hold_sub_ = nh_.subscribe("hold", 1, &EgoPlannerDriverNode::onHold, this);
  mavros_state_sub_ = nh_.subscribe("/mavros/state", 1, &EgoPlannerDriverNode::onMavrosState, this);
  direct_control_sub_ = nh_.subscribe("direct_control_active", 1,
                                      &EgoPlannerDriverNode::onDirectControl, this);
  setState(kStateHold);
  timer_ = nh_.createTimer(ros::Duration(1.0 / setpoint_rate_),
                           &EgoPlannerDriverNode::publishSetpoint, this);
  replan_thread_ = std::thread(&EgoPlannerDriverNode::replanLoop, this);
}

EgoPlannerDriverNode::~EgoPlannerDriverNode()
{
  stop_ = true;
  if (replan_thread_.joinable())
    replan_thread_.join();
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
    if (planning_horizon_ > 0.0)
    {
      const Eigen::Vector3d live_pos(local_pose_->pose.position.x,
                                     local_pose_->pose.position.y,
                                     local_pose_->pose.position.z);
      const bool near_final = !waypoints_.empty() &&
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
    for (const auto& item : inbox_)
    {
      if ((now - item.second.second).toSec() > kNeighborStaleS)
        continue;
      NeighborTrajectory nb = item.second.first;
      nb.start_time = nb.start_time - now.toSec();  // local axis: now==0
      data.neighbors.push_back(nb);
    }
    snap = Snapshot(std::move(data));
  }
  if (!snap)
    return false;
  TimedTrajectory traj;
  double cost = 0.0;
  if (!core_->planOnce(snap->start, snap->waypoints, snap->neighbors, traj, cost,
                       snap->start_vel, snap->start_acc))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    captureHoldTargetLocked(nowFunc());
    setState(kStatePlanFailed);
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hold_requested_)
      return true;
    completed_at = nowFunc();
    const double offset = (completed_at - snap->now).toSec();
    if (offset > 0.0)
    {
      for (double& tv : traj.t)
        tv += offset;
      traj.duration += offset;
    }
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
  if (msg->uav_id == uav_id_)
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = latest_traj_id_.find(msg->uav_id);
  if (it != latest_traj_id_.end() && msg->traj_id <= it->second)
    return;
  const size_t n = std::min(msg->t.size(), msg->sampled_traj.size());
  if (n < 2)
    return;
  NeighborTrajectory nb;
  nb.start_time = msg->stamp;  // absolute seconds; converted relative at replan
  nb.clearance = msg->clearance;
  nb.duration = msg->t[n - 1];
  nb.t.reserve(n);
  nb.pts.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    nb.t.push_back(msg->t[i]);
    nb.pts.emplace_back(msg->sampled_traj[i].position.x,
                        msg->sampled_traj[i].position.y,
                        msg->sampled_traj[i].position.z);
  }
  inbox_[msg->uav_id] = std::make_pair(nb, nowFunc());
  latest_traj_id_[msg->uav_id] = msg->traj_id;
  planning_requested_ = true;
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
    checkArrival(current_->pts.back(), current_->yaw.back(), now);
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

void EgoPlannerDriverNode::publishIntent(const TimedTrajectory& traj, const ros::Time& base)
{
  swarm_uav_interfaces::UavTrajectoryIntent msg;
  msg.protocol_version = "1.0";
  msg.uav_id = uav_id_;
  msg.exec_target = exec_target_;
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
      if (!planning_requested_ || !pending_goal_ || steady_target_)
        continue;
      planning_requested_ = false;
      const ros::Time now = nowFunc();
      if (!local_pose_ || !local_pose_stamp_ ||
          (now - *local_pose_stamp_).toSec() > pose_timeout_)
      {
        setState(kStatePoseStale);
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
      if (planning_horizon_ > 0.0)
      {
        const Eigen::Vector3d live_pos(local_pose_->pose.position.x,
                                       local_pose_->pose.position.y,
                                       local_pose_->pose.position.z);
        const bool near_final = !waypoints_.empty() &&
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
      for (const auto& item : inbox_)
      {
        if ((now - item.second.second).toSec() > kNeighborStaleS)
          continue;
        NeighborTrajectory nb = item.second.first;
        nb.start_time = nb.start_time - now.toSec();  // local axis: now==0
        data.neighbors.push_back(nb);
      }
      snap = Snapshot(std::move(data));
    }
    if (!snap)
      continue;
    TimedTrajectory traj;
    double cost = 0.0;
    if (!core_->planOnce(snap->start, snap->waypoints, snap->neighbors, traj, cost,
                         snap->start_vel, snap->start_acc))
    {
      std::lock_guard<std::mutex> lock(mutex_);
      setState(kStatePlanFailed);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (hold_requested_)
        continue;
      completed_at = nowFunc();
      const double offset = (completed_at - snap->now).toSec();
      if (offset > 0.0)
      {
        // The optimizer consumed wall-clock time after the snapshot; shift
        // the trajectory so its epoch is the publish moment, otherwise the
        // execution timeline starts mid-trajectory.
        for (double& tv : traj.t)
          tv += offset;
        traj.duration += offset;
      }
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
