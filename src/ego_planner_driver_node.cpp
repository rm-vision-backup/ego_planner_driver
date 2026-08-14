#include "ego_planner_driver/ego_planner_driver_node.h"

#include <algorithm>
#include <cmath>

namespace ego_planner_driver
{
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
  config.lambda_feasibility = pnh_.param("lambda_feasibility", 1.0);
  config.max_iterations = pnh_.param("max_iterations", 200);
  config.mem_size = pnh_.param("mem_size", 16);
  config.g_epsilon = pnh_.param("g_epsilon", 0.01);
  core_.reset(new EgoPlannerCore(config));
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
  setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("setpoint", 10);
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

bool EgoPlannerDriverNode::replanOnce()
{
  struct Snapshot
  {
    GoalPoint start;
    std::vector<WayPoint> waypoints;
    std::vector<NeighborTrajectory> neighbors;
    ros::Time now;
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
    data.start.position = Eigen::Vector3d(local_pose_->pose.position.x,
                                          local_pose_->pose.position.y,
                                          local_pose_->pose.position.z);
    data.start.yaw = goal_yaw_;
    data.waypoints = waypoints_;
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
  if (!core_->planOnce(snap->start, snap->waypoints, snap->neighbors, traj, cost))
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
  }
  return true;
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

geometry_msgs::PoseStamped EgoPlannerDriverNode::lastSetpoint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_setpoint_ ? *last_setpoint_ : geometry_msgs::PoseStamped();
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

void EgoPlannerDriverNode::onDirectControl(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  direct_control_active_ = msg->data;
  if (direct_control_active_)
  {
    planning_requested_ = false;
    current_.reset();
    steady_target_.reset();
    arrival_since_.reset();
    setState(kStateHold);
  }
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

void EgoPlannerDriverNode::publishPose(double x, double y, double z, double yaw)
{
  geometry_msgs::PoseStamped m;
  m.header.stamp = nowFunc();
  m.header.frame_id = frame_id_;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = std::sin(yaw * 0.5);
  m.pose.orientation.w = std::cos(yaw * 0.5);
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
    publishPose(x, y, current_z, yaw);
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
    publishPose(steady_target_->pos.x(), steady_target_->pos.y(),
                steady_target_->pos.z(), steady_target_->yaw);
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
  Eigen::Vector3d sample = current_->pts.back();
  double yaw = current_->yaw.back();
  if (elapsed < current_->duration)
  {
    for (size_t i = 0; i + 1 < current_->pts.size(); ++i)
    {
      if (elapsed < current_->t[i + 1])
      {
        const double a = (elapsed - current_->t[i]) / (current_->t[i + 1] - current_->t[i]);
        sample = current_->pts[i] * (1.0 - a) + current_->pts[i + 1] * a;
        yaw = current_->yaw[i] * (1.0 - a) + current_->yaw[i + 1] * a;
        break;
      }
    }
  }
  publishPose(sample.x(), sample.y(), sample.z(), yaw);
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
  if ((cur - target).norm() <= pos_tol_)
  {
    if (!arrival_since_)
      arrival_since_ = now;
    if ((now - *arrival_since_).toSec() >= stable_s_)
    {
      steady_target_ = HoldTarget{target, yaw};
      current_.reset();
      arrival_since_.reset();
      plan_start_ = ros::Time(0);
      setState(kStateHold);
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

void EgoPlannerDriverNode::replanLoop()
{
  const ros::Duration period(kReplanPeriodS);
  while (!stop_)
  {
    period.sleep();
    if (ros::isShuttingDown())
      break;
    struct Snapshot
    {
      GoalPoint start;
      std::vector<WayPoint> waypoints;
      std::vector<NeighborTrajectory> neighbors;
      ros::Time now;
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
      data.start.position = Eigen::Vector3d(local_pose_->pose.position.x,
                                            local_pose_->pose.position.y,
                                            local_pose_->pose.position.z);
      data.start.yaw = goal_yaw_;
      data.waypoints = waypoints_;
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
    if (!core_->planOnce(snap->start, snap->waypoints, snap->neighbors, traj, cost))
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
