#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <swarm_uav_interfaces/UavTrajectoryIntent.h>
#include <boost/optional.hpp>
#include <cmath>
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

namespace
{
constexpr double kSetpointRateHz = 30.0;
constexpr double kNeighborStaleS = 5.0;
constexpr double kReplanPeriodS = 0.5;
const std::string ST_IDLE = "IDLE";
const std::string ST_EXEC = "EXECUTING";
const std::string ST_DONE = "COMPLETED";
const std::string ST_HOLD = "HOLD";
const std::string ST_POSE = "POSE_STALE";
const std::string ST_PLAN = "EGO_PLAN_FAILED";
const std::string ST_TIMEOUT = "EGO_EXEC_TIMEOUT";
struct HoldTarget { Eigen::Vector3d pos; double yaw; };
}  // namespace

class EgoPlannerDriverNode
{
public:
  EgoPlannerDriverNode()
  {
    ros::NodeHandle pnh("~");
    EgoPlannerConfig config;
    config.ts = pnh.param("ts", 0.4);
    config.max_vel = pnh.param("max_vel", 8.0);
    config.max_acc = pnh.param("max_acc", 6.0);
    config.swarm_clearance = pnh.param("swarm_clearance", 2.0);
    config.planning_range = pnh.param("planning_range", 30.0);
    config.lambda_smooth = pnh.param("lambda_smooth", 1.0);
    config.lambda_swarm = pnh.param("lambda_swarm", 1.0);
    config.lambda_feasibility = pnh.param("lambda_feasibility", 1.0);
    config.max_iterations = pnh.param("max_iterations", 200);
    config.mem_size = pnh.param("mem_size", 16);
    config.g_epsilon = pnh.param("g_epsilon", 0.01);
    core_.reset(new EgoPlannerCore(config));
    frame_id_ = pnh.param("frame_id", std::string("map"));
    uav_id_ = pnh.param("uav_id", std::string("A01"));
    exec_target_ = pnh.param("exec_target", std::string("UAV1"));
    setpoint_rate_ = pnh.param("setpoint_rate_hz", kSetpointRateHz);
    pose_timeout_ = pnh.param("local_pose_timeout_s", 1.0);
    pos_tol_ = pnh.param("position_tolerance_m", 0.2);
    stable_s_ = pnh.param("arrival_stable_s", 1.0);
    timeout_margin_ = pnh.param("exec_timeout_margin_s", 5.0);

    setpoint_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("setpoint", 10);
    intent_pub_ = pnh.advertise<swarm_uav_interfaces::UavTrajectoryIntent>("trajectory_intent", 10);
    state_pub_ = pnh.advertise<std_msgs::String>("exec_state", 10, true);
    goal_sub_ = pnh.subscribe("goal", 1, &EgoPlannerDriverNode::onGoal, this);
    goal_yaw_sub_ = pnh.subscribe("goal_yaw", 1, &EgoPlannerDriverNode::onGoalYaw, this);
    waypoints_sub_ = pnh.subscribe("waypoints", 1, &EgoPlannerDriverNode::onWaypoints, this);
    pose_sub_ = pnh.subscribe("local_pose", 1, &EgoPlannerDriverNode::onLocalPose, this);
    intent_sub_ = pnh.subscribe("neighbor_intent", 10, &EgoPlannerDriverNode::onNeighborIntent, this);
    hold_sub_ = pnh.subscribe("hold", 1, &EgoPlannerDriverNode::onHold, this);
    setState(ST_IDLE);
    timer_ = pnh.createTimer(ros::Duration(1.0 / setpoint_rate_),
                             &EgoPlannerDriverNode::publishSetpoint, this);
    replan_thread_ = std::thread(&EgoPlannerDriverNode::replanLoop, this);
  }

  ~EgoPlannerDriverNode()
  {
    stop_ = true;
    if (replan_thread_.joinable())
      replan_thread_.join();
  }

private:
  void onGoal(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_goal_ = GoalPoint{Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z), goal_yaw_};
    waypoints_.assign(1, WayPoint{pending_goal_->position, goal_yaw_});
    resetRun();
    planning_requested_ = true;
  }

  void onGoalYaw(const std_msgs::Float64::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_yaw_ = msg->data;
    if (pending_goal_)
      pending_goal_->yaw = goal_yaw_;
  }

  void onWaypoints(const geometry_msgs::PolygonStamped::ConstPtr& msg)
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

  void onLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_pose_ = *msg;
    local_pose_stamp_ = msg->header.stamp;
  }

  void onNeighborIntent(const swarm_uav_interfaces::UavTrajectoryIntent::ConstPtr& msg)
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
    inbox_[msg->uav_id] = std::make_pair(nb, ros::Time::now());
    latest_traj_id_[msg->uav_id] = msg->traj_id;
    planning_requested_ = true;
  }

  void onHold(const std_msgs::Empty::ConstPtr&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_requested_ = true;
    planning_requested_ = false;
  }

  void resetRun()
  {
    steady_target_.reset();
    arrival_since_.reset();
    hold_requested_ = false;
  }

  void setState(const std::string& state)
  {
    if (exec_state_ == state)
      return;
    exec_state_ = state;
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  void publishPose(double x, double y, double z, double yaw)
  {
    geometry_msgs::PoseStamped m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = frame_id_;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = std::sin(yaw * 0.5);
    m.pose.orientation.w = std::cos(yaw * 0.5);
    setpoint_pub_.publish(m);
  }

  static double yawFromQuat(const geometry_msgs::Quaternion& q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void publishSetpoint(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (steady_target_)
    {
      publishPose(steady_target_->pos.x(), steady_target_->pos.y(),
                  steady_target_->pos.z(), steady_target_->yaw);
      return;
    }
    if (!current_ || current_->pts.empty())
      return;
    const ros::Time now = ros::Time::now();
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

  void checkArrival(const Eigen::Vector3d& target, double yaw, const ros::Time& now)
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
        setState(ST_DONE);
        steady_target_ = HoldTarget{target, yaw};
        current_.reset();
        arrival_since_.reset();
        plan_start_ = ros::Time(0);
      }
      return;
    }
    arrival_since_.reset();
    const double elapsed = (now - plan_start_).toSec();
    if (elapsed >= current_->duration + timeout_margin_)
    {
      setState(ST_TIMEOUT);
      steady_target_ = HoldTarget{target, yaw};
      current_.reset();
      plan_start_ = ros::Time(0);
    }
  }

  void publishIntent(const TimedTrajectory& traj, const ros::Time& base)
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

  static void appendSample(swarm_uav_interfaces::UavTrajectoryIntent& msg,
                           const TimedTrajectory& traj, size_t i)
  {
    msg.t.push_back(traj.t[i]);
    geometry_msgs::Pose pose;
    pose.position.x = traj.pts[i].x();
    pose.position.y = traj.pts[i].y();
    pose.position.z = traj.pts[i].z();
    msg.sampled_traj.push_back(pose);
  }

  void replanLoop()
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
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hold_requested_)
        {
          hold_requested_ = false;
          const ros::Time now = ros::Time::now();
          if (local_pose_ && local_pose_stamp_ &&
              (now - *local_pose_stamp_).toSec() <= pose_timeout_)
            steady_target_ = HoldTarget{
                Eigen::Vector3d(local_pose_->pose.position.x,
                                local_pose_->pose.position.y,
                                local_pose_->pose.position.z),
                yawFromQuat(local_pose_->pose.orientation)};
          else
            steady_target_.reset();  // fail-safe: no fresh pose, stop publishing setpoints
          current_.reset();
          arrival_since_.reset();
          plan_start_ = ros::Time(0);
          setState(ST_HOLD);
          continue;
        }
        if (!planning_requested_ || !pending_goal_ || steady_target_)
          continue;
        planning_requested_ = false;
        const ros::Time now = ros::Time::now();
        if (!local_pose_ || !local_pose_stamp_ ||
            (now - *local_pose_stamp_).toSec() > pose_timeout_)
        {
          setState(ST_POSE);
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
        setState(ST_PLAN);
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hold_requested_)
          continue;
        const ros::Time completed_at = ros::Time::now();
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
        setState(ST_EXEC);
        publishIntent(traj, completed_at);
      }
    }
  }

  ros::NodeHandle pnh_;
  std::unique_ptr<EgoPlannerCore> core_;
  std::string frame_id_;
  std::string uav_id_;
  std::string exec_target_;
  double setpoint_rate_ = kSetpointRateHz;
  double pose_timeout_ = 1.0;
  double pos_tol_ = 0.2;
  double stable_s_ = 1.0;
  double timeout_margin_ = 5.0;

  ros::Publisher setpoint_pub_;
  ros::Publisher intent_pub_;
  ros::Publisher state_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber goal_yaw_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber intent_sub_;
  ros::Subscriber hold_sub_;
  ros::Timer timer_;

  std::mutex mutex_;
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

  std::map<std::string, int32_t> latest_traj_id_;
  std::map<std::string, std::pair<NeighborTrajectory, ros::Time>> inbox_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_planner_driver_node");
  EgoPlannerDriverNode node;
  ros::spin();
  return 0;
}