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
#include <set>
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
} // namespace

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
    const double setpoint_rate_hz = pnh.param("setpoint_rate_hz", kSetpointRateHz);
    const double local_pose_timeout_s = pnh.param("local_pose_timeout_s", 1.0);

    setpoint_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("setpoint", 10);
    intent_pub_ = pnh.advertise<swarm_uav_interfaces::UavTrajectoryIntent>("trajectory_intent", 10);
    state_pub_ = pnh.advertise<std_msgs::String>("exec_state", 10, true);

    goal_sub_ = pnh.subscribe("goal", 1, &EgoPlannerDriverNode::onGoal, this);
    goal_yaw_sub_ = pnh.subscribe("goal_yaw", 1, &EgoPlannerDriverNode::onGoalYaw, this);
    waypoints_sub_ = pnh.subscribe("waypoints", 1, &EgoPlannerDriverNode::onWaypoints, this);
    pose_sub_ = pnh.subscribe("local_pose", 1, &EgoPlannerDriverNode::onLocalPose, this);
    intent_sub_ = pnh.subscribe("neighbor_intent", 10, &EgoPlannerDriverNode::onNeighborIntent, this);
    hold_sub_ = pnh.subscribe("hold", 1, &EgoPlannerDriverNode::onHold, this);

    publishState("IDLE");
    timer_ = pnh.createTimer(ros::Duration(1.0 / setpoint_rate_hz),
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
    waypoints_.clear();
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
    std::vector<WayPoint> waypoints;
    for (const auto& point : msg->polygon.points)
      waypoints.push_back(WayPoint{Eigen::Vector3d(point.x, point.y, point.z), goal_yaw_});
    if (waypoints.empty())
      return;
    pending_goal_ = GoalPoint{waypoints.back().position, goal_yaw_};
    waypoints_ = waypoints;
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
    const std::string key = msg->uav_id + "|" + std::to_string(msg->traj_id);
    if (neighbor_keys_.count(key))
      return; // out-of-order dedupe

    NeighborTrajectory neighbor;
    neighbor.drone_id = static_cast<int>(neighbor_keys_.size());
    neighbor.start_time = msg->stamp;
    neighbor.clearance = msg->clearance;
    neighbor.duration = msg->t.empty() ? 0.0 : msg->t.back();
    const size_t count = std::min(msg->t.size(), msg->sampled_traj.size());
    for (size_t i = 0; i < count; ++i)
    {
      neighbor.t.push_back(msg->t[i]);
      const auto& pose = msg->sampled_traj[i];
      neighbor.pts.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    }
    if (neighbor.pts.size() < 2)
      return;
    neighbor_keys_.insert(key);
    neighbor_trajs_[msg->uav_id] = std::make_pair(neighbor, ros::Time::now());
    planning_requested_ = true;
  }

  void onHold(const std_msgs::Empty::ConstPtr&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_requested_ = true;
    planning_requested_ = false;
  }

  void publishState(const std::string& state)
  {
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  void publishPose(double x, double y, double z, double yaw)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = std::sin(yaw * 0.5);
    msg.pose.orientation.w = std::cos(yaw * 0.5);
    setpoint_pub_.publish(msg);
  }

  void publishSetpoint(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_ || current_->pts.empty())
      return;
    const double elapsed = (ros::Time::now() - plan_start_).toSec();
    if (elapsed >= current_->duration)
    {
      publishPose(current_->pts.back().x(), current_->pts.back().y(),
                  current_->pts.back().z(), current_->yaw.back());
      return;
    }
    Eigen::Vector3d sample = current_->pts.back();
    double yaw = current_->yaw.back();
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
    publishPose(sample.x(), sample.y(), sample.z(), yaw);
  }

  void publishIntent(const TimedTrajectory& trajectory)
  {
    swarm_uav_interfaces::UavTrajectoryIntent msg;
    msg.protocol_version = "1.0";
    msg.uav_id = uav_id_;
    msg.exec_target = exec_target_;
    msg.traj_id = ++traj_id_;
    msg.stamp = ros::Time::now().toSec();
    msg.clearance = core_->config().swarm_clearance;

    const size_t step = std::max<size_t>(1, trajectory.pts.size() / 50);
    for (size_t i = 0; i < trajectory.pts.size(); i += step)
      appendSample(msg, trajectory, i);
    if (!trajectory.pts.empty() && (trajectory.pts.size() - 1) % step != 0)
      appendSample(msg, trajectory, trajectory.pts.size() - 1);
    intent_pub_.publish(msg);
  }

  static void appendSample(swarm_uav_interfaces::UavTrajectoryIntent& msg,
                           const TimedTrajectory& trajectory, size_t i)
  {
    msg.t.push_back(trajectory.t[i]);
    geometry_msgs::Pose pose;
    pose.position.x = trajectory.pts[i].x();
    pose.position.y = trajectory.pts[i].y();
    pose.position.z = trajectory.pts[i].z();
    msg.sampled_traj.push_back(pose);
  }

  void replanLoop()
  {
    const ros::Duration replan_period(0.5);
    while (!stop_)
    {
      if (ros::isShuttingDown())
        break;
      bool has_work = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        has_work = planning_requested_ || hold_requested_;
      }
      if (!has_work)
      {
        replan_period.sleep();
        continue;
      }

      std::lock_guard<std::mutex> lock(mutex_);
      if (hold_requested_)
      {
        hold_requested_ = false;
        current_.reset();
        publishState("HOLD");
        if (local_pose_)
        {
          publishPose(local_pose_->pose.position.x, local_pose_->pose.position.y,
                      local_pose_->pose.position.z, 0.0);
        }
        continue;
      }
      if (!planning_requested_ || !pending_goal_)
        continue;
      planning_requested_ = false;

      const ros::Time now = ros::Time::now();
      if (!local_pose_ || !local_pose_stamp_ ||
          (now - *local_pose_stamp_).toSec() > 1.0)
      {
        publishState("POSE_STALE");
        continue;
      }
      GoalPoint start;
      start.position = Eigen::Vector3d(local_pose_->pose.position.x,
                                       local_pose_->pose.position.y,
                                       local_pose_->pose.position.z);
      start.yaw = goal_yaw_;

      std::vector<NeighborTrajectory> neighbors;
      neighbors.reserve(neighbor_trajs_.size());
      for (const auto& item : neighbor_trajs_)
      {
        if ((now - item.second.second).toSec() <= kNeighborStaleS)
          neighbors.push_back(item.second.first);
      }

      TimedTrajectory trajectory;
      double cost = 0.0;
      if (!core_->planOnce(start, waypoints_, neighbors, trajectory, cost))
      {
        publishState("EGO_PLAN_FAILED");
        continue;
      }
      current_.reset(new TimedTrajectory(trajectory));
      plan_start_ = now;
      publishState("EXECUTING");
      publishIntent(trajectory);
    }
  }

  ros::NodeHandle pnh_;
  std::unique_ptr<EgoPlannerCore> core_;
  std::string frame_id_;
  std::string uav_id_;
  std::string exec_target_;

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

  std::set<std::string> neighbor_keys_;
  std::map<std::string, std::pair<NeighborTrajectory, ros::Time>> neighbor_trajs_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_planner_driver_node");
  EgoPlannerDriverNode node;
  ros::spin();
  return 0;
}