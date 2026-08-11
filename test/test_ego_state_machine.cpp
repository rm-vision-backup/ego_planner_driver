#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>

#include "ego_planner_driver/ego_planner_driver_node.h"

using ego_planner_driver::EgoPlannerDriverNode;
using ego_planner_driver::kStateCompleted;
using ego_planner_driver::kStateExecuting;
using ego_planner_driver::kStateHold;
using ego_planner_driver::kStateIdle;

namespace
{
// 可编程时钟：测试内完全控制仿真时间推进
class FakeClock
{
public:
  void set(double t) { t_ = t; }
  double get() const { return t_; }
  void advance(double dt) { t_ += dt; }
  ros::Time now() const { return ros::Time(t_); }

private:
  double t_ = 0.0;
};

geometry_msgs::PoseStamped makePose(double x, double y, double z, double t)
{
  geometry_msgs::PoseStamped p;
  p.header.stamp = ros::Time(t);
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;
  return p;
}

// 模拟理想位置控制器：把当前 setpoint 回灌为 local_pose
void echoSetpoint(EgoPlannerDriverNode& node, FakeClock& clock)
{
  const geometry_msgs::PoseStamped sp = node.lastSetpoint();
  node.injectLocalPose(makePose(sp.pose.position.x, sp.pose.position.y,
                                sp.pose.position.z, clock.get()));
}

TEST(EgoStateMachine, IdlePublishesHoverSetpointAndRefreshesPerSecond)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });

  node.injectLocalPose(makePose(1.0, 2.0, 15.0, 0.0));

  // 30Hz 发布：1 秒推进应约 30 条
  const size_t before = node.setpointCount();
  for (int i = 0; i < 30; ++i)
  {
    clock.advance(1.0 / 30.0);
    node.publishSetpointTick();
  }
  EXPECT_GE(node.setpointCount(), before + 29);
  EXPECT_EQ(node.execState(), kStateIdle);
  ASSERT_TRUE(static_cast<bool>(node.idleHoverTarget()));
  EXPECT_NEAR(node.idleHoverTarget()->x(), 1.0, 1e-6);

  // 每秒刷新：位姿移动到新位置后，setpoint 目标随之下一个周期更新
  clock.set(2.0);
  node.injectLocalPose(makePose(8.0, 9.0, 15.0, 2.0));
  node.publishSetpointTick();  // 进入刷新周期 -> 更新 idle_hover_target_
  ASSERT_TRUE(static_cast<bool>(node.idleHoverTarget()));
  EXPECT_NEAR(node.idleHoverTarget()->x(), 8.0, 1e-6);
  const geometry_msgs::PoseStamped sp = node.lastSetpoint();
  EXPECT_NEAR(sp.pose.position.x, 8.0, 1e-6);
  EXPECT_NEAR(sp.pose.position.y, 9.0, 1e-6);
}

TEST(EgoStateMachine, IdleStopsPublishingWithoutFreshPose)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });

  node.injectLocalPose(makePose(0.0, 0.0, 15.0, 0.0));
  node.publishSetpointTick();
  const size_t count = node.setpointCount();
  EXPECT_GT(count, 0u);

  // 位姿过期（> pose_timeout=1s）：fail-safe，不再发布
  clock.set(10.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.setpointCount(), count);
}

TEST(EgoStateMachine, CompletedFreezesThenRevertsToIdle)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.injectLocalPose(makePose(0.0, 0.0, 15.0, 0.0));

  node.sendGoal(5.0, 0.0, 15.0);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 模拟理想控制器 + 时钟推进直到轨迹走完并满足稳定窗口
  const double step = 1.0 / 30.0;
  for (int guard = 0; guard < 5000; ++guard)
  {
    clock.advance(step);
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    if (node.execState() == kStateCompleted)
      break;
  }
  EXPECT_EQ(node.execState(), kStateCompleted);
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 5.0, 0.5);

  // COMPLETED 后 2 秒内：继续发布冻结位姿（setpoint 流不断）
  const double frozen_x = node.lastSetpoint().pose.position.x;
  for (int i = 0; i < 30; ++i)
  {
    clock.advance(step);
    node.publishSetpointTick();
    EXPECT_NEAR(node.lastSetpoint().pose.position.x, frozen_x, 1e-3);
  }
  EXPECT_EQ(node.execState(), kStateCompleted);

  // > 2 秒：自动回 IDLE，继续发布当前位姿
  clock.advance(2.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateIdle);
  EXPECT_FALSE(static_cast<bool>(node.steadyTarget()));
  EXPECT_GT(node.setpointCount(), 0u);
}

TEST(EgoStateMachine, HoldFreezesThenRevertsToIdle)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.injectLocalPose(makePose(1.0, 1.0, 12.0, 0.0));

  node.sendHold();
  node.replanOnce();  // 处理 hold -> HOLD + 冻结当前位姿
  EXPECT_EQ(node.execState(), kStateHold);
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 1.0, 1e-6);
  EXPECT_NEAR(node.steadyTarget()->pos.y(), 1.0, 1e-6);

  // HOLD 后 2 秒内持续发布冻结位姿
  const double step = 1.0 / 30.0;
  for (int i = 0; i < 30; ++i)
  {
    clock.advance(step);
    node.publishSetpointTick();
    EXPECT_NEAR(node.lastSetpoint().pose.position.x, 1.0, 1e-3);
  }
  EXPECT_EQ(node.execState(), kStateHold);

  // > 2 秒：自动回 IDLE
  clock.advance(2.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateIdle);
  EXPECT_FALSE(static_cast<bool>(node.steadyTarget()));
}
}  // namespace

// 启动隔离 roscore，保证节点 advertise/subscribe/timer 可用
// （catkin_add_gtest 不自动启动 roscore）
int main(int argc, char** argv)
{
  setenv("ROS_MASTER_URI", "http://localhost:11411", 1);
  setenv("ROS_HOSTNAME", "localhost", 1);

  pid_t pid = fork();
  if (pid == 0)
  {
    execlp("roscore", "roscore", "--port", "11411", (char*)nullptr);
    _exit(127);
  }

  ros::init(argc, argv, "ego_state_machine_test");
  for (int i = 0; i < 200; ++i)
  {
    if (ros::master::check())
      break;
    usleep(100 * 1000);
  }

  testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();

  if (pid > 0)
  {
    kill(pid, SIGTERM);
    waitpid(pid, nullptr, 0);
  }
  return rc;
}
