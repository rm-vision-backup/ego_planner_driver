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
using ego_planner_driver::kStateExecuting;
using ego_planner_driver::kStateTakeoff;
using ego_planner_driver::kStateHold;

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

// executor/Python 驱动负责起飞；C++ 节点只需处于可规划的 HOLD。
bool setupHold(EgoPlannerDriverNode& node, FakeClock& clock, double start_z)
{
  node.injectLocalPose(makePose(0.0, 0.0, start_z, clock.get()));
  node.injectMavrosState(false, false);
  return node.execState() == kStateHold;
}

TEST(EgoStateMachine, InitialStateIsHold)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  EXPECT_EQ(node.execState(), kStateHold);
}

TEST(EgoStateMachine, ArmOffboardEdgeStartsTakeoff)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.injectLocalPose(makePose(0.0, 0.0, 2.0, 0.0));

  // 未 arm+OFFBOARD 时保持 HOLD
  node.injectMavrosState(false, false);
  EXPECT_EQ(node.execState(), kStateHold);

  // arm/OFFBOARD 上升沿进入 TAKEOFF。
  node.injectMavrosState(true, true);
  EXPECT_EQ(node.execState(), kStateTakeoff);

  // 下降沿（如手控接管丢失 OFFBOARD）：保持 HOLD
  node.injectMavrosState(true, false);
  EXPECT_EQ(node.execState(), kStateHold);
}

TEST(EgoStateMachine, TakeoffPublishesHoldSetpoint)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.injectLocalPose(makePose(0.0, 0.0, 2.0, 0.0));

  node.injectMavrosState(true, true);
  EXPECT_EQ(node.execState(), kStateTakeoff);
  clock.advance(1.0 / 30.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateTakeoff);
  EXPECT_NEAR(node.lastSetpoint().pose.position.z, 2.0, 1e-6);
}

TEST(EgoStateMachine, RepeatedArmEdgesRemainHold)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });

  ASSERT_TRUE(setupHold(node, clock, 2.0));
  EXPECT_EQ(node.execState(), kStateHold);

  // 手控落地：arm/OFFBOARD 丢失 -> 保持 HOLD
  node.injectMavrosState(true, false);
  EXPECT_EQ(node.execState(), kStateHold);
  node.injectMavrosState(false, false);
  EXPECT_EQ(node.execState(), kStateHold);

  // 再次 arm+OFFBOARD 也不会触发旧软起飞。
  node.injectMavrosState(true, true);
  EXPECT_EQ(node.execState(), kStateTakeoff);
}

TEST(EgoStateMachine, PublishSetpointDoesNotReclaimWhileGoalPending)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });

  ASSERT_TRUE(setupHold(node, clock, 5.0));
  EXPECT_EQ(node.execState(), kStateHold);
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));

  // 收到新目标：onGoal 的 resetRun 清除 steady_target_ 并请求规划。
  node.sendGoal(10.0, 0.0, 7.0);
  EXPECT_FALSE(static_cast<bool>(node.steadyTarget()));

  // 30Hz publishSetpoint 不得在 replan 前重新捕获 steady_target_，
  // 否则 replanLoop 会因 steady_target_ 非空而永远跳过规划（真实竞态：
  // 空闲 HOLD 捕获 -> onGoal 清除 -> publishSetpoint 重新捕获 -> 永不规划）。
  node.publishSetpointTick();
  EXPECT_FALSE(static_cast<bool>(node.steadyTarget()));

  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
}

TEST(EgoStateMachine, CompletedFreezesInHold)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });

  ASSERT_TRUE(setupHold(node, clock, 15.0));
  EXPECT_EQ(node.execState(), kStateHold);

  node.sendGoal(0.0, 0.0, 15.0);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 模拟理想控制器 + 时钟推进直到轨迹走完并满足稳定窗口
  const double step = 1.0 / 30.0;
  for (int guard = 0; guard < 5000; ++guard)
  {
    clock.advance(step);
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    if (node.execState() == kStateHold && node.steadyTarget())
      break;
  }
  EXPECT_EQ(node.execState(), kStateHold);
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 0.0, 0.5);

  // COMPLETED 后 2 秒内：继续发布冻结位姿（setpoint 流不断）
  const double frozen_x = node.lastSetpoint().pose.position.x;
  for (int i = 0; i < 30; ++i)
  {
    clock.advance(step);
    node.publishSetpointTick();
    EXPECT_NEAR(node.lastSetpoint().pose.position.x, frozen_x, 1e-3);
  }
  EXPECT_EQ(node.execState(), kStateHold);

  // 超过原有终端窗口后仍保持锁定
  clock.advance(20.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateHold);
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_GT(node.setpointCount(), 0u);
}

TEST(EgoStateMachine, HoldRemainsFixed)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });

  ASSERT_TRUE(setupHold(node, clock, 12.0));
  EXPECT_EQ(node.execState(), kStateHold);

  node.sendHold();
  node.replanOnce();  // 处理 hold -> HOLD + 冻结当前位姿
  EXPECT_EQ(node.execState(), kStateHold);
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 0.0, 1e-6);
  EXPECT_NEAR(node.steadyTarget()->pos.y(), 0.0, 1e-6);

  // HOLD 后 2 秒内持续发布冻结位姿
  const double step = 1.0 / 30.0;
  for (int i = 0; i < 30; ++i)
  {
    clock.advance(step);
    node.publishSetpointTick();
    EXPECT_NEAR(node.lastSetpoint().pose.position.x, 0.0, 1e-3);
  }
  EXPECT_EQ(node.execState(), kStateHold);

  // > 2 秒：自动回 IDLE
  clock.advance(20.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateHold);
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
}

TEST(EgoStateMachine, ArmOffboardDoesNotBlockReplan)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.injectLocalPose(makePose(0.0, 0.0, 2.0, 0.0));
  node.injectMavrosState(true, true);
  EXPECT_EQ(node.execState(), kStateTakeoff);
  node.sendGoal(10.0, 0.0, 7.0);
  EXPECT_FALSE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateTakeoff);
  EXPECT_EQ(node.execState(), kStateTakeoff);

  node.sendGoal(10.0, 0.0, 7.0);
  EXPECT_FALSE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateTakeoff);
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
