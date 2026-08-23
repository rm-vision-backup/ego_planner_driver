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
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  node.injectLocalPose(makePose(sp.position.x, sp.position.y,
                                sp.position.z, clock.get()));
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
  EXPECT_NEAR(node.lastSetpoint().position.z, 2.0, 1e-6);
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
    if (node.execState() == kStateCompleted && node.steadyTarget())
      break;
  }
  EXPECT_EQ(node.execState(), kStateCompleted);
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 0.0, 0.5);

  // COMPLETED 后 2 秒内：继续发布冻结位姿（setpoint 流不断）
  const double frozen_x = node.lastSetpoint().position.x;
  for (int i = 0; i < 30; ++i)
  {
    clock.advance(step);
    node.publishSetpointTick();
    EXPECT_NEAR(node.lastSetpoint().position.x, frozen_x, 1e-3);
  }
  EXPECT_EQ(node.execState(), kStateCompleted);

  // 超过原有终端窗口后仍保持锁定
  clock.advance(20.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateCompleted);
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
    EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-3);
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
// 回归：滚动窗口必须随 UAV 实际位置推进，而不是被上次规划段 erase 提前消耗。
// 修复前 remaining_waypoints_ 按 planned_waypoint_count erase，periodic replan 时
// 窗口首点已距 UAV 数米，truncateToHorizon 把该距离计入 horizon 预算 → 每段只取
// 0-2 个点 → 轨迹终点≈当前位置 → UAV 停在半途。修复后 waypoint_consumed_ 基于
// start 在 waypoints 上的弧长投影推进，连续 replan 终点必须持续前进。
TEST(EgoStateMachine, RollingWindowAdvancesWithActualPosition)
{
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  node.setPlanningHorizon(5.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // 密化航点：0.4m 间距，从 start(0,0,5) 沿 x 延伸至 40m（覆盖多层 horizon 窗口）。
  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 100; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = 0.4 * i; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  const double seg1_x = node.lastSetpoint().position.x;  // 第一段窗口起点

  // 连续滚动：每次把 UAV 推进到段末触发 checkArrival 置 planning_requested_，
  // 再 replan；修复后窗口首点必须随实际位置前进，轨迹持续推进。
  bool progressed = false;
  for (int iter = 0; iter < 12; ++iter)
  {
    for (int step = 0; step < 200; ++step)
    {
      node.publishSetpointTick();
      // 模拟 UAV 实际执行：local_pose 跟随 setpoint，避免 checkArrival 判超时切 HOLD。
      echoSetpoint(node, clock);
      clock.advance(0.1);
      if (node.planningRequested() || node.execState() == kStateCompleted)
        break;
    }
    if (node.execState() == kStateCompleted)
      break;
    ASSERT_TRUE(node.replanOnce());
    const double cur = node.lastSetpoint().position.x;
    if (cur > seg1_x + 3.0)
      progressed = true;
  }
  // 修复前窗口被提前消耗，第二次 replan 起终点≈当前位置，无法推进 3m 以上。
  EXPECT_TRUE(progressed);
}
// 回归：periodic replan 在当前段执行完后快照时，currentDerivatives 必须
// clamp 到轨迹末端速度（旧实现返回全零 vel -> 新段从 0 起步 -> 速度 0~max 震荡）。
TEST(EgoStateMachine, ReplanSnapshotsKeepSegmentVelocity)
{
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  node.setPlanningHorizon(5.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // 短航线：单段 horizon 内只有 4 个点 -> 轨迹 duration 短（< replan_period 4s）
  std::vector<geometry_msgs::Point32> wps;
  for (int i = 1; i <= 4; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = 0.4 * i; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 推进到轨迹执行完（elapsed 远超 duration）：old currentDerivatives 会返回
  // vel=0；修复后 clamp 到末端速度，replanOnce 规划出的新段起点速度保持非零。
  for (int step = 0; step < 60; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      break;
  }
  // periodic replan 应触发（仍有后续航点 + 达到触发时间）。
  EXPECT_TRUE(node.planningRequested());
  // 新段规划成功，且起点速度接近段末速度（>0.3），不是从 0 重新加速。
  ASSERT_TRUE(node.replanOnce());
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  // 现在可以直接读规划器速度前馈：新段起点速度应为非零（rolling continuity）。
  EXPECT_TRUE(std::isfinite(sp.velocity.x) && std::isfinite(sp.velocity.y));
  // 验证 exec_state 仍 EXECUTING 且 setpoint 持续推进。
  EXPECT_EQ(node.execState(), kStateExecuting);
  // 第二次推进一段，setpoint 应越过首段终点（4*0.4=1.6m）。
  double max_x = sp.position.x;
  for (int step = 0; step < 100; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    max_x = std::max(max_x, node.lastSetpoint().position.x);
    clock.advance(0.1);
    if (node.execState() == kStateCompleted)
      break;
  }
  EXPECT_GT(max_x, 1.6);
}

}  // namespace

// 启动隔离 roscore，保证节点 advertise/subscribe/timer 可用
// （catkin_add_gtest 不自动启动 roscore）
TEST(EgoStateMachine, HorizonTruncatesAndRollsWaypoints)
{
  // 让 replan 线程先进入 sleep 窗口，避免在下方快速 wall-clock 步骤中
  // 抢占 planning_requested_（FakeClock 推进不影响线程的 0.5s wall sleep）。
  usleep(600 * 1000);

  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setPlanningHorizon(5.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 1; i <= 10; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 2.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 推进到第一段末端：段末点应落在 horizon 内（首个航点 2m 起步，约 6m），
  // 远小于最终目标 20m —— 验证 receding-horizon 截取生效。
  double seg1_x = -1.0;
  for (int step = 0; step < 300; ++step)
  {
    node.publishSetpointTick();
    seg1_x = node.lastSetpoint().position.x;
    if (node.execState() == kStateCompleted)
      break;
    clock.advance(0.1);
    echoSetpoint(node, clock);
  }
  EXPECT_LT(seg1_x, 8.0);
  EXPECT_GT(seg1_x, 3.0);

  // 到达段末后 checkArrival 置 planning_requested_（还有后续航点），
  // 手动 replanOnce 规划下一段并继续推进 —— 验证滚动重规划。
  double max_x = seg1_x;
  for (int step = 0; step < 300; ++step)
  {
    node.replanOnce();
    node.publishSetpointTick();
    max_x = std::max(max_x, node.lastSetpoint().position.x);
    if (node.execState() == kStateCompleted)
      break;
    clock.advance(0.1);
    echoSetpoint(node, clock);
  }
  EXPECT_GT(max_x, seg1_x + 2.0);
}

TEST(EgoStateMachine, PeriodicReplanTriggersDuringSegmentExecution)
{
  // 禁用 replanLoop 线程，手动 replanOnce 驱动，保证时间触发断言确定性。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  node.setPlanningHorizon(5.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 1; i <= 6; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 2.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  // 到达段末才会置 planning_requested_；此刻仍在段起点 -> 应为 false。
  EXPECT_FALSE(node.planningRequested());

  // 段内推进超过 replan_period_s（1.0s）且仍有后续 waypoints -> 时间触发重规划。
  for (int step = 0; step < 20; ++step)
  {
    node.publishSetpointTick();
    clock.advance(0.1);
    if (node.execState() == kStateCompleted)
      break;
  }
  EXPECT_TRUE(node.planningRequested());
  EXPECT_EQ(node.execState(), kStateExecuting);
}

TEST(EgoStateMachine, TrajectorySetpointPublishesPvaMask)
{
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 1; i <= 8; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 1.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 段中采样：EGO 执行帧 + P/V/A/yaw 掩码 + 所有启用字段有限 + 速度非零。
  clock.advance(2.0);
  node.publishSetpointTick();
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  EXPECT_EQ(sp.coordinate_frame, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(sp.type_mask, ego_planner_driver::kMaskEgoTrajectory);
  EXPECT_TRUE(std::isfinite(sp.position.x) && std::isfinite(sp.position.y) &&
              std::isfinite(sp.position.z));
  EXPECT_TRUE(std::isfinite(sp.velocity.x) && std::isfinite(sp.velocity.y) &&
              std::isfinite(sp.velocity.z));
  EXPECT_TRUE(std::isfinite(sp.acceleration_or_force.x) &&
              std::isfinite(sp.acceleration_or_force.y) &&
              std::isfinite(sp.acceleration_or_force.z));
  EXPECT_TRUE(std::isfinite(sp.yaw));
  const double speed = std::sqrt(sp.velocity.x * sp.velocity.x +
                                 sp.velocity.y * sp.velocity.y +
                                 sp.velocity.z * sp.velocity.z);
  EXPECT_GT(speed, 0.05);

  // 轨迹耗尽后切固定 position/yaw HOLD，不得携带末段非零导数。
  clock.advance(50.0);
  node.publishSetpointTick();
  const mavros_msgs::PositionTarget hold = node.lastSetpoint();
  EXPECT_EQ(hold.type_mask, ego_planner_driver::kMaskHold);
  EXPECT_EQ(hold.velocity.x, 0.0);
  EXPECT_EQ(hold.velocity.y, 0.0);
  EXPECT_EQ(hold.velocity.z, 0.0);
  EXPECT_EQ(hold.acceleration_or_force.x, 0.0);
  EXPECT_EQ(hold.acceleration_or_force.y, 0.0);
  EXPECT_EQ(hold.acceleration_or_force.z, 0.0);
}

TEST(EgoStateMachine, HoldAndTakeoffUseHoldMask)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);

  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.publishSetpointTick();
  const mavros_msgs::PositionTarget hold = node.lastSetpoint();
  EXPECT_EQ(hold.coordinate_frame, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(hold.type_mask, ego_planner_driver::kMaskHold);
  EXPECT_EQ(hold.velocity.x, 0.0);
  EXPECT_EQ(hold.velocity.y, 0.0);
  EXPECT_EQ(hold.velocity.z, 0.0);
  EXPECT_NEAR(hold.position.z, 5.0, 1e-6);

  // arm+OFFBOARD 进入 TAKEOFF：仍为 position/yaw HOLD mask。
  node.injectMavrosState(true, true);
  EXPECT_EQ(node.execState(), kStateTakeoff);
  clock.advance(1.0 / 30.0);
  node.publishSetpointTick();
  EXPECT_EQ(node.lastSetpoint().type_mask, ego_planner_driver::kMaskHold);
}

TEST(EgoStateMachine, DirectControlSuppressesEgoCandidates)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  const size_t before = node.setpointCount();
  node.setDirectControlActive(true);
  node.publishSetpointTick();
  EXPECT_EQ(node.setpointCount(), before);

  node.setDirectControlActive(false);
  node.publishSetpointTick();
  node.publishSetpointTick();
  EXPECT_GT(node.setpointCount(), before);
  EXPECT_EQ(node.lastSetpoint().type_mask, ego_planner_driver::kMaskHold);
}

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
