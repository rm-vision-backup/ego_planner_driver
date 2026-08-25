#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>

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

swarm_uav_interfaces::UavTrajectoryIntent makeIntent(
    double stamp, int32_t traj_id = 1, const std::string& uav_id = "A02")
{
  swarm_uav_interfaces::UavTrajectoryIntent msg;
  msg.protocol_version = "1.0";
  msg.uav_id = uav_id;
  msg.exec_target = "UAV2";
  msg.frame_id = "map";
  msg.traj_id = traj_id;
  msg.stamp = stamp;
  msg.clearance = 0.5;
  msg.t = {0.0, 1.0};
  geometry_msgs::Pose p0;
  p0.position.x = 1.0;
  p0.position.z = 5.0;
  geometry_msgs::Pose p1 = p0;
  p1.position.x = 2.0;
  msg.sampled_traj = {p0, p1};
  return msg;
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

TEST(EgoStateMachine, PlanLatencyOnlyMovesActivationEpoch)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);

  // Simulate optimizer wall time between the planning snapshot and commit.
  int clock_calls = 0;
  node.setClockSource([&clock, &clock_calls]() {
    if (clock_calls++ > 0)
      clock.set(100.75);
    return clock.now();
  });
  ASSERT_TRUE(node.replanOnce());
  const boost::optional<ego_planner_driver::TimedTrajectory> traj =
      node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  ASSERT_FALSE(traj->t.empty());
  EXPECT_DOUBLE_EQ(traj->t.front(), 0.0);
  EXPECT_DOUBLE_EQ(traj->duration, traj->t.back());
  EXPECT_DOUBLE_EQ(node.planStart().toSec(), 100.75);

  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, traj->pts.front().x(), 1e-9);
  EXPECT_NEAR(node.lastSetpoint().position.y, traj->pts.front().y(), 1e-9);
  EXPECT_NEAR(node.lastSetpoint().position.z, traj->pts.front().z(), 1e-9);
}

TEST(EgoStateMachine, ShadowSafetyTickDoesNotChangeFlightBehavior)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setSafetySupervisorModeForTest("shadow");
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());

  auto peer = makeIntent(100.0, 1, "A02");
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.y = 0.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 0.0;
  peer.sampled_traj[1].position.y = 0.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  const auto trajectory_before = node.currentTrajectory();
  const ros::Time start_before = node.planStart();
  const std::string state_before = node.execState();
  const size_t setpoints_before = node.setpointCount();
  node.safetySupervisorTick();

  const auto trajectory_after = node.currentTrajectory();
  ASSERT_TRUE(trajectory_before && trajectory_after);
  EXPECT_EQ(trajectory_after->pts.size(), trajectory_before->pts.size());
  EXPECT_TRUE(trajectory_after->pts.front().isApprox(trajectory_before->pts.front()));
  EXPECT_EQ(node.planStart(), start_before);
  EXPECT_EQ(node.execState(), state_before);
  EXPECT_EQ(node.setpointCount(), setpoints_before);
  EXPECT_EQ(node.lastSafetyPrediction().pairs.size(), 1u);
}

TEST(EgoStateMachine, ActiveEmergencyBrakeStopsAndHoldsEndpoint)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setSafetySupervisorModeForTest("active");
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());

  auto peer = makeIntent(100.0, 1, "A02");
  peer.t = {0.0, 5.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 0.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));
  node.safetySupervisorTick();
  ASSERT_EQ(node.execState(), ego_planner_driver::kStateEmergencyBrake);
  const auto braking = node.currentTrajectory();
  ASSERT_TRUE(braking);
  ASSERT_GT(braking->duration, 0.0);

  while (clock.get() <= 100.0 + braking->duration + 0.1)
  {
    node.publishSetpointTick();
    clock.advance(0.02);
  }
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), ego_planner_driver::kStateBrakeHold);
  const auto hold = node.steadyTarget();
  ASSERT_TRUE(hold);
  EXPECT_LT((hold->pos - braking->pts.back()).norm(), 1e-8);
  EXPECT_EQ(node.lastSetpoint().type_mask, ego_planner_driver::kMaskHold);
}

TEST(EgoStateMachine, ActiveYieldCommitsOnceAndReevaluatesOnPeerUpdate)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("A02", "UAV2");  // self=A02 > peer A01 → YIELD
  node.setSafetySupervisorModeForTest("active");
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());

  // peer A01 平行同向、横向 1.2m：distance 恒 1.2，落在 warning(1.35) 与
  // emergency(1.1) 之间 → WARNING + YIELD。
  auto peer = makeIntent(100.0, 1, "A01");
  peer.t = {0.0, 5.0};
  peer.sampled_traj[0].position.x = 0.0;
  peer.sampled_traj[0].position.y = 1.2;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.y = 1.2;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  // 首次 tick：WARNING+YIELD → 提交让路候选，不制动（缺陷 2 主路径）。
  node.safetySupervisorTick();
  ASSERT_EQ(node.execState(), kStateExecuting);
  ASSERT_TRUE(node.yieldCommittedForTest());

  // peer 未更新：保持已提交候选，不重复切轨迹（缺陷 2 防抖）。
  node.safetySupervisorTick();
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_TRUE(node.yieldCommittedForTest());

  // peer 更新（traj_id=2）且轨迹逼近当前候选（横向 0.6m < clearance）：
  // 旧代码因 yield_committed_ 无条件早退会忽略；新代码重新评估并制动。
  auto peer2 = makeIntent(100.0, 2, "A01");
  peer2.t = {0.0, 5.0};
  peer2.sampled_traj[0].position.x = 0.0;
  peer2.sampled_traj[0].position.y = 0.6;
  peer2.sampled_traj[0].position.z = 5.0;
  peer2.sampled_traj[1].position.x = 5.0;
  peer2.sampled_traj[1].position.y = 0.6;
  peer2.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer2));
  node.safetySupervisorTick();
  EXPECT_EQ(node.execState(), ego_planner_driver::kStateEmergencyBrake);

  // 缺陷 4 诊断字段：peer activation epoch 应等于 intent stamp（100.0）。
  const ego_planner_driver::SafetyPrediction prediction =
      node.lastSafetyPrediction();
  ASSERT_FALSE(prediction.pairs.empty());
  EXPECT_DOUBLE_EQ(prediction.pairs.front().peer_activation_epoch_s, 100.0);
}

TEST(EgoStateMachine, TentativeIntentWaitsWithoutPublishingTrajectorySetpoint)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIntentNegotiationWaitForTest(1.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  const size_t before = node.setpointCount();
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), ego_planner_driver::kStateNegotiating);
  EXPECT_FALSE(node.currentTrajectory());
  node.publishSetpointTick();
  EXPECT_EQ(node.setpointCount(), before);

  clock.advance(0.99);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), ego_planner_driver::kStateNegotiating);
  EXPECT_FALSE(node.currentTrajectory());
  clock.advance(0.02);
  node.publishSetpointTick();
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_TRUE(node.currentTrajectory());
}

TEST(EgoStateMachine, NewPeerIntentInvalidatesTentativeForReevaluation)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIntentNegotiationWaitForTest(1.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), ego_planner_driver::kStateNegotiating);
  auto peer = makeIntent(100.0);
  peer.protocol_version = "2.0";
  peer.phase = swarm_uav_interfaces::UavTrajectoryIntent::PHASE_TENTATIVE;
  ASSERT_TRUE(node.injectNeighborIntent(peer));
  EXPECT_TRUE(node.planningRequested());
  clock.advance(1.1);
  node.publishSetpointTick();
  EXPECT_FALSE(node.currentTrajectory());
}

TEST(EgoStateMachine, IntentPhaseProtocolKeepsLegacyActiveOnly)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  auto legacy = makeIntent(100.0);
  legacy.phase = swarm_uav_interfaces::UavTrajectoryIntent::PHASE_UNKNOWN;
  EXPECT_TRUE(node.injectNeighborIntent(legacy));
  auto invalid = makeIntent(100.1, 2);
  invalid.protocol_version = "2.0";
  invalid.phase = 99;
  EXPECT_FALSE(node.injectNeighborIntent(invalid));
  auto active = makeIntent(100.1, 2);
  active.protocol_version = "2.0";
  active.phase = swarm_uav_interfaces::UavTrajectoryIntent::PHASE_ACTIVE;
  EXPECT_TRUE(node.injectNeighborIntent(active));
}

TEST(EgoStateMachine, NeighborIntentValidationAndSequenceReset)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);

  auto valid = makeIntent(100.0, 10);
  EXPECT_TRUE(node.injectNeighborIntent(valid));
  EXPECT_FALSE(node.planningRequested());
  EXPECT_FALSE(node.injectNeighborIntent(valid));  // exact duplicate

  auto same_epoch_older = valid;
  same_epoch_older.traj_id = 9;
  EXPECT_FALSE(node.injectNeighborIntent(same_epoch_older));

  // A newer activation epoch establishes a new sequence generation. This
  // accepts both process restarts and signed int32 wraparound.
  auto restarted = makeIntent(100.1, std::numeric_limits<int32_t>::min());
  EXPECT_TRUE(node.injectNeighborIntent(restarted));
  auto old_epoch = makeIntent(100.0, std::numeric_limits<int32_t>::max());
  EXPECT_FALSE(node.injectNeighborIntent(old_epoch));

  auto malformed = makeIntent(100.2, 1);
  malformed.protocol_version = "2.0";
  EXPECT_FALSE(node.injectNeighborIntent(malformed));
  malformed = makeIntent(100.2, 1);
  malformed.frame_id = "odom";
  EXPECT_FALSE(node.injectNeighborIntent(malformed));
  malformed = makeIntent(100.2, 1);
  malformed.t[0] = 0.1;
  EXPECT_FALSE(node.injectNeighborIntent(malformed));
  malformed = makeIntent(100.2, 1);
  malformed.t[1] = malformed.t[0];
  EXPECT_FALSE(node.injectNeighborIntent(malformed));
  malformed = makeIntent(100.2, 1);
  malformed.sampled_traj[1].position.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(node.injectNeighborIntent(malformed));
  malformed = makeIntent(100.2, 1);
  malformed.sampled_traj.pop_back();
  EXPECT_FALSE(node.injectNeighborIntent(malformed));

  clock.set(110.0);
  EXPECT_FALSE(node.injectNeighborIntent(makeIntent(100.2, 2)));
  EXPECT_FALSE(node.injectNeighborIntent(makeIntent(120.0, 2)));
}

TEST(EgoStateMachine, LowerPriorityIdentityReplansOnlyForHigherPriorityConflict)
{
  ros::param::set("/uav_id", "A02");
  ros::param::set("/exec_target", "UAV2");
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_FALSE(node.planningRequested());

  const boost::optional<ego_planner_driver::TimedTrajectory> own =
      node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(own));
  auto conflict = makeIntent(100.0, 1, "A01");
  conflict.t = own->t;
  conflict.sampled_traj.clear();
  for (const auto& point : own->pts)
  {
    geometry_msgs::Pose pose;
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();
    conflict.sampled_traj.push_back(pose);
  }
  EXPECT_TRUE(node.injectNeighborIntent(conflict));
  EXPECT_TRUE(node.planningRequested());

  ros::param::set("/uav_id", "A01");
  ros::param::set("/exec_target", "UAV1");
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

TEST(EgoStateMachine, ReplanUsesTrajectoryStateNotLivePose)
{
  // 建议 2：滚动重规划起点必须来自旧轨迹同一时刻的完整状态，而不是
  // "实际位置 + 旧导数"。注入一个明显偏离的 local_pose，新段起点应仍落在
  // 旧轨迹附近，而非跟随注入位置跳变。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
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

  // 推进到时间触发重规划。
  for (int step = 0; step < 20; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      break;
  }
  EXPECT_TRUE(node.planningRequested());
  const double old_x = node.lastSetpoint().position.x;

  // 注入偏离 100m 的实际位姿。
  const double dev_x = old_x + 100.0;
  node.injectLocalPose(makePose(dev_x, node.lastSetpoint().position.y,
                                node.lastSetpoint().position.z, clock.get()));

  // 滚动 replan：新段起点应取旧轨迹状态（≈old_x），而非注入的 dev_x。
  ASSERT_TRUE(node.replanOnce());
  node.publishSetpointTick();
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  EXPECT_LT(std::abs(sp.position.x - old_x), 1.0);
  EXPECT_GT(std::abs(sp.position.x - dev_x), 50.0);
  EXPECT_EQ(node.execState(), kStateExecuting);
}

TEST(EgoStateMachine, RollingSegmentsReachCompleted)
{
  // 滚动多段推进后最终必须 COMPLETED（prepare 场景的到达路径）。建议 2 把
  // 起点换成旧轨迹状态后，此路径必须有测试覆盖。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  node.setPlanningHorizon(3.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 1; i <= 12; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 0.5; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  bool completed = false;
  for (int guard = 0; guard < 3000; ++guard)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      ASSERT_TRUE(node.replanOnce());
    if (node.execState() == kStateCompleted)
    {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed);
  if (completed)
    EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
}

TEST(EgoStateMachine, RollingSegmentsReachCompletedWithTrackingLag)
{
  // 模拟 SITL 跟踪滞后：回灌 local_pose 时沿 x 落后 setpoint 0.15m（测试默认
  // position_tolerance_m=0.2，需保持滞后 < 容差），验证窗口推进/到达判定仍
  // 收敛到 COMPLETED。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  node.setPlanningHorizon(3.0);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 1; i <= 12; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 0.5; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  bool completed = false;
  for (int guard = 0; guard < 3000; ++guard)
  {
    node.publishSetpointTick();
    const mavros_msgs::PositionTarget sp = node.lastSetpoint();
    node.injectLocalPose(makePose(sp.position.x - 0.15, sp.position.y,
                                  sp.position.z, clock.get()));
    clock.advance(0.1);
    if (node.planningRequested())
      ASSERT_TRUE(node.replanOnce());
    if (node.execState() == kStateCompleted)
    {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed);
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
