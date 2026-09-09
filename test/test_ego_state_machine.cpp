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
    double stamp, int32_t traj_id = 1, const std::string& uav_id = "UAV2")
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

  auto peer = makeIntent(100.0, 1, "UAV2");
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

TEST(EgoStateMachine, EmergencyPredictionDirectLocksBrakeHold)
{
  // implementation_plan_26090900 §0：监督层 EMERGENCY（预测 min_d<=1.0 或
  // ttc<=1.5）只置 emergency_pending_；replanOnce 先尝试常规 replan（同轨迹
  // peer → 必然失败）与让路候选（默认关闭）→ 绕开失败 → 直接 BRAKE_HOLD 锁存，
  // 不再生成制动轨迹、无 EMERGENCY_BRAKE 中间态。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setSafetySupervisorModeForTest("active");
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), kStateExecuting);

  auto peer = makeIntent(100.0, 1, "UAV2");
  peer.t = {0.0, 5.0};
  peer.sampled_traj[0].position.x = 0.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));
  node.safetySupervisorTick();
  // 监督回调不直接改轨迹/锁存：先保持 EXECUTING，由 replanOnce 统一处理。
  ASSERT_EQ(node.execState(), kStateExecuting);
  ASSERT_TRUE(node.replanOnce());
  // 无 EMERGENCY_BRAKE 中间态：直接 BRAKE_HOLD。
  ASSERT_EQ(node.execState(), ego_planner_driver::kStateBrakeHold);
  const auto hold = node.steadyTarget();
  ASSERT_TRUE(hold);
  // 锁存位置来自本机当前有效位置（fresh pose (0,0,5)），非 runtime 目标/邻机点。
  EXPECT_NEAR(hold->pos.x(), 0.0, 1e-6);
  EXPECT_NEAR(hold->pos.y(), 0.0, 1e-6);
  EXPECT_NEAR(hold->pos.z(), 5.0, 1e-6);
  // 普通轨迹已清除；setpoint 为固定位置 HOLD mask。
  EXPECT_FALSE(static_cast<bool>(node.currentTrajectory()));
  node.publishSetpointTick();
  EXPECT_EQ(node.lastSetpoint().type_mask, ego_planner_driver::kMaskHold);
}

TEST(EgoStateMachine, DirectBrakeHoldRequestLocksAtCurrentPose)
{
  // implementation_plan_26090900 §4.4：runtime 指定 HOLD / 组级安全 HOLD 经
  // executor → driver → /brake_hold 到达。replanOnce 在顶部门禁处理：捕获本机
  // 当前位置锁存 BRAKE_HOLD，后续 replan 不得覆盖（steady_target_ 拦截）。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), kStateExecuting);

  node.sendBrakeHold();
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), ego_planner_driver::kStateBrakeHold);
  const auto hold = node.steadyTarget();
  ASSERT_TRUE(hold);
  EXPECT_NEAR(hold->pos.x(), 0.0, 1e-6);
  EXPECT_NEAR(hold->pos.z(), 5.0, 1e-6);
  EXPECT_FALSE(static_cast<bool>(node.currentTrajectory()));
  // 已锁存后后续 replanOnce 不覆盖（steady_target_ 拦截返回 false）。
  EXPECT_FALSE(node.replanOnce());
  EXPECT_EQ(node.execState(), ego_planner_driver::kStateBrakeHold);
}

TEST(EgoStateMachine, FirstPlanReadyForLeaderImmediately)
{
  // SEQUENTIAL_START：leader(id<=1) 首轮规划立即放行。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setIdentityForTest("UAV1", "UAV1");
  node.setClockSource([&clock]() { return clock.now(); });
  EXPECT_TRUE(node.firstPlanReadyForTest());
}

TEST(EgoStateMachine, FirstPlanWaitsForPredecessorIntent)
{
  // SEQUENTIAL_START：非 leader（UAV2）首轮规划等待前驱（UAV1）intent。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setIdentityForTest("UAV2", "UAV2");
  node.setClockSource([&clock]() { return clock.now(); });
  EXPECT_FALSE(node.firstPlanReadyForTest());

  // 前驱 UAV1 intent 到达 → 放行。
  auto prev = makeIntent(100.0, 1, "UAV1");
  prev.t = {0.0, 5.0};
  prev.sampled_traj[0].position.x = 0.0;
  prev.sampled_traj[0].position.z = 5.0;
  prev.sampled_traj[1].position.x = 5.0;
  prev.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(prev));
  EXPECT_TRUE(node.firstPlanReadyForTest());
}

TEST(EgoStateMachine, FirstPlanTimeoutReleasesWithoutPredecessor)
{
  // SEQUENTIAL_START 超时兜底：前驱缺失时不永久阻塞首轮规划。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setIdentityForTest("UAV2", "UAV2");
  node.setClockSource([&clock]() { return clock.now(); });
  node.setFirstPlanWaitTimeoutForTest(0.0);
  EXPECT_TRUE(node.firstPlanReadyForTest());
}

TEST(EgoStateMachine, EmergencyReplanFirstMitigatesCollision)
{
  // implementation_plan_26090900 §0：EMERGENCY 先尝试常规重规划/让路绕开，
  // 成功提交则清除 emergency_pending_ 继续执行（不锁存）；绕开失败才直接
  // 锁存 BRAKE_HOLD（无 EMERGENCY_BRAKE 中间态）。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setSafetySupervisorModeForTest("active");
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), kStateExecuting);

  // 对向交叉 peer：与本机直线轨迹在 x=2.5 交叉（<1m，预测 EMERGENCY）。
  // 监督回调只置位；replanOnce 统一处理：常规绕开成功 → 继续执行，失败 →
  // 直接锁存 BRAKE_HOLD（不再收口 PLAN_FAILED / 无 EMERGENCY_BRAKE）。
  auto peer = makeIntent(100.0, 1, "UAV2");
  peer.t = {0.0, 5.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 0.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));
  node.safetySupervisorTick();
  // 监督回调只置位；replanOnce 统一处理（可能成功绕开，也可能直接 BRAKE_HOLD）。
  ASSERT_EQ(node.execState(), kStateExecuting);
  node.replanOnce();
  // 重规划成功（清除紧急标志继续执行）或直接 BRAKE_HOLD；不出现 PLAN_FAILED
  // 与 EMERGENCY_BRAKE 中间态。
  EXPECT_NE(node.execState(), ego_planner_driver::kStatePlanFailed);
  EXPECT_TRUE(node.execState() == kStateExecuting ||
              node.execState() == ego_planner_driver::kStateBrakeHold);
}

TEST(EgoStateMachine, YieldHighIdCommitsLateralCandidate)
{
  // 本机 UAV2（ID 2 > peer UAV1 1）→ YIELD：collisionCheckTick 只置 planning_
  // requested_（单一入口）；replanOnce 常规规划失败后由 tryYieldReplan 生成
  // LEFT/RIGHT 候选并提交（exec_state 保持 EXECUTING，冲突区横向偏移）。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV2", "UAV2");
  node.setYieldCandidatesEnabledForTest(true);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), kStateExecuting);

  // peer UAV1 悬停在本机路径中点 (5,0,5)。
  auto peer = makeIntent(100.0, 1, "UAV1");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  node.runCollisionCheckTick();
  // 10Hz 检测只置位，不直接提交候选。
  EXPECT_TRUE(node.planningRequested());
  EXPECT_FALSE(node.yieldCommittedForTest());
  ASSERT_TRUE(node.replanOnce());  // 常规 replan 失败 → 候选提交
  EXPECT_TRUE(node.yieldCommittedForTest());
  EXPECT_EQ(node.execState(), kStateExecuting);
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  // 候选 LEFT/RIGHT 在冲突区横向偏移（时间轴经 scale 拉伸，偏移窗口不固定在
  // 原 t=5s；扫描全轨迹最大 |y| 即可）。
  double max_dev = 0.0;
  for (size_t i = 0; i < traj->t.size(); ++i)
    max_dev = std::max(max_dev, std::fabs(traj->pts[i].y()));
  EXPECT_GT(max_dev, 0.3);
}

TEST(EgoStateMachine, YieldLowIdStandOnRequestsReplan)
{
  // 本机 UAV1（ID 1 < peer UAV2 2）→ STAND_ON：collisionCheckTick 只置位；
  // replanOnce 常规规划失败后 STAND_ON 只尝试 SLOW 变速候选（26082621 迭代），
  // 悬停 peer 下 SLOW 无效（减速撞悬停点）→ 无候选 → PLAN_FAILED 收口。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV1", "UAV1");
  node.setYieldCandidatesEnabledForTest(true);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), kStateExecuting);

  auto peer = makeIntent(100.0, 1, "UAV2");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  node.runCollisionCheckTick();
  EXPECT_FALSE(node.yieldCommittedForTest());
  EXPECT_TRUE(node.planningRequested());
  // 软着陆（26082700）：第一次失败不立即 PLAN_FAILED，保持 EXECUTING 重试，
  // 软着陆计时开始（plan_fail_since_ 非零）。
  EXPECT_FALSE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_TRUE(!node.planFailSinceForTest().isZero());
  // 1s 定时重试（timed_trigger，短步进避免重试起点与悬停 peer 重合）：仍失败
  // 且 <10s 累计 → 继续软着陆不收口（10s 累计收口由
  // YieldReplanStandOnFailsToPlan 覆盖）。
  clock.advance(0.5);
  node.injectLocalPose(makePose(0.0, 0.0, 5.0, clock.get()));
  EXPECT_FALSE(node.replanOnce(true));
  EXPECT_EQ(node.execState(), kStateExecuting);
}

TEST(EgoStateMachine, YieldEpisodeLocksCommit)
{
  // episode 语义（单一入口）：collisionCheckTick 只置位，不直接提交候选；
  // 候选提交后若已清除冲突（min_clear ≥1.3）则锁存释放、轨迹不被 tick 改写。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV2", "UAV2");
  node.setYieldCandidatesEnabledForTest(true);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());

  auto peer = makeIntent(100.0, 1, "UAV1");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  node.runCollisionCheckTick();
  ASSERT_TRUE(node.planningRequested());
  EXPECT_FALSE(node.yieldCommittedForTest());  // tick 不提交候选
  ASSERT_TRUE(node.replanOnce());              // 首轮失败 → 提交候选
  EXPECT_TRUE(node.yieldCommittedForTest());
  EXPECT_EQ(node.execState(), kStateExecuting);
  const auto traj1 = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj1));

  // 候选已清除冲突（min_clear ≈1.5 ≥1.3 gate）→ 同一 peer 再次 tick 不置位，
  // 候选锁存释放（yield_committed_ 复位），轨迹不被 tick 改写。
  node.runCollisionCheckTick();
  EXPECT_FALSE(node.planningRequested());
  EXPECT_FALSE(node.yieldCommittedForTest());
  const auto traj2 = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj2));
  ASSERT_EQ(traj1->pts.size(), traj2->pts.size());
  for (size_t i = 0; i < traj1->pts.size(); ++i)
    EXPECT_TRUE(traj1->pts[i].isApprox(traj2->pts[i]));
}

TEST(EgoStateMachine, YieldReplanOnPlanFailureCommitsCandidate)
{
  // 方案 A：首轮规划穿越失败（UAV1 悬停挡路）→ tryYieldReplan 生成 LEFT/RIGHT
  // 候选并提交（replanOnce 经候选成功，exec_state 保持 EXECUTING，轨迹绕行）。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV2", "UAV2");
  node.setYieldCandidatesEnabledForTest(true);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);

  auto peer = makeIntent(100.0, 1, "UAV1");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  // 首轮 replan 因硬门失败 → tryYieldReplan 提交让路候选。
  EXPECT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  // 候选在冲突区横向偏移（时间轴经 scale 拉伸，扫描全轨迹最大 |y|）。
  double max_dev = 0.0;
  for (size_t i = 0; i < traj->t.size(); ++i)
    max_dev = std::max(max_dev, std::fabs(traj->pts[i].y()));
  EXPECT_GT(max_dev, 0.3);
}

TEST(EgoStateMachine, YieldReplanStandOnFailsToPlan)
{
  // 本机 UAV1（ID 1 < peer UAV2 2）→ STAND_ON：规划失败不生成候选 → PLAN_FAILED。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV1", "UAV1");
  node.setYieldCandidatesEnabledForTest(true);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);

  auto peer = makeIntent(100.0, 1, "UAV2");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  // 软着陆（26082700）：首败保持 HOLD（未进入 EXECUTING，不立即收口），
  // 连续失败累计 >10s 才收口 EGO_PLAN_FAILED。
  EXPECT_FALSE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateHold);
  clock.advance(11.0);
  node.injectLocalPose(makePose(0.0, 0.0, 5.0, clock.get()));
  EXPECT_FALSE(node.replanOnce(true));
  EXPECT_EQ(node.execState(), ego_planner_driver::kStatePlanFailed);
}

TEST(EgoStateMachine, ReplanFailSoftLandingResetsOnCommit)
{
  // 软着陆恢复（26082700）：失败后成功提交（定时 replan 恢复）→ plan_fail_since_
  // 清空重新计时，避免恢复后旧失败累计直接收口。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV1", "UAV1");
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());  // 首轮成功
  ASSERT_EQ(node.execState(), kStateExecuting);

  // 注入悬停 peer 挡路 → 常规 replan 失败（STAND_ON 只 SLOW，悬停无解）→ 软着陆。
  auto peer = makeIntent(100.0, 1, "UAV2");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));
  node.runCollisionCheckTick();
  EXPECT_FALSE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_TRUE(!node.planFailSinceForTest().isZero());  // 计时开始

  // peer 挪开（新轨迹版本）→ 定时 replan 成功 → 计时清空。
  auto peer_away = makeIntent(100.0, 2, "UAV2");
  peer_away.t = {0.0, 20.0};
  peer_away.sampled_traj[0].position.x = 5.0;
  peer_away.sampled_traj[0].position.z = 5.0;
  peer_away.sampled_traj[1].position.x = 5.0;
  peer_away.sampled_traj[1].position.y = -8.0;
  peer_away.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer_away));
  clock.advance(1.0);
  node.injectLocalPose(makePose(0.0, 0.0, 5.0, clock.get()));
  ASSERT_TRUE(node.replanOnce(true));
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_TRUE(node.planFailSinceForTest().isZero());  // 已重置
}



TEST(EgoStateMachine, ReplanWithLowClearanceMarginCommitsCandidate)
{
  // B（26082621 迭代）：常规规划通过 1m 硬门但裕度不足（min_clear < clearance_
  // factor×1m=1.2）→ 也生成候选替换成更安全轨迹（防"规划通过但实际间距偏小"）。
  // UAV2（YIELD）直线到 (10,0,5)，peer UAV1 悬停在路径侧面 (5,1.05,5)：常规轨迹
  // 在 x=5 处距 peer 1.05m（>1m 硬门通过，但 <1.2m 裕度）→ 候选 LEFT/RIGHT 提交。
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  node.setIdentityForTest("UAV2", "UAV2");
  node.setYieldCandidatesEnabledForTest(true);
  ASSERT_TRUE(setupHold(node, clock, 5.0));
  node.sendGoal(10.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  ASSERT_EQ(node.execState(), kStateExecuting);

  auto peer = makeIntent(100.0, 1, "UAV1");
  peer.t = {0.0, 20.0};
  peer.sampled_traj[0].position.x = 5.0;
  peer.sampled_traj[0].position.y = 1.05;
  peer.sampled_traj[0].position.z = 5.0;
  peer.sampled_traj[1].position.x = 5.0;
  peer.sampled_traj[1].position.y = 1.05;
  peer.sampled_traj[1].position.z = 5.0;
  ASSERT_TRUE(node.injectNeighborIntent(peer));

  // 周期碰撞检查置位（冲突 gate 1.2m：本机轨迹在 x=5 处距 peer 1.05m < 1.2m）。
  node.runCollisionCheckTick();
  ASSERT_TRUE(node.planningRequested());

  // 常规 replan 通过 1m 硬门（1.05m），但 B 裕度门槛 1.2m 触发候选。
  ASSERT_TRUE(node.replanOnce());
  EXPECT_TRUE(node.yieldCommittedForTest());
  EXPECT_EQ(node.execState(), kStateExecuting);
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  double max_dev = 0.0;
  for (size_t i = 0; i < traj->t.size(); ++i)
    max_dev = std::max(max_dev, std::fabs(traj->pts[i].y()));
  EXPECT_GT(max_dev, 0.5);  // 候选横向偏移 ≥1.3
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

TEST(EgoStateMachine, GpsBiasCorrectsNeighborIntentEpoch)
{
  FakeClock clock;
  clock.set(100.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // 本机 bias=2.0s、邻机 bias=3.0s → 修正量 own-peer = -1.0s：
  // 邻机激活时刻 100.0（邻机时间轴）修正到本机时间轴 99.0。
  node.injectOwnBias(2.0);
  node.injectPeerBias("UAV2", 3.0);
  ASSERT_TRUE(node.injectNeighborIntent(makeIntent(100.0, 1, "UAV2")));
  EXPECT_NEAR(node.neighborStartTimeForTest("UAV2"), 99.0, 1e-6);

  // bias 过期（> bias_valid_s_=120s）→ 回退不修正（epoch = 原始 stamp）。
  clock.advance(121.0);  // now = 221.0
  ASSERT_TRUE(node.injectNeighborIntent(makeIntent(221.0, 2, "UAV2")));
  EXPECT_NEAR(node.neighborStartTimeForTest("UAV2"), 221.0, 1e-6);
}

TEST(EgoStateMachine, MissingOneGpsBiasFallsBackToUncorrectedEpoch)
{
  FakeClock clock;
  clock.set(100.0);

  {
    EgoPlannerDriverNode node;
    node.setReplanLoopEnabled(false);
    node.setClockSource([&clock]() { return clock.now(); });
    ASSERT_TRUE(setupHold(node, clock, 5.0));
    node.injectOwnBias(2.0);
    ASSERT_TRUE(node.injectNeighborIntent(makeIntent(100.0, 1, "UAV2")));
    EXPECT_NEAR(node.neighborStartTimeForTest("UAV2"), 100.0, 1e-6);
  }

  {
    EgoPlannerDriverNode node;
    node.setReplanLoopEnabled(false);
    node.setClockSource([&clock]() { return clock.now(); });
    ASSERT_TRUE(setupHold(node, clock, 5.0));
    node.injectPeerBias("UAV2", 3.0);
    ASSERT_TRUE(node.injectNeighborIntent(makeIntent(100.0, 1, "UAV2")));
    EXPECT_NEAR(node.neighborStartTimeForTest("UAV2"), 100.0, 1e-6);
  }
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

  // kNeighborStaleS=30s（周期 intent 刷新 + 长轨迹）：意图 epoch 允许在
  // [now-30, now+30]，超过才拒绝。
  clock.set(140.0);
  EXPECT_FALSE(node.injectNeighborIntent(makeIntent(100.2, 2)));  // age=39.8s > 30s
  EXPECT_FALSE(node.injectNeighborIntent(makeIntent(60.0, 2)));   // age=80s > 30s
}

TEST(EgoStateMachine, CollisionCheckTickRequestsReplanOnConflict)
{
  ros::param::set("/uav_id", "UAV2");
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
  // 无邻机时周期碰撞检查不置位（replan 后立即检查）。
  node.runCollisionCheckTick();
  EXPECT_FALSE(node.planningRequested());

  const boost::optional<ego_planner_driver::TimedTrajectory> own =
      node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(own));
  auto conflict = makeIntent(100.0, 1, "UAV1");
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
  // 事件驱动 replan 已删除（implementation_plan_26082602）：接收邻机 intent
  // 只更新缓冲，不再立即置 planning_requested_。
  EXPECT_FALSE(node.planningRequested());
  // 周期碰撞检查（10Hz tick 等价）：本机轨迹 vs 冲突邻机 < 1m → 请求 replan
  // （方案 A 单一入口：只置位，候选/制动由 replan 线程统一提交）。
  node.runCollisionCheckTick();
  EXPECT_TRUE(node.planningRequested());
  EXPECT_FALSE(node.yieldCommittedForTest());

  ros::param::set("/uav_id", "UAV1");
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

  // 非退化目标（起点 (0,0,15) -> 目标 (1,0,15)）：正常 EXECUTING → COMPLETED。
  node.sendGoal(1.0, 0.0, 15.0);
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
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 1.0, 0.5);

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

// §3.1（260828 修复）：无 current_ 时同点目标由 snapshot 前的 fresh odometry
// 判定真实到达 → COMPLETED（原缺陷：checkArrival 在无 current_ 时直接退出，
// 同点目标持续 HOLD；又因 DEGENERATE_SEGMENT 只返回不收口，任务永不完成）。
TEST(EgoStateMachine, DegenerateSamePointRealArrivalCompletes)
{
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });

  ASSERT_TRUE(setupHold(node, clock, 15.0));
  EXPECT_EQ(node.execState(), kStateHold);

  // 目标 == 起点：(0,0,15)。真实 odometry 已在最终关键点附近（无 current_）：
  // snapshot 前 fresh odometry 收口等待稳定窗口，不构建退化 snapshot。
  node.sendGoal(0.0, 0.0, 15.0);
  EXPECT_TRUE(node.replanOnce());  // 稳定窗口未满：本轮已由 odometry 等待，返回 true
  EXPECT_EQ(node.execState(), kStateHold);
  EXPECT_FALSE(static_cast<bool>(node.steadyTarget()));
  EXPECT_TRUE(node.planFailSinceForTest().isZero());

  // 推进 1.1s + 刷新 pose → 稳定窗口满 → COMPLETED（非 HOLD 锁存）。
  clock.advance(1.1);
  node.injectLocalPose(makePose(0.0, 0.0, 15.0, clock.get()));
  EXPECT_TRUE(node.replanOnce(true));
  EXPECT_EQ(node.execState(), kStateCompleted);
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 0.0, 1e-6);
  EXPECT_NEAR(node.steadyTarget()->pos.z(), 15.0, 1e-6);
  EXPECT_TRUE(node.planFailSinceForTest().isZero());
}

// §3.1（260828 修复）：已有 plan_fail_since_ 后持续 DEGENERATE_SEGMENT 必须走
// 普通可重试失败路径（soft-landing 计时），10s 内收口 EGO_PLAN_FAILED；既不
// 清除失败计时也不无限 HOLD。
TEST(EgoStateMachine, DegenerateWhileFailingClosesAfterTimeout)
{
  FakeClock clock;
  clock.set(100.0);  // 非零起步：plan_start_ 非零（isZero() 假），collision tick 不提前返回
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });

  ASSERT_TRUE(setupHold(node, clock, 5.0));
  EXPECT_EQ(node.execState(), kStateHold);

  // 首轮：目标 (5,0,5) 成功 EXECUTING（无 peer）。
  node.sendGoal(5.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 注入与本机轨迹完全同刻同点的挡路 peer（任何时刻同刻距离 0 < 1m 硬门）
  // → 冲突检测必然置位；常规 replan 也必然因 1m 硬门失败。
  const boost::optional<ego_planner_driver::TimedTrajectory> own =
      node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(own));
  auto peer = makeIntent(100.0, 1, "UAV2");
  peer.t = own->t;
  peer.sampled_traj.clear();
  for (const auto& p : own->pts)
  {
    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();
    peer.sampled_traj.push_back(pose);
  }
  ASSERT_TRUE(node.injectNeighborIntent(peer));
  node.runCollisionCheckTick();
  ASSERT_TRUE(node.planningRequested());
  EXPECT_FALSE(node.replanOnce());  // 挡路 peer → 失败 → plan_fail_since_ 启动
  EXPECT_FALSE(node.planFailSinceForTest().isZero());

  // 推进时钟模拟轨迹执行（elapsed 接近 duration）：pose 固定 (4.3,0,5)
  // （距最终点 (5,0,5) 0.7m > 0.5m 到达阈值，不算到达）。planned_start 采样
  // clamp 到轨迹末端 (5,0,5) == target → core 返回 DEGENERATE_SEGMENT；此时已有
  // 失败计时 → 走软着陆失败路径：不清计时、不提交、定时重试直到 10s 收口。
  for (int i = 0; i < 14; ++i)
  {
    clock.advance(1.0);
    node.injectLocalPose(makePose(4.3, 0.0, 5.0, clock.get()));
    node.publishSetpointTick();  // 维持 setpoint 流（不触发 COMPLETED/HOLD 分支以外的干扰）
    EXPECT_FALSE(node.replanOnce(true)) << "iter " << i;
    // 收口前失败计时持续不清（收口后清零，由下方最终断言验证）。
    if (node.execState() != ego_planner_driver::kStatePlanFailed)
      EXPECT_FALSE(node.planFailSinceForTest().isZero());
    else
      EXPECT_TRUE(node.planFailSinceForTest().isZero());
    if (node.execState() == ego_planner_driver::kStatePlanFailed)
      break;
  }
  // 总失败时长超过 plan_fail_timeout_s_（10s）→ EGO_PLAN_FAILED 收口。
  EXPECT_EQ(node.execState(), ego_planner_driver::kStatePlanFailed);
  EXPECT_TRUE(node.planFailSinceForTest().isZero());  // 收口后清计时
}

// §4.2（260828 方式 B，node 层）：RESOURCE_LIMIT 零 activation retry、零 yield、
// 不发布部分 intent、立即失败/HOLD 收口。通过私有参数注入紧 parameterization
// cap（8），长距离目标触发 core 资源失败。
TEST(EgoStateMachine, ResourceLimitClosesPlanFailedImmediately)
{
  ros::param::set("/ego_state_machine_test/max_parameterization_points", 8);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setReplanLoopEnabled(false);
  node.setClockSource([&clock]() { return clock.now(); });
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // node 构造已读取一次私有参数。立即删除，防止 cap=8 残留污染后续测试
  // （node 构造只读一次，删除不影响本测试后续 replanOnce）。
  ros::param::del("/ego_state_machine_test/max_parameterization_points");
  // 6m 目标（cap=8 时弧长重采样点约 16 > 8）→ core RESOURCE_LIMIT。
  node.sendGoal(6.0, 0.0, 5.0);
  ASSERT_TRUE(node.replanOnce());  // 同一事务内立即收口 → 返回 true
  EXPECT_EQ(node.execState(), ego_planner_driver::kStatePlanFailed);
  EXPECT_TRUE(node.planFailSinceForTest().isZero());
  EXPECT_FALSE(node.yieldCommittedForTest());          // 零 yield
  EXPECT_FALSE(static_cast<bool>(node.currentTrajectory()));  // 不发布部分 intent
  // 立即 HOLD 语义：steady_target_ 由 captureHoldTargetLocked 锁存（fresh pose）。
  EXPECT_TRUE(static_cast<bool>(node.steadyTarget()));
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

TEST(EgoStateMachine, PlanExecAdvanceTargetSkipsReachedKeypoints)
{
  // plan-exec 分支 A：距当前目标 <0.5m → 切相邻下一个 keypoint 为目标。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // start(0) + 3m 间距关键点：(0),(3),(6),(9) —— waypoints_[0] 为 start。
  std::vector<geometry_msgs::Point32> wps;
  for (int x : {0, 3, 6, 9})
  {
    geometry_msgs::Point32 p;
    p.x = x; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 位置 (3.2,0,5)：距当前目标 (3,0,5)=0.2m <0.5m → 推进到 (6,0,5)。
  node.injectLocalPose(makePose(3.2, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  EXPECT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  EXPECT_NEAR(traj->pts.back().x(), 6.0, 0.6);

  // 位置 (6.2,0,5)：距当前目标 (6,0,5)=0.2m <0.5m → 推进到最终 (9,0,5)。
  node.injectLocalPose(makePose(6.2, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  ASSERT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const auto traj2 = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj2));
  EXPECT_NEAR(traj2->pts.back().x(), 9.0, 0.6);

  // 最终关键点已为当前目标（waypoint_consumed_ == size-1）→ 不再推进。
  node.injectLocalPose(makePose(8.9, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  EXPECT_FALSE(node.planningRequested());
}

TEST(EgoStateMachine, PlanExecAdvanceDiscardsTooCloseTargets)
{
  // plan-exec 分支 A：多个关键点挤在 0.5m 内时丢弃推进，直接跳到 >=0.5m 目标。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  // start(0), (3),(3.1),(3.2),(6) —— (3)/(3.1)/(3.2) 三连点 <0.5m 内。
  std::vector<geometry_msgs::Point32> wps;
  for (double x : {0.0, 3.0, 3.1, 3.2, 6.0})
  {
    geometry_msgs::Point32 p;
    p.x = x; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());

  // 位置 (3.05,0,5)：距 (3,0,5)=0.05 <0.5 → 丢弃 (3)/(3.1)/(3.2)，目标 → (6,0,5)。
  node.injectLocalPose(makePose(3.05, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();
  ASSERT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const auto traj = node.currentTrajectory();
  ASSERT_TRUE(static_cast<bool>(traj));
  EXPECT_NEAR(traj->pts.back().x(), 6.0, 0.6);
}

TEST(EgoStateMachine, PlanExecArrivalCompletesAtFinalKeypoint)
{
  // plan-exec 分支 B：距最终关键点 < arrival_reach_thresh_m_（0.5m，稳定窗口后）
  // → COMPLETED，冻结目标位姿。0.15m 偏差在 0.5m 阈值内（260828 由 0.2m 放宽，
  // 覆盖 15 机 prepare 实测 0.29m 终点偏差）。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int x : {0, 3, 6, 9})
  {
    geometry_msgs::Point32 p;
    p.x = x; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 飞到最终关键点 (9,0,5)：0.15m 偏差 < 0.2m 到达阈值（> 0.1m 旧阈值）。
  node.injectLocalPose(makePose(8.85, 0.0, 5.0, clock.get()));
  node.arrivalCheckTick();  // 到达窗口开始（稳定未满）
  EXPECT_EQ(node.execState(), kStateExecuting);
  clock.advance(1.1);  // > arrival_stable_s
  node.injectLocalPose(makePose(8.85, 0.0, 5.0, clock.get()));  // pose 保持新鲜
  node.arrivalCheckTick();
  EXPECT_EQ(node.execState(), kStateCompleted);
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  EXPECT_NEAR(node.steadyTarget()->pos.x(), 9.0, 1e-6);
  EXPECT_NEAR(node.steadyTarget()->pos.z(), 5.0, 1e-6);
}

TEST(EgoStateMachine, RollingSegmentsReachCompleted)
{
  // 相邻目标推进：3m 密化关键点多段滚动后最终 COMPLETED（prepare 场景到达路径）。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 8; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  bool completed = false;
  for (int guard = 0; guard < 4000; ++guard)
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
  // 模拟 SITL 跟踪滞后 0.05m（< arrival_reach_thresh_m 0.5m）：推进/到达判定
  // 仍收敛到 COMPLETED。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 8; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  bool completed = false;
  for (int guard = 0; guard < 4000; ++guard)
  {
    node.publishSetpointTick();
    const mavros_msgs::PositionTarget sp = node.lastSetpoint();
    node.injectLocalPose(makePose(sp.position.x - 0.05, sp.position.y,
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

TEST(EgoStateMachine, ReplanSnapshotsKeepSegmentVelocity)
{
  // 回归：periodic replan 在当前段执行中快照时，新段起点必须保持旧轨迹速度
  // （rolling continuity），不是从 0 重新加速。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 4; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

  for (int step = 0; step < 60; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    clock.advance(0.1);
    if (node.planningRequested())
      break;
  }
  EXPECT_TRUE(node.planningRequested());
  ASSERT_TRUE(node.replanOnce());
  const mavros_msgs::PositionTarget sp = node.lastSetpoint();
  EXPECT_TRUE(std::isfinite(sp.velocity.x) && std::isfinite(sp.velocity.y));
  const double speed = std::sqrt(sp.velocity.x * sp.velocity.x +
                                 sp.velocity.y * sp.velocity.y +
                                 sp.velocity.z * sp.velocity.z);
  EXPECT_GT(speed, 0.05);
  EXPECT_EQ(node.execState(), kStateExecuting);

  // 持续推进，setpoint 应越过首段目标（3m）。
  double max_x = sp.position.x;
  for (int step = 0; step < 100; ++step)
  {
    node.publishSetpointTick();
    echoSetpoint(node, clock);
    max_x = std::max(max_x, node.lastSetpoint().position.x);
    clock.advance(0.1);
    if (node.planningRequested())
      ASSERT_TRUE(node.replanOnce());
    if (node.execState() == kStateCompleted)
      break;
  }
  EXPECT_GT(max_x, 3.0);
}

TEST(EgoStateMachine, PeriodicReplanTriggersDuringSegmentExecution)
{
  // 分支 C：段内 elapsed >= replan_period_s（1.0s）→ 时间触发置 planning_requested_。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 4; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);
  EXPECT_FALSE(node.planningRequested());

  // 段内推进超过 replan_period_s（1.0s）且未到最终目标 → 时间触发重规划。
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

TEST(EgoStateMachine, ReplanUsesTrajectoryStateNotLivePose)
{
  // 滚动重规划起点必须来自旧轨迹同一时刻的完整状态，而不是"实际位置 + 旧导数"。
  usleep(600 * 1000);
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);
  ASSERT_TRUE(setupHold(node, clock, 5.0));

  std::vector<geometry_msgs::Point32> wps;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::Point32 p;
    p.x = i * 3.0; p.y = 0.0; p.z = 5.0;
    wps.push_back(p);
  }
  node.sendWaypoints(wps);
  ASSERT_TRUE(node.replanOnce());
  EXPECT_EQ(node.execState(), kStateExecuting);

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

TEST(EgoStateMachine, HoldLocksPositionNotPoseDrift)
{
  // 删 POSE_STALE 后：HOLD setpoint = 进入 HOLD 时锁存的位置/yaw，不随 pose 漂移。
  FakeClock clock;
  clock.set(0.0);
  EgoPlannerDriverNode node;
  node.setClockSource([&clock]() { return clock.now(); });
  node.setReplanLoopEnabled(false);

  ASSERT_TRUE(setupHold(node, clock, 12.0));  // 捕获 HOLD → 锁存 (0,0,12)
  ASSERT_TRUE(static_cast<bool>(node.steadyTarget()));
  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.z, 12.0, 1e-6);

  // pose 漂移（如 GPS 抖动）不影响 HOLD setpoint：锁存位置不更新。
  node.injectLocalPose(makePose(5.0, 3.0, 9.0, clock.get()));
  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.y, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.z, 12.0, 1e-6);
  EXPECT_EQ(node.execState(), kStateHold);

  // 持续漂移也不改变锁存。
  node.injectLocalPose(makePose(10.0, 10.0, 2.0, clock.get()));
  node.publishSetpointTick();
  EXPECT_NEAR(node.lastSetpoint().position.x, 0.0, 1e-6);
  EXPECT_NEAR(node.lastSetpoint().position.z, 12.0, 1e-6);
}

}  // namespace

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
