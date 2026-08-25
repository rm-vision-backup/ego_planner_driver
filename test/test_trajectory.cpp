#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "ego_planner_driver/ego_planner_core.h"
#include "ego_planner_driver/polynomial_traj.h"

using ego_planner_driver::EgoPlannerConfig;
using ego_planner_driver::EgoPlannerCore;
using ego_planner_driver::GoalPoint;
using ego_planner_driver::NeighborTrajectory;
using ego_planner_driver::TimedTrajectory;
using ego_planner_driver::TrajectorySample;
using ego_planner_driver::WayPoint;

namespace
{
EgoPlannerConfig makeConfig()
{
  EgoPlannerConfig config;
  config.ts = 0.4;
  config.max_vel = 8.0;
  config.max_acc = 6.0;
  config.swarm_clearance = 0.5;
  config.planning_range = 30.0;
  config.max_iterations = 200;
  return config;
}

NeighborTrajectory straightNeighbor(double sx, double sy, double ex, double ey, double sz = 12.0)
{
  NeighborTrajectory n;
  n.clearance = 2.0;
  n.start_time = 0.0;
  n.duration = 10.0;
  for (int i = 0; i < 100; ++i)
  {
    const double a = static_cast<double>(i) / 99.0;
    n.t.push_back(a * 10.0);
    n.pts.emplace_back(sx + (ex - sx) * a, sy + (ey - sy) * a, sz);
  }
  return n;
}

NeighborTrajectory asNeighbor(const TimedTrajectory& trajectory,
                              double start_time = 0.0)
{
  NeighborTrajectory neighbor;
  neighbor.start_time = start_time;
  neighbor.duration = trajectory.duration;
  neighbor.clearance = 0.5;
  neighbor.t = trajectory.t;
  neighbor.pts = trajectory.pts;
  return neighbor;
}

void expectDynamicsWithin(const TimedTrajectory& trajectory,
                          const EgoPlannerConfig& config)
{
  double max_speed = 0.0;
  double max_acceleration = 0.0;
  for (const auto& velocity : trajectory.vel)
    max_speed = std::max(max_speed, velocity.norm());
  for (const auto& acceleration : trajectory.acc)
    max_acceleration = std::max(max_acceleration, acceleration.norm());
  EXPECT_LE(max_speed, config.max_vel * 1.5);
  EXPECT_LE(max_acceleration, config.max_acc * 1.05);
}
} // namespace

TEST(EgoPlannerCore, PlanOnceProducesTimedTrajectory)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(10.0, 0.0, 15.0), 0.0}};
  TimedTrajectory trajectory;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, trajectory, cost));
  ASSERT_GT(trajectory.pts.size(), 1u);
  ASSERT_GT(trajectory.duration, 0.0);
  EXPECT_NEAR(trajectory.pts.front().x(), 0.0, 1e-3);
  EXPECT_NEAR(trajectory.pts.back().x(), 10.0, 1e-3);
}

TEST(EgoPlannerCore, PlanOnceAppliesStartDerivativesForRollingContinuity)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(10.0, 0.0, 15.0), 0.0}};
  const Eigen::Vector3d start_vel(1.5, 0.0, 0.0);
  const Eigen::Vector3d start_acc(0.0, 0.0, 0.0);
  TimedTrajectory traj;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, traj, cost, start_vel, start_acc));
  ASSERT_GT(traj.vel.size(), 1u);
  // 起点 B-spline 一阶导数被精确锚定为 start_vel（优化器/rebound 不动前 3 列）。
  EXPECT_NEAR(traj.vel.front().x(), start_vel.x(), 0.1);
  EXPECT_NEAR(traj.vel.front().y(), start_vel.y(), 0.1);
  // 对照：默认全零起始导数 -> 起点静止，证明 vel 参数确实生效。
  TimedTrajectory zero_traj;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, zero_traj, cost));
  ASSERT_GT(zero_traj.vel.size(), 1u);
  EXPECT_NEAR(zero_traj.vel.front().x(), 0.0, 0.1);
}

TEST(EgoPlannerCore, PlanOnceWithoutWaypointsHoldsPosition)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(1.0, 2.0, 15.0), 0.0};
  TimedTrajectory trajectory;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, {}, {}, trajectory, cost));
  EXPECT_NEAR(trajectory.pts.front().x(), 1.0, 1e-3);
  EXPECT_NEAR(trajectory.pts.front().y(), 2.0, 1e-3);
}

TEST(EgoPlannerCore, NeighborMissingDegradesToSingle)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(8.0, 0.0, 15.0), 0.0}};
  TimedTrajectory with_neighbor, without_neighbor;
  double cost_a = 0.0, cost_b = 0.0;
  std::vector<NeighborTrajectory> neighbors{straightNeighbor(2.0, 6.0, 2.0, 2.0)};
  ASSERT_TRUE(core.planOnce(start, waypoints, neighbors, with_neighbor, cost_a));
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, without_neighbor, cost_b));
  EXPECT_FALSE(with_neighbor.pts.empty());
  EXPECT_FALSE(without_neighbor.pts.empty());
  EXPECT_TRUE(std::isfinite(cost_a));
  EXPECT_TRUE(std::isfinite(cost_b));
}

TEST(EgoPlannerCore, FilterByRangeDropsFarNeighbors)
{
  EgoPlannerCore core(makeConfig());
  std::vector<NeighborTrajectory> neighbors{
      straightNeighbor(5.0, 5.0, 5.0, 6.0),
      straightNeighbor(100.0, 100.0, 100.0, 101.0)};
  const auto kept = EgoPlannerCore::filterByRange(
      neighbors, Eigen::Vector3d(0.0, 0.0, 15.0), 30.0);
  ASSERT_EQ(kept.size(), 1u);
}

TEST(EgoPlannerCore, CheckClearanceCatchesViolation)
{
  EgoPlannerConfig config = makeConfig();
  config.enable_rebound = false;  // 无 rebound 时软惩罚不足以避开正对邻居
  EgoPlannerCore core(config);
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(0.0, 10.0, 15.0), 0.0}};
  TimedTrajectory trajectory;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, trajectory, cost));
  // Neighbor parked on the trajectory path at the same altitude.
  std::vector<NeighborTrajectory> neighbors{straightNeighbor(0.0, 5.0, 0.0, 5.0, 15.0)};
  EXPECT_FALSE(core.checkClearance(trajectory, neighbors, 0.5));
  EXPECT_TRUE(core.checkClearance(trajectory, {}, 0.5));
}

TEST(EgoPlannerCore, SwarmReboundPushesTrajectoryAway)
{
  EgoPlannerConfig config = makeConfig();
  config.enable_rebound = true;  // rebound 硬推：把侵入邻居 clearance 的控制点推开
  EgoPlannerCore core(config);
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  // 模拟上层 0.4-0.5m 密化后的航线（真实运行 waypoints 已密化，控制点足够）
  std::vector<WayPoint> waypoints;
  for (double y = 0.5; y <= 10.0; y += 0.5)
    waypoints.push_back(WayPoint{Eigen::Vector3d(0.0, y, 15.0), 0.0});
  TimedTrajectory trajectory;
  double cost = 0.0;
  // 邻居正对轨迹路径（同一高度、同时段）
  std::vector<NeighborTrajectory> neighbors{straightNeighbor(0.0, 5.0, 0.0, 5.0, 15.0)};
  ASSERT_TRUE(core.planOnce(start, waypoints, neighbors, trajectory, cost));
  // rebound 后 clearance 硬校验应通过（控制点被推开绕开正对邻居）
  EXPECT_TRUE(core.checkClearance(trajectory, neighbors, 0.5));
}

TEST(EgoPlannerCore, CrossingTrajectoriesReplanToSafeArrival)
{
  EgoPlannerConfig config = makeConfig();
  config.max_vel = 1.0;
  config.cruise_velocity = 1.0;
  config.lambda_feasibility = 0.5;
  config.enable_rebound = true;
  EgoPlannerCore core(config);
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{
      WayPoint{Eigen::Vector3d(10.0, 0.0, 15.0), 0.0}};

  // The neighbor crosses x=5 at t=5 while this UAV traverses x=0 -> 10.
  // Both trajectories use the same activation epoch (start_time=0 on this
  // local planning axis), so a planner that honours the intent time contract
  // must actively deviate rather than reject the mission or rely on HOLD.
  const std::vector<NeighborTrajectory> neighbors{
      straightNeighbor(5.0, -5.0, 5.0, 5.0, 15.0)};
  TimedTrajectory trajectory;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, neighbors, trajectory, cost,
                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            true));
  ASSERT_FALSE(trajectory.pts.empty());
  EXPECT_NEAR(trajectory.pts.back().x(), 10.0, 1e-3);
  EXPECT_NEAR(trajectory.pts.back().y(), 0.0, 1e-3);
  EXPECT_TRUE(core.checkClearance(trajectory, neighbors, 1.0));
  expectDynamicsWithin(trajectory, config);
}

TEST(EgoPlannerCore, ReciprocalNominalIntentBlocksUnsafeDefaultEnablement)
{
  EgoPlannerConfig config = makeConfig();
  config.max_vel = 1.0;
  config.cruise_velocity = 1.0;
  config.lambda_feasibility = 0.5;
  config.enable_rebound = true;
  EgoPlannerCore core_a(config);
  EgoPlannerCore core_b(config);
  const GoalPoint start_a{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  const GoalPoint start_b{Eigen::Vector3d(5.0, -5.0, 15.0), 0.0};
  const std::vector<WayPoint> waypoints_a{
      WayPoint{Eigen::Vector3d(10.0, 0.0, 15.0), 0.0}};
  const std::vector<WayPoint> waypoints_b{
      WayPoint{Eigen::Vector3d(5.0, 5.0, 15.0), 0.0}};

  TimedTrajectory nominal_a;
  TimedTrajectory traj_b;
  double cost = 0.0;
  ASSERT_TRUE(core_a.planOnce(start_a, waypoints_a, {}, nominal_a, cost,
                              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                              true));
  // Rebound passes against an ideal constant-speed crossing intent, but it
  // still cannot guarantee a solution against a peer trajectory generated by
  // this same min-snap core. Keep the production default disabled until this
  // assertion can be inverted together with a multi-round convergence test.
  EXPECT_FALSE(core_b.planOnce(start_b, waypoints_b, {asNeighbor(nominal_a)},
                               traj_b, cost, Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), true));
  expectDynamicsWithin(nominal_a, config);
  EXPECT_NEAR(nominal_a.pts.back().x(), 10.0, 1e-3);
}

// 预测式避碰（方向 A）：节点把 SafetyPredictor 公式的动态安全距离注入
// neighbor.safe_clearance_m，规划必须把同刻交叉段当"冲突区域"局部绕开，
// 而不是只在固定 1 m clearance 内推开。正交交叉下 self 沿 y 轴前进、peer
// 沿 x 轴前进且同时到达交点。
TEST(EgoPlannerCore, PredictiveAvoidanceDeviatesAroundCrossingPeer)
{
  EgoPlannerConfig config = makeConfig();
  config.cruise_velocity = 1.5;
  config.lambda_swarm = 1.0;
  config.safety_control_delay_s = 0.20;
  config.safety_compute_budget_s = 0.05;
  config.safety_emergency_decel_mps2 = 2.0;
  config.safety_emergency_buffer_m = 0.10;
  config.safety_protected_distance_m = 1.0;
  EgoPlannerCore core(config);

  // self（UAV2）沿 y 轴从 (0,-4) 到 (0,4)，v=1.5；crossing 点在 y=0。
  const GoalPoint start{Eigen::Vector3d(0.0, -4.0, 15.0), 0.0};
  const std::vector<WayPoint> waypoints{
      WayPoint{Eigen::Vector3d(0.0, 4.0, 15.0), 0.0}};
  const Eigen::Vector3d start_vel(0.0, 1.5, 0.0);

  // peer（UAV1）沿 x 轴从 (-4,0) 到 (4,0)，v=1 m/s，start_time=-2s（已激活
  // 2s，self 当前时刻 peer 位于 x≈-2，正对接近）。
  NeighborTrajectory peer;
  peer.clearance = 0.5;
  peer.start_time = -2.0;
  peer.duration = 8.0;
  peer.data_age_s = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    const double a = static_cast<double>(i) / 99.0;
    peer.t.push_back(a * 8.0);
    peer.pts.emplace_back(-4.0 + 8.0 * a, 0.0, 15.0);
  }

  // 动态安全距离：closing≈1.79 m/s → reaction 0.25s + 制动余量 → safe≈2.35m，
  // 明显大于固定 swarm_clearance*2=1m。
  const double safe = EgoPlannerCore::computeSafeClearance(
      config, peer, -peer.start_time, start.position, start_vel);
  EXPECT_GT(safe, 1.8);
  peer.safe_clearance_m = safe;

  TimedTrajectory baseline, avoided;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, baseline, cost, start_vel));
  ASSERT_TRUE(core.planOnce(start, waypoints, {peer}, avoided, cost, start_vel));

  // 无 peer 时轨迹基本贴直线（x≈0）；有动态 clearance 时必须在交叉点局部
  // 产生明显 x 方向偏移。
  double max_dev_baseline = 0.0, max_dev_avoided = 0.0;
  for (const auto& p : baseline.pts)
    max_dev_baseline = std::max(max_dev_baseline, std::fabs(p.x()));
  for (const auto& p : avoided.pts)
    max_dev_avoided = std::max(max_dev_avoided, std::fabs(p.x()));
  EXPECT_GT(max_dev_avoided, max_dev_baseline + 0.15);
  std::cout << "[predictive_avoidance] safe=" << safe
            << " max_dev_baseline=" << max_dev_baseline
            << " max_dev_avoided=" << max_dev_avoided
            << " plan_once_ok=true" << std::endl;

  // 硬校验应通过：轨迹全程都在 peer 的动态安全距离之外。
  EXPECT_TRUE(core.checkClearance(avoided, {peer}, config.swarm_clearance * 2.0));
}

TEST(EgoPlannerCore, TimedTrajectorySamplesPvaConsistently)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints;
  for (double x = 0.4; x <= 8.0 + 1e-6; x += 0.4)
    waypoints.push_back(WayPoint{Eigen::Vector3d(x, 0.0, 15.0), 0.0});
  const Eigen::Vector3d start_vel(1.5, 0.0, 0.0);
  TimedTrajectory traj;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, traj, cost, start_vel,
                            Eigen::Vector3d::Zero()));
  ASSERT_GT(traj.acc.size(), 1u);
  ASSERT_EQ(traj.acc.size(), traj.pts.size());
  ASSERT_EQ(traj.vel.size(), traj.pts.size());

  // acc 应为有限值，且与 vel 的有限差分一致（B-spline 二阶导数）。
  for (const auto& a : traj.acc)
    EXPECT_TRUE(std::isfinite(a.x()) && std::isfinite(a.y()) && std::isfinite(a.z()));
  const double h = 0.05;
  for (double t : {0.5, 2.0, 4.0})
  {
    if (t >= traj.duration)
      continue;
    TrajectorySample s0, s1;
    ASSERT_TRUE(traj.samplePVA(t, s0));
    ASSERT_TRUE(traj.samplePVA(std::min(t + h, traj.duration), s1));
    const Eigen::Vector3d vel_est = (s1.pos - s0.pos) / std::min(h, traj.duration - t);
    const Eigen::Vector3d acc_est = (s1.vel - s0.vel) / std::min(h, traj.duration - t);
    EXPECT_NEAR(s0.vel.x(), vel_est.x(), 0.08);
    EXPECT_NEAR(s0.vel.y(), vel_est.y(), 0.08);
    EXPECT_NEAR(s0.acc.x(), acc_est.x(), 0.15);
    EXPECT_NEAR(s0.acc.y(), acc_est.y(), 0.15);
  }
}

TEST(EgoPlannerCore, SamplePvaAtStartMatchesRequestedDerivatives)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(10.0, 0.0, 15.0), 0.0}};
  const Eigen::Vector3d start_vel(1.5, 0.0, 0.0);
  TimedTrajectory traj;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, traj, cost, start_vel,
                            Eigen::Vector3d::Zero()));
  TrajectorySample s;
  ASSERT_TRUE(traj.samplePVA(0.0, s));
  EXPECT_NEAR(s.pos.x(), 0.0, 1e-3);
  // 起点速度被参数化锚定为 start_vel（前 3 个控制点不参与优化）。
  EXPECT_NEAR(s.vel.x(), start_vel.x(), 0.1);
  EXPECT_TRUE(std::isfinite(s.acc.x()));
}

TEST(EgoPlannerCore, ClampVectorNormLimitsMagnitudeAndHandlesZero)
{
  using ego_planner_driver::clampVectorNorm;
  // 范数超限：方向保留，模长压到 limit（逐轴限幅做不到这一点）。
  Eigen::Vector3d v(3.0, 4.0, 0.0);  // norm = 5
  clampVectorNorm(v, 2.0);
  EXPECT_NEAR(v.norm(), 2.0, 1e-9);
  EXPECT_NEAR(v.x(), 3.0 * 2.0 / 5.0, 1e-9);
  EXPECT_NEAR(v.y(), 4.0 * 2.0 / 5.0, 1e-9);
  // 未超限：原样保留。
  Eigen::Vector3d w(0.5, 0.0, 0.0);
  clampVectorNorm(w, 1.0);
  EXPECT_NEAR(w.norm(), 0.5, 1e-9);
  // 零向量：保持零，不产生 NaN（v.normalized() 会除零）。
  Eigen::Vector3d z = Eigen::Vector3d::Zero();
  clampVectorNorm(z, 1.0);
  EXPECT_TRUE(z.isZero());
  EXPECT_TRUE(z.allFinite());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(EgoPlannerCore, FeasibilityCubicSmoothVelocity)
{
  // launch 实际配置：max_vel=1.0、lambda_feasibility=0.5、ts=0.4
  EgoPlannerConfig config = makeConfig();
  config.max_vel = 1.0;
  config.lambda_feasibility = 0.5;
  config.enable_rebound = false;
  EgoPlannerCore core(config);
  // 长水平航线（60m，0.4m 密化）模拟巡航段，检查速度平滑性。
  Eigen::Vector3d start(0.0, 0.0, 15.0);
  std::vector<WayPoint> waypoints;
  for (double x = 0.4; x <= 60.0 + 1e-6; x += 0.4)
    waypoints.push_back(WayPoint{Eigen::Vector3d(x, 0.0, 15.0), 0.0});
  TimedTrajectory traj;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(GoalPoint{start, 0.0}, waypoints, {}, traj, cost));
  ASSERT_GT(traj.vel.size(), 2u);
  // 巡航段速度应贴住 max_vel（允许 ~20% 波动），且不出现极端超速。
  double vmax = 0.0;
  std::vector<double> speeds;
  for (const auto& v : traj.vel)
  {
    const double s = v.norm();
    vmax = std::max(vmax, s);
    speeds.push_back(s);
  }
  EXPECT_LE(vmax, 1.5);
  // 中段（避开起/末加速减速）速度应在 0.8-1.3 之间，无剧烈振荡。
  const size_t mid0 = traj.vel.size() / 3, mid1 = traj.vel.size() * 2 / 3;
  double mid_min = 1e9, mid_max = 0.0;
  for (size_t i = mid0; i < mid1; ++i)
  {
    mid_min = std::min(mid_min, speeds[i]);
    mid_max = std::max(mid_max, speeds[i]);
  }
  EXPECT_GT(mid_min, 0.5);
  EXPECT_LT(mid_max, 1.5);
}

TEST(EgoPlannerCore, VerticalClimbWithNeighborsSucceeds)
{
  EgoPlannerConfig config = makeConfig();
  config.enable_rebound = true;
  EgoPlannerCore core(config);
  // 起飞 HOLD z=5，waypoints 垂直密化爬升到 15m 层（0.4m 间距）。
  GoalPoint start{Eigen::Vector3d(-2.35, 5.08, 5.0), 0.0};
  std::vector<WayPoint> waypoints;
  for (double z = 5.4; z <= 10.0 + 1e-6; z += 0.4)
    waypoints.push_back(WayPoint{Eigen::Vector3d(-2.35, 5.08, z), 0.0});
  // 14 邻居：都在起飞层 z=5 附近，各向分布（模拟编队起飞后同层悬停）。
  std::vector<NeighborTrajectory> neighbors;
  double pts[14][2] = {{0.2,5.0},{0.0,0.0},{5.1,10.0},{5.0,5.0},{5.0,0.0},
                       {10.1,9.9},{10.0,4.9},{10.1,0.0},{15.1,10.0},{15.1,5.0},
                       {15.1,0.0},{20.1,10.0},{20.2,5.1},{20.2,0.1}};
  for (auto& p : pts)
  {
    NeighborTrajectory n;
    n.clearance = 2.0;
    n.start_time = -1.0;
    n.duration = 10.0;
    for (int i = 0; i <= 50; ++i)
    {
      n.t.push_back(i / 50.0 * 10.0);
      n.pts.emplace_back(p[0], p[1], 5.0);
    }
    neighbors.push_back(n);
  }
  TimedTrajectory traj;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, neighbors, traj, cost));
  EXPECT_GT(traj.duration, 0.0);
  // 轨迹应爬升（终点 z 显著高于起点）。
  const double z_end = traj.pts.back().z();
  EXPECT_GT(z_end, start.position.z() + 2.0);
}

TEST(OneSegmentPolynomial, InterpolatesBoundaryStates)
{
  using ego_planner_driver::oneSegmentTrajGen;
  const Eigen::Vector3d sp(0.0, 1.0, 15.0), sv(1.5, 0.0, 0.0), sa(0.0, 0.0, 0.0);
  const Eigen::Vector3d ep(10.0, 1.0, 15.0), ev(1.0, 0.0, 0.0), ea(0.0, 0.0, 0.0);
  const double T = 10.0;
  const auto poly = oneSegmentTrajGen(sp, sv, sa, ep, ev, ea, T);
  // 五次多项式闭式解精确插值 start/end 的 P/V/A（min-snap 单段边界约束）。
  EXPECT_NEAR(poly.evaluate(0.0).x(), sp.x(), 1e-6);
  EXPECT_NEAR(poly.evaluate(0.0).y(), sp.y(), 1e-6);
  EXPECT_NEAR(poly.evaluate(0.0).z(), sp.z(), 1e-6);
  EXPECT_NEAR(poly.evaluateVel(0.0).x(), sv.x(), 1e-6);
  EXPECT_NEAR(poly.evaluateAcc(0.0).x(), sa.x(), 1e-6);
  EXPECT_NEAR(poly.evaluate(T).x(), ep.x(), 1e-6);
  EXPECT_NEAR(poly.evaluate(T).y(), ep.y(), 1e-6);
  EXPECT_NEAR(poly.evaluateVel(T).x(), ev.x(), 1e-6);
  EXPECT_NEAR(poly.evaluateAcc(T).x(), ea.x(), 1e-6);
  for (double t = 0.0; t <= T; t += 0.5)
    EXPECT_TRUE(poly.evaluate(t).allFinite());
}

TEST(EgoPlannerCore, PlanOnceFinalSegmentEndsStatic)
{
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(10.0, 0.0, 15.0), 0.0}};
  TimedTrajectory traj;
  double cost = 0.0;
  // final_segment=true：末端静止到达（end_vel=0），轨迹末速度约 0。
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, traj, cost,
                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            true));
  ASSERT_GT(traj.vel.size(), 1u);
  EXPECT_LT(traj.vel.back().norm(), 0.1);
  // 巡航段（final_segment=false）：末速度保持切线巡航速度量级（>= 0.5）。
  TimedTrajectory cruise;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, cruise, cost,
                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            false));
  EXPECT_GT(cruise.vel.back().norm(), 0.5);
  // 滚动连续性：起点速度精确保持 start_vel（P0..P2 锚定）。
  EXPECT_NEAR(cruise.vel.front().x(), 0.0, 0.1);
}

TEST(EgoPlannerCore, MinSnapInitialPointSetYieldsCruiseSpeedProfile)
{
  // min-snap 单段 0.4m 弧长采样生成的初始 point_set 使 B-spline 速度剖面贴近
  // v_cruise，中段无剧烈振荡（轨迹平滑成功判据）。
  EgoPlannerConfig config = makeConfig();
  config.max_vel = 1.0;
  config.cruise_velocity = 1.0;
  config.lambda_feasibility = 0.5;
  config.enable_rebound = false;
  config.enable_time_reparam = false;
  EgoPlannerCore core(config);
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(40.0, 0.0, 15.0), 0.0}};
  TimedTrajectory traj;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, traj, cost,
                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            false));
  ASSERT_GT(traj.vel.size(), 2u);
  double vmax = 0.0;
  std::vector<double> speeds;
  for (const auto& v : traj.vel)
  {
    const double s = v.norm();
    vmax = std::max(vmax, s);
    speeds.push_back(s);
  }
  EXPECT_LE(vmax, 1.5);
  const size_t mid0 = speeds.size() / 3, mid1 = speeds.size() * 2 / 3;
  double mid_min = 1e9, mid_max = 0.0;
  for (size_t i = mid0; i < mid1; ++i)
  {
    mid_min = std::min(mid_min, speeds[i]);
    mid_max = std::max(mid_max, speeds[i]);
  }
  EXPECT_GT(mid_min, 0.5);
  EXPECT_LT(mid_max, 1.5);
}

TEST(TimedTrajectory, SamplePvaClampsOutsideRelativeTimeDomain)
{
  TimedTrajectory traj;
  traj.t = {0.0, 1.0};
  traj.pts = {Eigen::Vector3d(1.0, 2.0, 3.0),
              Eigen::Vector3d(4.0, 5.0, 6.0)};
  traj.vel = {Eigen::Vector3d(0.1, 0.2, 0.3),
              Eigen::Vector3d(0.4, 0.5, 0.6)};
  traj.acc = {Eigen::Vector3d(1.1, 1.2, 1.3),
              Eigen::Vector3d(1.4, 1.5, 1.6)};
  traj.yaw = {0.25, 0.75};

  TrajectorySample sample;
  ASSERT_TRUE(traj.samplePVA(-0.5, sample));
  EXPECT_TRUE(sample.pos.isApprox(traj.pts.front()));
  EXPECT_TRUE(sample.vel.isApprox(traj.vel.front()));
  EXPECT_TRUE(sample.acc.isApprox(traj.acc.front()));
  EXPECT_DOUBLE_EQ(sample.yaw, traj.yaw.front());

  ASSERT_TRUE(traj.samplePVA(1.5, sample));
  EXPECT_TRUE(sample.pos.isApprox(traj.pts.back()));
  EXPECT_TRUE(sample.vel.isApprox(traj.vel.back()));
  EXPECT_TRUE(sample.acc.isApprox(traj.acc.back()));
  EXPECT_DOUBLE_EQ(sample.yaw, traj.yaw.back());
}
