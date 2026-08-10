#include <gtest/gtest.h>
#include <cmath>
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
EgoPlannerConfig makeConfig()
{
  EgoPlannerConfig config;
  config.ts = 0.4;
  config.max_vel = 8.0;
  config.max_acc = 6.0;
  config.swarm_clearance = 2.0;
  config.planning_range = 30.0;
  config.max_iterations = 200;
  return config;
}

NeighborTrajectory straightNeighbor(double sx, double sy, double ex, double ey)
{
  NeighborTrajectory n;
  n.clearance = 2.0;
  n.start_time = 0.0;
  n.duration = 10.0;
  for (int i = 0; i < 100; ++i)
  {
    const double a = static_cast<double>(i) / 99.0;
    n.t.push_back(a * 10.0);
    n.pts.emplace_back(sx + (ex - sx) * a, sy + (ey - sy) * a, 12.0);
  }
  return n;
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
  EgoPlannerCore core(makeConfig());
  GoalPoint start{Eigen::Vector3d(0.0, 0.0, 15.0), 0.0};
  std::vector<WayPoint> waypoints{WayPoint{Eigen::Vector3d(0.0, 10.0, 15.0), 0.0}};
  TimedTrajectory trajectory;
  double cost = 0.0;
  ASSERT_TRUE(core.planOnce(start, waypoints, {}, trajectory, cost));
  std::vector<NeighborTrajectory> neighbors{straightNeighbor(0.0, 5.0, 0.0, 5.0)};
  EXPECT_FALSE(core.checkClearance(trajectory, neighbors, 0.5));
  EXPECT_TRUE(core.checkClearance(trajectory, {}, 0.5));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
</｜｜DSML｜｜>
</write_to_file>