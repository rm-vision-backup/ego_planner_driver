#include <gtest/gtest.h>

#include "ego_planner_driver/emergency_brake.h"

using namespace ego_planner_driver;

namespace
{
void expectBrake(double speed, const Eigen::Vector3d& direction,
                 const Eigen::Vector3d& acceleration = Eigen::Vector3d::Zero())
{
  EmergencyBrakeConfig config;
  EmergencyBrakeGenerator generator(config);
  TrajectorySample start;
  start.pos = Eigen::Vector3d(1.0, -2.0, 3.0);
  start.vel = direction.normalized() * speed;
  start.acc = acceleration;
  start.yaw = 0.7;
  TimedTrajectory trajectory;
  ASSERT_TRUE(generator.generate(start, trajectory));
  ASSERT_TRUE(generator.validate(trajectory));
  EXPECT_LT((trajectory.pts.front() - start.pos).norm(), 1e-8);
  EXPECT_LT((trajectory.vel.front() - start.vel).norm(), 1e-8);
  EXPECT_LT((trajectory.acc.front() - start.acc).norm(), 1e-8);
  EXPECT_LT(trajectory.vel.back().norm(), 1e-4);
  EXPECT_LT(trajectory.acc.back().norm(), 1e-4);
  EXPECT_NEAR(trajectory.yaw.back(), start.yaw, 1e-12);
}
}  // namespace

TEST(EmergencyBrake, HandlesZeroAndConfiguredSpeeds)
{
  expectBrake(0.0, Eigen::Vector3d::UnitX());
  expectBrake(0.2, Eigen::Vector3d::UnitX());
  expectBrake(1.0, Eigen::Vector3d::UnitY());
  expectBrake(1.5, Eigen::Vector3d(-1.0, 1.0, 0.2));
}

TEST(EmergencyBrake, PreservesNonzeroInitialAcceleration)
{
  expectBrake(1.0, Eigen::Vector3d::UnitX(), Eigen::Vector3d(-0.2, 0.1, 0.0));
}

TEST(EmergencyBrake, RejectsNonfiniteAndOutOfRangeStart)
{
  EmergencyBrakeGenerator generator{EmergencyBrakeConfig()};
  TrajectorySample start;
  start.pos.x() = std::numeric_limits<double>::quiet_NaN();
  TimedTrajectory trajectory;
  EXPECT_FALSE(generator.generate(start, trajectory));
  start.pos.setZero();
  start.vel = Eigen::Vector3d(2.0, 0.0, 0.0);
  EXPECT_FALSE(generator.generate(start, trajectory));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}