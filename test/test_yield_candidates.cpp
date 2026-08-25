#include <gtest/gtest.h>

#include "ego_planner_driver/yield_candidates.h"

using namespace ego_planner_driver;

namespace
{
TimedTrajectory nominal()
{
  TimedTrajectory out;
  out.duration = 4.0;
  for (int i = 0; i <= 80; ++i)
  {
    const double t = i * 0.05;
    out.t.push_back(t);
    out.pts.emplace_back(t, 0.0, 0.0);
    out.vel.emplace_back(1.0, 0.0, 0.0);
    out.acc.push_back(Eigen::Vector3d::Zero());
    out.yaw.push_back(0.0);
  }
  return out;
}

NeighborTrajectory nearParallelPeer()
{
  NeighborTrajectory peer;
  peer.start_time = 0.0;
  peer.duration = 4.0;
  peer.clearance = 0.5;
  for (int i = 0; i <= 80; ++i)
  {
    const double t = i * 0.05;
    peer.t.push_back(t);
    peer.pts.emplace_back(t, 1.3, 0.0);
  }
  return peer;
}
}  // namespace

TEST(YieldCandidates, BuildsDeterministicFourManeuvers)
{
  YieldCandidateGenerator generator{YieldCandidateConfig()};
  const auto candidates = generator.generate(nominal(), {});
  ASSERT_EQ(candidates.size(), 4u);
  EXPECT_EQ(candidates[0].maneuver, YieldManeuver::LEFT);
  EXPECT_EQ(candidates[1].maneuver, YieldManeuver::RIGHT);
  EXPECT_EQ(candidates[2].maneuver, YieldManeuver::SLOW);
  EXPECT_EQ(candidates[3].maneuver, YieldManeuver::FAST);
  EXPECT_GT(candidates[0].trajectory.pts[40].y(), 1.0);
  EXPECT_LT(candidates[1].trajectory.pts[40].y(), -1.0);
  EXPECT_GT(candidates[2].trajectory.duration, 4.0);
  EXPECT_LT(candidates[3].trajectory.duration, 4.0);
  EXPECT_NEAR(candidates[0].trajectory.pts.front().y(), 0.0, 1e-12);
  EXPECT_NEAR(candidates[0].trajectory.pts.back().y(), 0.0, 1e-12);
}

TEST(YieldCandidates, SelectsSafeManeuverForNearParallelPeer)
{
  YieldCandidateConfig config;
  config.max_jerk_mps3 = 20.0;
  YieldCandidateGenerator generator(config);
  // 平行近距 peer（横向 1.3m、closing≈0）：nominal 落在 warning 区但候选
  // 可在不越动态 emergency 边界前提下选出安全让路轨迹（缺陷 3 修复后语义：
  // 交叉高 closing 场景无安全候选时应转制动，不再用旧 1m-only 判定）。
  const auto candidates = generator.generate(nominal(), {nearParallelPeer()});
  YieldCandidate selected;
  std::string reasons;
  for (const auto& candidate : candidates)
    reasons += std::string(YieldCandidateGenerator::maneuverName(candidate.maneuver)) +
        ":" + candidate.rejection_reason + " ";
  ASSERT_TRUE(generator.chooseBest(candidates, selected)) << reasons;
  EXPECT_TRUE(selected.feasible);
  EXPECT_GE(selected.minimum_clearance_m, 1.0);
}

TEST(YieldCandidates, StableScoreUsesEnumerationOrderOnExactTie)
{
  YieldCandidateGenerator generator{YieldCandidateConfig()};
  YieldCandidate left, right, selected;
  left.maneuver = YieldManeuver::LEFT;
  right.maneuver = YieldManeuver::RIGHT;
  left.feasible = right.feasible = true;
  left.minimum_clearance_m = right.minimum_clearance_m = 2.0;
  left.arrival_time_delta_s = right.arrival_time_delta_s = 0.0;
  left.lateral_offset_m = right.lateral_offset_m = 2.2;
  ASSERT_TRUE(generator.chooseBest({left, right}, selected));
  EXPECT_EQ(selected.maneuver, YieldManeuver::LEFT);
}

TEST(YieldCandidates, ReportsAllFailedWhenPeerOccupiesStart)
{
  NeighborTrajectory peer;
  peer.start_time = 0.0;
  peer.duration = 4.0;
  peer.t = {0.0, 4.0};
  peer.pts = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  YieldCandidateGenerator generator{YieldCandidateConfig()};
  const auto candidates = generator.generate(nominal(), {peer});
  YieldCandidate selected;
  EXPECT_FALSE(generator.chooseBest(candidates, selected));
  for (const auto& candidate : candidates)
    EXPECT_FALSE(candidate.feasible);
}

TEST(YieldCandidates, RejectsCandidateInsideDynamicEmergencyBoundary)
{
  YieldCandidateConfig config;
  YieldCandidateGenerator generator(config);
  // 对向 peer：横向 1.2m、closing≈1.9m/s。1m clearance 检查能通过（最近
  // 距离 1.2m），但动态 emergency 门槛（reaction≈2.25s、emergency_dist≈6.3m）
  // 必须拒绝候选，否则提交后下一 safety tick 会立即被判 EMERGENCY（缺陷 3）。
  NeighborTrajectory peer;
  peer.start_time = 0.0;
  peer.duration = 4.0;
  peer.clearance = 0.5;
  for (int i = 0; i <= 80; ++i)
  {
    const double t = i * 0.05;
    peer.t.push_back(t);
    peer.pts.emplace_back(4.0 - t, -1.2, 0.0);
  }
  const auto candidates = generator.generate(nominal(), {peer});
  YieldCandidate selected;
  EXPECT_FALSE(generator.chooseBest(candidates, selected));
  for (const auto& candidate : candidates)
  {
    EXPECT_FALSE(candidate.feasible);
    EXPECT_EQ(candidate.rejection_reason, "emergency_boundary");
  }
}

TEST(YieldCandidates, AllowsCandidateOutsideDynamicEmergencyBoundary)
{
  YieldCandidateConfig config;
  YieldCandidateGenerator generator(config);
  // 同向远距 peer（5m、closing≈0）：动态门槛约 1.1m，候选不受限，且仍满足
  // 1m clearance。
  NeighborTrajectory peer;
  peer.start_time = 0.0;
  peer.duration = 4.0;
  peer.clearance = 0.5;
  for (int i = 0; i <= 80; ++i)
  {
    const double t = i * 0.05;
    peer.t.push_back(t);
    peer.pts.emplace_back(t, 5.0, 0.0);
  }
  const auto candidates = generator.generate(nominal(), {peer});
  YieldCandidate selected;
  ASSERT_TRUE(generator.chooseBest(candidates, selected));
  EXPECT_TRUE(selected.feasible);
  EXPECT_GE(selected.minimum_clearance_m, 1.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}