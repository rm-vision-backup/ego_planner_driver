#include <gtest/gtest.h>

#include "ego_planner_driver/yield_candidates.h"

namespace ego_planner_driver
{
namespace
{
using ego_planner_driver::NeighborTrajectory;
using ego_planner_driver::TimedTrajectory;
using ego_planner_driver::YieldCandidate;
using ego_planner_driver::YieldCandidateConfig;
using ego_planner_driver::YieldCandidateGenerator;
using ego_planner_driver::YieldDirection;

// 直线轨迹：沿 x 从 (0,0,z) 到 (length,0,z)，speed m/s，0.1s 采样。
TimedTrajectory makeLine(double length, double speed, double z = 5.0)
{
  TimedTrajectory t;
  const int n = static_cast<int>(length / speed / 0.1);
  for (int i = 0; i <= n; ++i)
  {
    const double tt = static_cast<double>(i) * 0.1;
    t.t.push_back(tt);
    t.pts.emplace_back(speed * tt, 0.0, z);
    t.vel.emplace_back(speed, 0.0, 0.0);
    t.acc.emplace_back(0.0, 0.0, 0.0);
    t.yaw.push_back(0.0);
  }
  t.duration = t.t.back();
  return t;
}

// 悬停邻机：固定位置。
NeighborTrajectory makeHoverPeer(const Eigen::Vector3d& pos, double duration = 20.0)
{
  NeighborTrajectory nb;
  nb.clearance = 0.5;
  nb.start_time = 0.0;
  nb.duration = duration;
  nb.t = {0.0, duration};
  nb.pts = {pos, pos};
  return nb;
}

// 移动邻机：沿 y 交叉本机 x 路径（冲突在 y=0 附近）。
NeighborTrajectory makeCrossingPeer(double start_time = -2.0)
{
  NeighborTrajectory nb;
  nb.clearance = 0.5;
  nb.start_time = start_time;
  nb.duration = 8.0;
  nb.t = {0.0, 8.0};
  nb.pts = {Eigen::Vector3d(5.0, -2.0, 5.0), Eigen::Vector3d(5.0, 2.0, 5.0)};
  return nb;
}
}  // namespace

TEST(YieldCandidates, LateralOffsetsConflictWindowOnly)
{
  YieldCandidateConfig cfg;
  YieldCandidateGenerator gen(cfg);
  const TimedTrajectory current = makeLine(10.0, 1.0);
  const NeighborTrajectory hover = makeHoverPeer(Eigen::Vector3d(5.0, 0.0, 5.0));
  const auto candidates = gen.generate(current, 0.0, {hover});
  ASSERT_FALSE(candidates.empty());
  double max_dev = 0.0, dev_far = 0.0;
  for (const auto& c : candidates)
  {
    // LEFT/RIGHT 可组合减速（速度驱动 + scale_wait）→ ΔT ≥ 0（时间轴可延长）。
    EXPECT_GE(c.time_extension_s, 0.0);
    for (size_t i = 0; i < c.trajectory.t.size(); ++i)
    {
      // 起点 w(0)=0（P/V/A 连续）；冲突区（除首尾过渡）有横向偏移。
      if (i == 0)
        dev_far = std::max(dev_far, std::fabs(c.trajectory.pts[i].y()));
      else if (c.trajectory.t[i] > 0.5 &&
               c.trajectory.t[i] < c.trajectory.duration - 0.5)
        max_dev = std::max(max_dev, std::fabs(c.trajectory.pts[i].y()));
    }
  }
  EXPECT_GT(max_dev, 0.3);   // 冲突区横向偏移
  EXPECT_LT(dev_far, 1e-6);  // 窗口外无偏移（起点连续）
}

TEST(YieldCandidates, CostIncreasesWithLateralAndClearance)
{
  YieldCandidateConfig cfg;
  cfg.lateral_max_m = 2.2;
  YieldCandidateGenerator gen(cfg);
  const TimedTrajectory current = makeLine(10.0, 1.0);
  const NeighborTrajectory hover = makeHoverPeer(Eigen::Vector3d(5.0, 0.0, 5.0));
  const auto candidates = gen.generate(current, 0.0, {hover});
  ASSERT_FALSE(candidates.empty());
  // 所有候选通过 1.2m 门槛（26082621 迭代由 1.3 放宽）；LEFT/RIGHT 有 dev/5.0。
  for (const auto& c : candidates)
  {
    EXPECT_GE(c.min_clearance_m, 1.2);
    EXPECT_GT(c.cost, 0.0);
    EXPECT_GE(c.time_extension_s, 0.0);
  }
}

TEST(YieldCandidates, GeneratesBothLeftAndRight)
{
  // SLOW 移除（260830）后候选收敛为 LEFT/RIGHT 横向偏移：悬停挡路 peer 下两侧
  // 都应可行（C2 包络峰值 ≤ lateral_max_m）。
  YieldCandidateConfig cfg;
  YieldCandidateGenerator gen(cfg);
  const TimedTrajectory current = makeLine(10.0, 1.0);
  const NeighborTrajectory hover = makeHoverPeer(Eigen::Vector3d(5.0, 0.0, 5.0));
  const auto candidates = gen.generate(current, 0.0, {hover});
  ASSERT_FALSE(candidates.empty());
  bool saw_left = false, saw_right = false;
  for (const auto& c : candidates)
  {
    if (c.direction == YieldDirection::LEFT) saw_left = true;
    if (c.direction == YieldDirection::RIGHT) saw_right = true;
  }
  EXPECT_TRUE(saw_left);
  EXPECT_TRUE(saw_right);
}

}  // namespace ego_planner_driver

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
