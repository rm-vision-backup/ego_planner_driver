#ifndef EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_
#define EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_

#include <Eigen/Eigen>
#include <limits>
#include <string>
#include <vector>

#include "ego_planner_driver/ego_planner_core.h"

namespace ego_planner_driver
{

struct YieldCandidateConfig
{
  // 安全门槛：候选 min_clear >= clearance_factor * 2 * swarm_clearance。
  // 1.3 提供轨迹规划通过后的机间距离抖动缓冲，防止触达 1m 硬门后 BRAKE_HOLD。
  double clearance_factor = 1.3;
  double lateral_max_m = 2.0;   // LEFT/RIGHT 峰值横向偏移上限
  double slow_min_scale = 0.3;  // SLOW 最低时间缩放（速度 ×scale，时长 ÷scale）
  double max_velocity_mps = 1.5;
  double max_acceleration_mps2 = 6.0;
  double hovering_velocity_mps = 0.1;  // 邻机末速 < 该值判"近似悬停"→ 跳过 SLOW
};

enum class YieldDirection { NONE, LEFT, RIGHT };

struct YieldCandidate
{
  enum class Type { SLOW, LEFT, RIGHT };
  Type type = Type::SLOW;
  YieldDirection direction = YieldDirection::NONE;
  TimedTrajectory trajectory;  // 相对提交时刻的时间轴（t[0]=0）
  double min_clearance_m = 0.0;
  double time_extension_s = 0.0;  // SLOW 延长的秒数
  double lateral_dev_m = 0.0;     // LEFT/RIGHT 峰值偏移
  double cost = std::numeric_limits<double>::infinity();
};

// 机间让路候选生成器（implementation_plan_26082602 §10）：
//   SLOW   ：几何不变，时间轴缩放（t'=t/scale, vel'=vel·scale, acc'=acc·scale²）
//   LEFT/RIGHT：横向 C2 法向包络偏移（时间轴不变，解析 P/V/A 叠加）
// 仅处理机间避碰（邻机轨迹同刻采样），不涉及环境障碍。
class YieldCandidateGenerator
{
public:
  explicit YieldCandidateGenerator(const YieldCandidateConfig& config);

  // 基于本机执行轨迹 current_（相对激活时刻）在 t_elapsed 时刻生成候选。
  // neighbors 已 rebase 到 current_ 时间轴（start_time 相对 current_ 激活）。
  // peer_hovering：primary peer 近似悬停 → 跳过 SLOW。
  // 返回按 cost 升序的可行候选（空 = 无解）。
  std::vector<YieldCandidate> generate(const TimedTrajectory& current,
                                       double t_elapsed,
                                       const std::vector<NeighborTrajectory>& neighbors,
                                       bool peer_hovering) const;

private:
  // 轨迹 vs 邻机全时域同刻最小距离；conflict_center 记最近时刻（相对轨迹起点）。
  bool computeMinClearance(const TimedTrajectory& traj,
                           const std::vector<NeighborTrajectory>& neighbors,
                           double& min_clear, double& conflict_center) const;
  // 全时域 min_clear >= 门槛 + 动力学 + finite 验证。
  bool validate(const TimedTrajectory& traj,
                const std::vector<NeighborTrajectory>& neighbors,
                double& min_clear) const;
  // 生成 SLOW 候选（scale 档位从 min 到 1.0 尝试，首个可行）。
  YieldCandidate makeSlow(const TimedTrajectory& remaining, double remaining_nominal,
                          const std::vector<NeighborTrajectory>& neighbors) const;
  // 生成 LEFT/RIGHT 候选（lateral 档位从 0.5 到 max 尝试，首个可行）。
  YieldCandidate makeLateral(const TimedTrajectory& remaining, double remaining_nominal,
                             const std::vector<NeighborTrajectory>& neighbors,
                             double conflict_center, YieldDirection dir) const;
  // 从 current_ 截取 t>=t_elapsed 剩余段并重基准到 0。
  static TimedTrajectory extractRemainder(const TimedTrajectory& current, double t_elapsed);

  YieldCandidateConfig config_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_