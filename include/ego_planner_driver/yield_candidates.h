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
  // 档位（implementation_plan_26082621 §6）：
  //   SLOW：scale {0.5, 0.3}；LEFT/RIGHT：lateral {1.0, 1.5, 2.0} × scale {0.7, 0.5}。
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
  // 轨迹 vs 邻机全时域同刻最小距离；conflict_center 记最近时刻；overlap 记
  // 时空重叠时段包络（dist < gate 的 t 范围，plateau 偏移窗口）。
  bool computeMinClearance(const TimedTrajectory& traj,
                           const std::vector<NeighborTrajectory>& neighbors,
                           double& min_clear, double& conflict_center,
                           double& overlap_start, double& overlap_end) const;
  // 全时域 min_clear >= 门槛 + 合法性（finite + 时间单调）。动力学由时间轴
  // 生成保证（速度 ≤1.5 驱动时间缩放），不做分解校验。
  bool validate(const TimedTrajectory& traj,
                const std::vector<NeighborTrajectory>& neighbors,
                double& min_clear) const;
  // 生成 SLOW 候选（时间缩放 scale ∈ {0.7, 0.5}，叠加速度驱动时间轴）。
  YieldCandidate makeSlow(const TimedTrajectory& remaining, double remaining_nominal,
                          const std::vector<NeighborTrajectory>& neighbors) const;
  // 生成 LEFT/RIGHT 候选：远离 peer 重叠时段平均位置的 plateau 偏移（覆盖时空
  // 重叠全程 + C2 过渡），叠加 scale_wait 减速与速度驱动时间轴。
  YieldCandidate makeLateral(const TimedTrajectory& remaining, double remaining_nominal,
                             const std::vector<NeighborTrajectory>& neighbors,
                             double overlap_start, double overlap_end,
                             YieldDirection dir) const;
  // 从 current_ 截取 t>=t_elapsed 剩余段并重基准到 0。
  static TimedTrajectory extractRemainder(const TimedTrajectory& current, double t_elapsed);
  // 整体时间缩放：t'=t/factor、vel'=vel·factor、acc'=acc·factor²；起点速度保持。
  static void applyTimeScale(TimedTrajectory& traj, double factor);

  YieldCandidateConfig config_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_