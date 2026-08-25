#ifndef EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_
#define EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_

#include <string>
#include <vector>

#include "ego_planner_driver/ego_planner_core.h"

namespace ego_planner_driver
{

enum class YieldManeuver { LEFT, RIGHT, SLOW, FAST };

struct YieldCandidateConfig
{
  double lateral_offset_m = 2.2;
  double lateral_time_scale = 1.5;
  double slow_time_scale = 1.5;
  double fast_time_scale = 0.8;
  double max_velocity_mps = 1.5;
  double max_acceleration_mps2 = 6.0;
  double max_jerk_mps3 = 12.0;
  double clearance_m = 1.0;
  // 动态 emergency 门槛（与 SafetyPredictor 对齐）。候选在提交前必须在全时域
  // 满足"不进入 emergency 边界"，否则提交后下一 safety tick 会立即被判
  // EMERGENCY 而触发制动（缺陷 3）。reaction 用 peer_data_age_cap 上界，
  // 保证 worst-case 数据年龄下仍安全。
  double protected_distance_m = 1.0;
  double control_delay_s = 0.20;
  double local_compute_budget_s = 0.05;
  double peer_data_age_cap_s = 2.0;
  double emergency_decel_mps2 = 2.0;
  double emergency_buffer_m = 0.10;
};

struct YieldCandidate
{
  YieldManeuver maneuver = YieldManeuver::LEFT;
  TimedTrajectory trajectory;
  bool feasible = false;
  std::string rejection_reason;
  double minimum_clearance_m = 0.0;
  double arrival_time_delta_s = 0.0;
  double lateral_offset_m = 0.0;
};

class YieldCandidateGenerator
{
public:
  explicit YieldCandidateGenerator(const YieldCandidateConfig& config);

  std::vector<YieldCandidate> generate(
      const TimedTrajectory& nominal,
      const std::vector<NeighborTrajectory>& peers) const;
  bool chooseBest(const std::vector<YieldCandidate>& candidates,
                  YieldCandidate& selected) const;
  bool validate(const TimedTrajectory& trajectory,
                const std::vector<NeighborTrajectory>& peers,
                double& minimum_clearance,
                std::string& reason) const;

  static const char* maneuverName(YieldManeuver maneuver);

private:
  TimedTrajectory lateral(const TimedTrajectory& nominal, double sign) const;
  TimedTrajectory retime(const TimedTrajectory& nominal, double scale) const;
  static Eigen::Vector3d samplePeer(const NeighborTrajectory& peer, double t);
  static Eigen::Vector3d samplePeerVelocity(const NeighborTrajectory& peer,
                                            double t);

  YieldCandidateConfig config_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_