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

  YieldCandidateConfig config_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_YIELD_CANDIDATES_H_