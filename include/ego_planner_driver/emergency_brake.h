#ifndef EGO_PLANNER_DRIVER_EMERGENCY_BRAKE_H_
#define EGO_PLANNER_DRIVER_EMERGENCY_BRAKE_H_

#include "ego_planner_driver/ego_planner_core.h"

namespace ego_planner_driver
{

struct EmergencyBrakeConfig
{
  double max_velocity_mps = 1.5;
  double max_acceleration_mps2 = 6.0;
  double max_jerk_mps3 = 8.0;
  double nominal_deceleration_mps2 = 2.0;
  double sample_period_s = 0.02;
  double minimum_duration_s = 0.4;
  double maximum_duration_s = 12.0;
};

class EmergencyBrakeGenerator
{
public:
  explicit EmergencyBrakeGenerator(const EmergencyBrakeConfig& config);

  bool generate(const TrajectorySample& start, TimedTrajectory& output) const;
  bool validate(const TimedTrajectory& trajectory) const;

private:
  EmergencyBrakeConfig config_;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_EMERGENCY_BRAKE_H_