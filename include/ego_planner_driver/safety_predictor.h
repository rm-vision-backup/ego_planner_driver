#ifndef EGO_PLANNER_DRIVER_SAFETY_PREDICTOR_H_
#define EGO_PLANNER_DRIVER_SAFETY_PREDICTOR_H_

#include <Eigen/Eigen>
#include <limits>
#include <string>
#include <vector>

#include "ego_planner_driver/ego_planner_core.h"

namespace ego_planner_driver
{

enum class SafetyLevel { SAFE, WARNING, EMERGENCY, INSUFFICIENT_DATA };

struct SafetyPredictorConfig
{
  double protected_distance_m = 1.0;
  double prediction_horizon_s = 5.0;
  double sample_period_s = 0.05;
  double control_delay_s = 0.20;
  double local_compute_budget_s = 0.05;
  double peer_data_age_cap_s = 2.0;
  double emergency_decel_mps2 = 2.0;
  double warning_buffer_m = 0.35;
  double emergency_buffer_m = 0.10;
  double warning_ttc_s = 3.0;
  double emergency_ttc_s = 1.5;
  double minimum_prediction_window_s = 0.25;
};

struct PeerTrajectoryInput
{
  std::string uav_id;
  std::string exec_target;
  int32_t traj_id = 0;
  TimedTrajectory trajectory;
  double activation_epoch_s = 0.0;
  double receive_epoch_s = 0.0;
};

struct PairPrediction
{
  std::string peer_id;
  int32_t peer_traj_id = 0;
  SafetyLevel level = SafetyLevel::SAFE;
  double data_age_s = 0.0;
  double peer_activation_epoch_s = 0.0;
  double common_window_s = 0.0;
  double current_distance_m = std::numeric_limits<double>::infinity();
  double closing_speed_mps = 0.0;
  double t_cpa_s = 0.0;
  double cpa_distance_m = std::numeric_limits<double>::infinity();
  double ttc_s = std::numeric_limits<double>::infinity();
  double min_distance_m = std::numeric_limits<double>::infinity();
  double warning_distance_m = 0.0;
  double emergency_distance_m = 0.0;
};

struct SafetyPrediction
{
  SafetyLevel level = SafetyLevel::SAFE;
  std::string primary_peer_id;
  std::vector<PairPrediction> pairs;
};

class SafetyPredictor
{
public:
  explicit SafetyPredictor(const SafetyPredictorConfig& config);

  SafetyPrediction evaluate(const TimedTrajectory& self,
                            double self_activation_epoch_s,
                            const std::string& self_id,
                            const std::vector<PeerTrajectoryInput>& peers,
                            double now_epoch_s) const;

  static bool parseNumericId(const std::string& identity, int& numeric_id);
  static const char* levelName(SafetyLevel level);
  // 共享的动态距离制动余量：margin = closing*reaction + closing²/(2*decel)。
  // 监督门槛（evaluatePair）与规划软惩罚（EgoPlannerCore::computeSafeClearance）
  // 共用此函数，避免公式漂移（历史上出现过 age cap 与实际 age 不一致）。
  static double dynamicMargin(double closing_mps, double reaction_s,
                              double decel_mps2);

private:
  PairPrediction evaluatePair(const TimedTrajectory& self,
                              double self_activation_epoch_s,
                              const std::string& self_id,
                              const PeerTrajectoryInput& peer,
                              double now_epoch_s) const;

  SafetyPredictorConfig config_;
};

class ConflictEpisode
{
public:
  void update(const SafetyPrediction& prediction, double now_s,
              double release_hysteresis_s);
  bool active() const { return active_; }
  const std::string& peerId() const { return peer_id_; }

private:
  bool active_ = false;
  std::string peer_id_;
  double safe_since_s_ = -1.0;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_SAFETY_PREDICTOR_H_