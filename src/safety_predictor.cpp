#include "ego_planner_driver/safety_predictor.h"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace ego_planner_driver
{
namespace
{
bool samplePosition(const TimedTrajectory& trajectory, double activation_s,
                    double epoch_s, Eigen::Vector3d& position)
{
  if (!std::isfinite(epoch_s) || !std::isfinite(activation_s))
    return false;
  return trajectory.sample(epoch_s - activation_s, position) && position.allFinite();
}

Eigen::Vector3d sampleVelocity(const TimedTrajectory& trajectory,
                               double activation_s, double epoch_s)
{
  TrajectorySample sample;
  if (trajectory.samplePVA(epoch_s - activation_s, sample) && sample.vel.allFinite())
    return sample.vel;
  const double dt = 0.02;
  Eigen::Vector3d first, second;
  const double relative = epoch_s - activation_s;
  if (relative <= dt)
  {
    if (samplePosition(trajectory, activation_s, epoch_s, first) &&
        samplePosition(trajectory, activation_s, epoch_s + dt, second))
      return (second - first) / dt;
  }
  else if (relative >= trajectory.duration - dt)
  {
    if (samplePosition(trajectory, activation_s, epoch_s - dt, first) &&
        samplePosition(trajectory, activation_s, epoch_s, second))
      return (second - first) / dt;
  }
  else if (samplePosition(trajectory, activation_s, epoch_s - dt, first) &&
           samplePosition(trajectory, activation_s, epoch_s + dt, second))
  {
    return (second - first) / (2.0 * dt);
  }
  return Eigen::Vector3d::Zero();
}

int severity(SafetyLevel level)
{
  if (level == SafetyLevel::EMERGENCY) return 3;
  if (level == SafetyLevel::WARNING) return 2;
  if (level == SafetyLevel::INSUFFICIENT_DATA) return 1;
  return 0;
}
}  // namespace

SafetyPredictor::SafetyPredictor(const SafetyPredictorConfig& config)
    : config_(config)
{
}

bool SafetyPredictor::parseNumericId(const std::string& identity, int& numeric_id)
{
  if (identity.empty()) return false;
  size_t first_digit = identity.size();
  while (first_digit > 0 && std::isdigit(static_cast<unsigned char>(identity[first_digit - 1])))
    --first_digit;
  if (first_digit == identity.size()) return false;
  for (size_t i = 0; i < first_digit; ++i)
    if (!std::isalnum(static_cast<unsigned char>(identity[i])) && identity[i] != '_' && identity[i] != '-')
      return false;
  try
  {
    const long value = std::stol(identity.substr(first_digit));
    if (value < 0 || value > std::numeric_limits<int>::max()) return false;
    numeric_id = static_cast<int>(value);
    return true;
  }
  catch (...)
  {
    return false;
  }
}

PairRole SafetyPredictor::determineRole(const std::string& self_id,
                                        const std::string& peer_id)
{
  int self = -1, peer = -1;
  if (!parseNumericId(self_id, self) || !parseNumericId(peer_id, peer) || self == peer)
    return PairRole::UNKNOWN;
  return self < peer ? PairRole::STAND_ON : PairRole::YIELD;
}

PairPrediction SafetyPredictor::evaluatePair(
    const TimedTrajectory& self, double self_activation_epoch_s,
    const std::string& self_id, const PeerTrajectoryInput& peer,
    double now_epoch_s) const
{
  PairPrediction out;
  out.peer_id = peer.uav_id.empty() ? peer.exec_target : peer.uav_id;
  out.peer_traj_id = peer.traj_id;
  out.role = determineRole(self_id, out.peer_id);
  out.data_age_s = std::max(0.0, now_epoch_s - peer.receive_epoch_s);

  const double self_end = self_activation_epoch_s + self.duration;
  const double peer_end = peer.activation_epoch_s + peer.trajectory.duration;
  const double begin = std::max(now_epoch_s,
      std::max(self_activation_epoch_s, peer.activation_epoch_s));
  const double end = std::min(begin + std::max(0.0, config_.prediction_horizon_s),
                              std::min(self_end, peer_end));
  out.common_window_s = std::max(0.0, end - begin);
  if (out.common_window_s < config_.minimum_prediction_window_s)
  {
    out.level = SafetyLevel::INSUFFICIENT_DATA;
    return out;
  }

  Eigen::Vector3d self_pos, peer_pos;
  if (!samplePosition(self, self_activation_epoch_s, begin, self_pos) ||
      !samplePosition(peer.trajectory, peer.activation_epoch_s, begin, peer_pos))
  {
    out.level = SafetyLevel::INSUFFICIENT_DATA;
    return out;
  }
  const Eigen::Vector3d self_vel = sampleVelocity(self, self_activation_epoch_s, begin);
  const Eigen::Vector3d peer_vel = sampleVelocity(peer.trajectory, peer.activation_epoch_s, begin);
  const Eigen::Vector3d r = peer_pos - self_pos;
  const Eigen::Vector3d v_rel = peer_vel - self_vel;
  out.current_distance_m = r.norm();
  if (out.current_distance_m > 1e-9)
    out.closing_speed_mps = std::max(0.0, -r.dot(v_rel) / out.current_distance_m);
  const double rel_speed_sq = v_rel.squaredNorm();
  out.t_cpa_s = rel_speed_sq > 1e-12
      ? std::max(0.0, std::min(out.common_window_s, -r.dot(v_rel) / rel_speed_sq)) : 0.0;
  out.cpa_distance_m = (r + v_rel * out.t_cpa_s).norm();

  const double age = std::min(out.data_age_s,
      std::max(0.0, config_.peer_data_age_cap_s));
  const double reaction = std::max(0.0, config_.control_delay_s) +
      std::max(0.0, config_.local_compute_budget_s) + age;
  const double decel = std::max(1e-6, config_.emergency_decel_mps2);
  const double margin = out.closing_speed_mps * reaction +
      out.closing_speed_mps * out.closing_speed_mps / (2.0 * decel);
  out.warning_distance_m = config_.protected_distance_m + margin +
      std::max(0.0, config_.warning_buffer_m);
  out.emergency_distance_m = config_.protected_distance_m + margin +
      std::max(0.0, config_.emergency_buffer_m);

  const double dt = std::max(0.01, config_.sample_period_s);
  bool warning = false, emergency = false;
  for (double epoch = begin; epoch <= end + 1e-9; epoch += dt)
  {
    if (!samplePosition(self, self_activation_epoch_s, std::min(epoch, end), self_pos) ||
        !samplePosition(peer.trajectory, peer.activation_epoch_s, std::min(epoch, end), peer_pos))
      continue;
    const double distance = (peer_pos - self_pos).norm();
    out.min_distance_m = std::min(out.min_distance_m, distance);
    const double relative_t = std::min(epoch, end) - begin;
    if (!std::isfinite(out.ttc_s) && distance <= config_.protected_distance_m)
      out.ttc_s = relative_t;
    warning = warning || distance <= out.warning_distance_m;
    emergency = emergency || distance <= out.emergency_distance_m;
  }
  warning = warning || (std::isfinite(out.ttc_s) && out.ttc_s <= config_.warning_ttc_s);
  emergency = emergency || (std::isfinite(out.ttc_s) && out.ttc_s <= config_.emergency_ttc_s);
  out.level = emergency ? SafetyLevel::EMERGENCY
                        : (warning ? SafetyLevel::WARNING : SafetyLevel::SAFE);
  return out;
}

SafetyPrediction SafetyPredictor::evaluate(
    const TimedTrajectory& self, double self_activation_epoch_s,
    const std::string& self_id, const std::vector<PeerTrajectoryInput>& peers,
    double now_epoch_s) const
{
  SafetyPrediction result;
  int best_severity = -1;
  double best_ttc = std::numeric_limits<double>::infinity();
  double best_distance = std::numeric_limits<double>::infinity();
  for (const PeerTrajectoryInput& peer : peers)
  {
    PairPrediction pair = evaluatePair(self, self_activation_epoch_s,
                                       self_id, peer, now_epoch_s);
    const int pair_severity = severity(pair.level);
    if (pair_severity > best_severity ||
        (pair_severity == best_severity && pair.ttc_s < best_ttc) ||
        (pair_severity == best_severity && pair.ttc_s == best_ttc &&
         pair.min_distance_m < best_distance))
    {
      best_severity = pair_severity;
      best_ttc = pair.ttc_s;
      best_distance = pair.min_distance_m;
      result.primary_peer_id = pair.peer_id;
      result.level = pair.level;
    }
    result.pairs.push_back(pair);
  }
  return result;
}

const char* SafetyPredictor::levelName(SafetyLevel level)
{
  switch (level)
  {
    case SafetyLevel::SAFE: return "SAFE";
    case SafetyLevel::WARNING: return "WARNING";
    case SafetyLevel::EMERGENCY: return "EMERGENCY";
    case SafetyLevel::INSUFFICIENT_DATA: return "INSUFFICIENT_DATA";
  }
  return "UNKNOWN";
}

const char* SafetyPredictor::roleName(PairRole role)
{
  switch (role)
  {
    case PairRole::STAND_ON: return "STAND_ON";
    case PairRole::YIELD: return "YIELD";
    case PairRole::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

void ConflictEpisode::update(const SafetyPrediction& prediction, double now_s,
                             double release_hysteresis_s)
{
  if (prediction.level == SafetyLevel::WARNING ||
      prediction.level == SafetyLevel::EMERGENCY)
  {
    if (!active_)
    {
      active_ = true;
      peer_id_ = prediction.primary_peer_id;
      const auto it = std::find_if(prediction.pairs.begin(), prediction.pairs.end(),
          [this](const PairPrediction& pair) { return pair.peer_id == peer_id_; });
      role_ = it == prediction.pairs.end() ? PairRole::UNKNOWN : it->role;
    }
    safe_since_s_ = -1.0;
    return;
  }
  if (!active_) return;
  if (safe_since_s_ < 0.0) safe_since_s_ = now_s;
  if (now_s - safe_since_s_ >= std::max(0.0, release_hysteresis_s))
  {
    active_ = false;
    peer_id_.clear();
    role_ = PairRole::UNKNOWN;
    safe_since_s_ = -1.0;
  }
}

}  // namespace ego_planner_driver