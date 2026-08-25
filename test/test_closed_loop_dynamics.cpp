#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "ego_planner_driver/emergency_brake.h"
#include "ego_planner_driver/safety_predictor.h"

using namespace ego_planner_driver;

namespace
{
constexpr double kProtectedDistanceM = 1.0;

struct ClosedLoopConfig
{
  double dt_s = 0.01;
  double control_delay_s = 0.30;
  double actuator_time_constant_s = 0.25;
  double peer_intent_age_s = 0.80;
  double position_gain = 2.0;
  double velocity_gain = 2.4;
  double acceleration_limit_mps2 = 6.0;
};

struct Plant
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  std::deque<Eigen::Vector3d> delayed_commands;
};

TimedTrajectory linearTrajectory(const Eigen::Vector3d& start,
                                 const Eigen::Vector3d& finish,
                                 double duration, double dt = 0.02)
{
  TimedTrajectory out;
  out.duration = duration;
  const Eigen::Vector3d velocity = (finish - start) / duration;
  const size_t samples = static_cast<size_t>(std::ceil(duration / dt)) + 1;
  for (size_t i = 0; i < samples; ++i)
  {
    const double t = i + 1 == samples ? duration
        : duration * static_cast<double>(i) / static_cast<double>(samples - 1);
    out.t.push_back(t);
    out.pts.push_back(start + velocity * t);
    out.vel.push_back(velocity);
    out.acc.push_back(Eigen::Vector3d::Zero());
    out.yaw.push_back(std::atan2(velocity.y(), velocity.x()));
  }
  return out;
}

NeighborTrajectory asNeighbor(const TimedTrajectory& trajectory)
{
  NeighborTrajectory peer;
  peer.start_time = 0.0;
  peer.duration = trajectory.duration;
  peer.t = trajectory.t;
  peer.pts = trajectory.pts;
  peer.clearance = 0.5;
  return peer;
}

PeerTrajectoryInput asPeerInput(const std::string& id,
                                const TimedTrajectory& trajectory,
                                const ClosedLoopConfig& config)
{
  PeerTrajectoryInput peer;
  peer.uav_id = id;
  peer.exec_target = id;
  peer.traj_id = 1;
  peer.trajectory = trajectory;
  peer.activation_epoch_s = 0.0;
  peer.receive_epoch_s = -config.peer_intent_age_s;
  return peer;
}

Eigen::Vector3d bounded(const Eigen::Vector3d& value, double limit)
{
  const double norm = value.norm();
  return norm > limit ? value * (limit / norm) : value;
}

void initializePlant(const TimedTrajectory& trajectory, Plant& plant)
{
  TrajectorySample start;
  ASSERT_TRUE(trajectory.samplePVA(0.0, start));
  plant.pos = start.pos;
  plant.vel = start.vel;
  plant.acc = start.acc;
}

void stepPlant(const TimedTrajectory& command, double time,
               const ClosedLoopConfig& config, Plant& plant)
{
  TrajectorySample reference;
  ASSERT_TRUE(command.samplePVA(std::min(time, command.duration), reference));
  Eigen::Vector3d desired = reference.acc +
      config.position_gain * (reference.pos - plant.pos) +
      config.velocity_gain * (reference.vel - plant.vel);
  plant.delayed_commands.push_back(bounded(desired, config.acceleration_limit_mps2));
  const size_t delay_steps = static_cast<size_t>(
      std::ceil(config.control_delay_s / config.dt_s));
  Eigen::Vector3d delayed = Eigen::Vector3d::Zero();
  if (plant.delayed_commands.size() > delay_steps)
  {
    delayed = plant.delayed_commands.front();
    plant.delayed_commands.pop_front();
  }
  const double alpha = std::min(1.0,
      config.dt_s / config.actuator_time_constant_s);
  plant.acc += alpha * (delayed - plant.acc);
  plant.acc = bounded(plant.acc, config.acceleration_limit_mps2);
  plant.vel += plant.acc * config.dt_s;
  plant.pos += plant.vel * config.dt_s;
}

double simulatePair(const std::string& scenario,
                    const TimedTrajectory& first_command,
                    const TimedTrajectory& second_command,
                    const ClosedLoopConfig& config)
{
  Plant first, second;
  initializePlant(first_command, first);
  initializePlant(second_command, second);
  double minimum = (first.pos - second.pos).norm();
  const double end = std::max(first_command.duration, second_command.duration) + 2.0;
  for (double time = 0.0; time <= end; time += config.dt_s)
  {
    stepPlant(first_command, time, config, first);
    stepPlant(second_command, time, config, second);
    minimum = std::min(minimum, (first.pos - second.pos).norm());
  }
  std::cout << "[stage_e_closed_loop] scenario=" << scenario
            << " min_center_distance_m=" << minimum
            << " control_delay_s=" << config.control_delay_s
            << " actuator_tau_s=" << config.actuator_time_constant_s
            << " peer_intent_age_s=" << config.peer_intent_age_s << std::endl;
  return minimum;
}

SafetyPredictor makePredictor(const ClosedLoopConfig& config)
{
  SafetyPredictorConfig predictor;
  predictor.protected_distance_m = kProtectedDistanceM;
  predictor.control_delay_s = config.control_delay_s;
  predictor.local_compute_budget_s = 0.05;
  predictor.peer_data_age_cap_s = 2.0;
  predictor.emergency_decel_mps2 = 2.0;
  predictor.warning_buffer_m = 0.35;
  predictor.emergency_buffer_m = 0.10;
  predictor.warning_ttc_s = 3.0;
  predictor.emergency_ttc_s = 1.5;
  return SafetyPredictor(predictor);
}

EmergencyBrakeGenerator makeBrakeGenerator()
{
  EmergencyBrakeConfig config;
  config.max_velocity_mps = 1.5;
  config.max_acceleration_mps2 = 6.0;
  config.max_jerk_mps3 = 8.0;
  config.nominal_deceleration_mps2 = 2.0;
  config.sample_period_s = 0.02;
  return EmergencyBrakeGenerator(config);
}

// 从轨迹的 elapsed 时刻切出剩余部分，t 重基准为 0（同 node remainingTrajectory）。
TimedTrajectory sliceFrom(const TimedTrajectory& source, double elapsed)
{
  TimedTrajectory out;
  if (source.t.empty()) return out;
  const double begin = std::max(0.0, std::min(elapsed, source.duration));
  TrajectorySample first;
  if (!source.samplePVA(begin, first)) return out;
  out.t.push_back(0.0);
  out.pts.push_back(first.pos);
  out.vel.push_back(first.vel);
  out.acc.push_back(first.acc);
  out.yaw.push_back(first.yaw);
  for (size_t i = 0; i < source.t.size(); ++i)
  {
    if (source.t[i] <= begin + 1e-9) continue;
    out.t.push_back(source.t[i] - begin);
    out.pts.push_back(source.pts[i]);
    out.vel.push_back(source.vel[i]);
    out.acc.push_back(source.acc[i]);
    out.yaw.push_back(source.yaw[i]);
  }
  out.duration = source.duration - begin;
  return out;
}
}  // namespace


TEST(StageEClosedLoop, SwapTransitionsToEmergencyBrake)
{
  const ClosedLoopConfig config;
  const TimedTrajectory stand_on = linearTrajectory(
      Eigen::Vector3d(-1.5, 0.0, 0.0), Eigen::Vector3d(1.5, 0.0, 0.0), 4.0);
  const TimedTrajectory nominal_yield = linearTrajectory(
      Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d(-1.5, 0.0, 0.0), 4.0);
  const SafetyPrediction prediction = makePredictor(config).evaluate(
      nominal_yield, 0.0, "A02", {asPeerInput("A01", stand_on, config)}, 0.0);
  ASSERT_EQ(prediction.level, SafetyLevel::EMERGENCY);

  TrajectorySample first_start, second_start;
  ASSERT_TRUE(stand_on.samplePVA(0.0, first_start));
  ASSERT_TRUE(nominal_yield.samplePVA(0.0, second_start));
  TimedTrajectory first_brake, second_brake;
  EmergencyBrakeGenerator generator = makeBrakeGenerator();
  ASSERT_TRUE(generator.generate(first_start, first_brake));
  ASSERT_TRUE(generator.generate(second_start, second_brake));
  EXPECT_GE(simulatePair("swap_emergency_brake", first_brake, second_brake, config),
            kProtectedDistanceM);
}

TEST(StageEClosedLoop, EmergencyBrakeMaintainsActualProtectedDistance)
{
  const ClosedLoopConfig config;
  const TimedTrajectory first_nominal = linearTrajectory(
      Eigen::Vector3d(-1.5, 0.0, 0.0), Eigen::Vector3d(1.5, 0.0, 0.0), 4.0);
  const TimedTrajectory second_nominal = linearTrajectory(
      Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d(-1.5, 0.0, 0.0), 4.0);
  const SafetyPrediction prediction = makePredictor(config).evaluate(
      second_nominal, 0.0, "A02", {asPeerInput("A01", first_nominal, config)}, 0.0);
  ASSERT_EQ(prediction.level, SafetyLevel::EMERGENCY);

  TrajectorySample first_start, second_start;
  ASSERT_TRUE(first_nominal.samplePVA(0.0, first_start));
  ASSERT_TRUE(second_nominal.samplePVA(0.0, second_start));
  TimedTrajectory first_brake, second_brake;
  EmergencyBrakeGenerator generator = makeBrakeGenerator();
  ASSERT_TRUE(generator.generate(first_start, first_brake));
  ASSERT_TRUE(generator.generate(second_start, second_brake));
  EXPECT_GE(simulatePair("emergency_brake", first_brake, second_brake, config),
            kProtectedDistanceM);
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}