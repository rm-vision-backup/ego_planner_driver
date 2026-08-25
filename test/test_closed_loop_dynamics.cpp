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
#include "ego_planner_driver/yield_candidates.h"

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

YieldCandidateGenerator makeYieldGenerator()
{
  YieldCandidateConfig config;
  config.lateral_offset_m = 2.2;
  config.lateral_time_scale = 1.5;
  config.slow_time_scale = 1.5;
  config.fast_time_scale = 0.8;
  config.max_velocity_mps = 1.5;
  config.max_acceleration_mps2 = 6.0;
  config.max_jerk_mps3 = 12.0;
  config.clearance_m = kProtectedDistanceM;
  return YieldCandidateGenerator(config);
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

TimedTrajectory selectYield(const TimedTrajectory& nominal,
                            const TimedTrajectory& peer,
                            YieldManeuver* maneuver = nullptr)
{
  YieldCandidateGenerator generator = makeYieldGenerator();
  const std::vector<YieldCandidate> candidates =
      generator.generate(nominal, {asNeighbor(peer)});
  YieldCandidate selected;
  EXPECT_TRUE(generator.chooseBest(candidates, selected));
  EXPECT_TRUE(selected.feasible);
  if (maneuver) *maneuver = selected.maneuver;
  return selected.trajectory;
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

TEST(StageEClosedLoop, OrthogonalYieldMaintainsActualProtectedDistance)
{
  const ClosedLoopConfig config;
  // Perpendicular near miss with exactly 3 m initial center spacing. Its
  // nominal CPA is inside the warning margin but outside the emergency margin.
  const double peer_x = 1.49;
  const double peer_y = -std::sqrt(9.0 - std::pow(peer_x + 1.5, 2.0));
  const TimedTrajectory stand_on = linearTrajectory(
      Eigen::Vector3d(-1.5, 0.0, 0.0), Eigen::Vector3d(1.5, 0.0, 0.0), 6.0);
  const TimedTrajectory nominal_yield = linearTrajectory(
      Eigen::Vector3d(peer_x, peer_y, 0.0),
      Eigen::Vector3d(peer_x, peer_y + 3.0, 0.0), 6.0);
  ASSERT_NEAR((stand_on.pts.front() - nominal_yield.pts.front()).norm(), 3.0, 1e-9);
  const SafetyPrediction prediction = makePredictor(config).evaluate(
      nominal_yield, 0.0, "A02", {asPeerInput("A01", stand_on, config)}, 0.0);
  ASSERT_EQ(prediction.level, SafetyLevel::WARNING);
  ASSERT_EQ(prediction.pairs.front().role, PairRole::YIELD);

  YieldManeuver maneuver = YieldManeuver::FAST;
  const TimedTrajectory yield = selectYield(nominal_yield, stand_on, &maneuver);
  EXPECT_TRUE(maneuver == YieldManeuver::LEFT || maneuver == YieldManeuver::RIGHT);
  EXPECT_GE(simulatePair("orthogonal_yield", stand_on, yield, config),
            kProtectedDistanceM);
}

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
  ASSERT_EQ(prediction.pairs.front().role, PairRole::YIELD);

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

TEST(StageEClosedLoop, FailedYieldCandidatesTransitionToBrake)
{
  const ClosedLoopConfig config;
  const TimedTrajectory first_nominal = linearTrajectory(
      Eigen::Vector3d(-1.5, 0.0, 0.0), Eigen::Vector3d(1.5, 0.0, 0.0), 4.0);
  const TimedTrajectory second_nominal = linearTrajectory(
      Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d(-1.5, 0.0, 0.0), 4.0);

  NeighborTrajectory blocked;
  blocked.start_time = 0.0;
  blocked.duration = second_nominal.duration;
  blocked.t = {0.0, second_nominal.duration};
  blocked.pts = {second_nominal.pts.front(), second_nominal.pts.front()};
  YieldCandidateGenerator yield_generator = makeYieldGenerator();
  const std::vector<YieldCandidate> candidates =
      yield_generator.generate(second_nominal, {blocked});
  YieldCandidate selected;
  ASSERT_FALSE(yield_generator.chooseBest(candidates, selected));

  TrajectorySample first_start, second_start;
  ASSERT_TRUE(first_nominal.samplePVA(0.0, first_start));
  ASSERT_TRUE(second_nominal.samplePVA(0.0, second_start));
  TimedTrajectory first_brake, second_brake;
  EmergencyBrakeGenerator brake_generator = makeBrakeGenerator();
  ASSERT_TRUE(brake_generator.generate(first_start, first_brake));
  ASSERT_TRUE(brake_generator.generate(second_start, second_brake));
  EXPECT_GE(simulatePair("candidate_failure_brake", first_brake, second_brake, config),
            kProtectedDistanceM);
}

TEST(StageEClosedLoop, ConflictEpisodeLatchesYieldRoleAcrossSafeGap)
{
  ConflictEpisode episode;
  SafetyPrediction warning;
  warning.level = SafetyLevel::WARNING;
  warning.primary_peer_id = "A01";
  PairPrediction pair;
  pair.peer_id = "A01";
  pair.level = SafetyLevel::WARNING;
  pair.role = PairRole::YIELD;
  warning.pairs.push_back(pair);
  episode.update(warning, 0.0, 1.0);
  ASSERT_TRUE(episode.active());
  EXPECT_EQ(episode.peerId(), "A01");
  EXPECT_EQ(episode.role(), PairRole::YIELD);

  SafetyPrediction safe;
  episode.update(safe, 0.8, 1.0);
  EXPECT_TRUE(episode.active());
  EXPECT_EQ(episode.role(), PairRole::YIELD);
  episode.update(safe, 1.81, 1.0);
  EXPECT_FALSE(episode.active());
}

TEST(StageEClosedLoop, DualNodeActiveAvoidanceMaintainsDistance)
{
  const ClosedLoopConfig config;
  const double peer_delay_s = 0.8;   // 跨 master intent 延迟注入
  const double decision_dt = 1.0 / 25.0;  // supervisor 25Hz
  const double dt = 0.01;

  // 正交近失（3m 初始中心距，nominal CPA 在 warning 内）：
  // A01 = STAND_ON 沿 x；A02 = YIELD 沿 +y 进入 A01 路径。
  const double peer_x = 1.49;
  const double peer_y = -std::sqrt(9.0 - std::pow(peer_x + 1.5, 2.0));
  const TimedTrajectory nominal_a01 = linearTrajectory(
      Eigen::Vector3d(-1.5, 0.0, 0.0), Eigen::Vector3d(1.5, 0.0, 0.0), 6.0);
  const TimedTrajectory nominal_a02 = linearTrajectory(
      Eigen::Vector3d(peer_x, peer_y, 0.0),
      Eigen::Vector3d(peer_x, peer_y + 3.0, 0.0), 6.0);

  SafetyPredictor predictor = makePredictor(config);
  YieldCandidateGenerator yield_generator = makeYieldGenerator();
  EmergencyBrakeGenerator brake_generator = makeBrakeGenerator();

  TimedTrajectory current_a01 = nominal_a01;
  TimedTrajectory current_a02 = nominal_a02;
  double activation_a01 = 0.0;
  double activation_a02 = 0.0;
  Plant plant_a01, plant_a02;
  initializePlant(current_a01, plant_a01);
  initializePlant(current_a02, plant_a02);

  double min_dist = (plant_a01.pos - plant_a02.pos).norm();
  int a02_commits = 0;
  bool a01_emergency = false;

  const double end = 9.0;
  for (double now = 0.0; now <= end; now += dt)
  {
    if (std::fmod(now, decision_dt) < dt * 0.5)
    {
      // A02 (YIELD)：用延迟到达的 A01 intent 评估当前轨迹并决策。
      if (now >= peer_delay_s)
      {
        PeerTrajectoryInput peer_a01;
        peer_a01.uav_id = "A01";
        peer_a01.exec_target = "UAV1";
        peer_a01.traj_id = 1;
        peer_a01.trajectory = current_a01;
        peer_a01.activation_epoch_s = activation_a01;
        peer_a01.receive_epoch_s = now - peer_delay_s;
        const SafetyPrediction prediction = predictor.evaluate(
            current_a02, activation_a02, "A02", {peer_a01}, now);
        const SafetyLevel level = prediction.level;
        const PairRole role = prediction.pairs.empty()
            ? PairRole::UNKNOWN : prediction.pairs.front().role;
        if (level == SafetyLevel::EMERGENCY)
        {
          TrajectorySample start;
          if (current_a02.samplePVA(now - activation_a02, start))
          {
            TimedTrajectory braking;
            if (brake_generator.generate(start, braking))
            {
              current_a02 = braking;
              activation_a02 = now;
              ++a02_commits;
            }
          }
        }
        else if (level == SafetyLevel::WARNING && role == PairRole::YIELD &&
                 a02_commits == 0)
        {
          const TimedTrajectory remaining = sliceFrom(
              current_a02, now - activation_a02);
          NeighborTrajectory peer = asNeighbor(current_a01);
          peer.start_time = activation_a01 - now;  // rebase 到候选激活 now
          const auto candidates = yield_generator.generate(remaining, {peer});
          YieldCandidate selected;
          if (yield_generator.chooseBest(candidates, selected))
          {
            current_a02 = selected.trajectory;
            activation_a02 = now;
            ++a02_commits;
          }
        }
      }
      // A01 (STAND_ON)：应保持 nominal，不应进入 emergency（A02 让路应消除冲突）。
      if (now >= peer_delay_s && !a01_emergency)
      {
        PeerTrajectoryInput peer_a02;
        peer_a02.uav_id = "A02";
        peer_a02.exec_target = "UAV2";
        peer_a02.traj_id = 1;
        peer_a02.trajectory = current_a02;
        peer_a02.activation_epoch_s = activation_a02;
        peer_a02.receive_epoch_s = now - peer_delay_s;
        const SafetyPrediction prediction = predictor.evaluate(
            current_a01, activation_a01, "A01", {peer_a02}, now);
        a01_emergency = prediction.level == SafetyLevel::EMERGENCY;
      }
    }
    stepPlant(current_a01, now - activation_a01, config, plant_a01);
    stepPlant(current_a02, now - activation_a02, config, plant_a02);
    min_dist = std::min(min_dist, (plant_a01.pos - plant_a02.pos).norm());
  }
  EXPECT_GE(min_dist, kProtectedDistanceM);
  EXPECT_GT(a02_commits, 0) << "A02 (YIELD) should commit a yield maneuver";
  EXPECT_FALSE(a01_emergency)
      << "STAND_ON A01 must not enter EMERGENCY when A02 yields";
  std::cout << "[dual_node] min_center_distance_m=" << min_dist
            << " a02_commits=" << a02_commits
            << " intent_delay_s=" << peer_delay_s << std::endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}