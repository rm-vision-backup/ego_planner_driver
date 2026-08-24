#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include "ego_planner_driver/safety_logger.h"
#include "ego_planner_driver/safety_predictor.h"

using namespace ego_planner_driver;

namespace
{
TimedTrajectory linear(const Eigen::Vector3d& start, const Eigen::Vector3d& velocity,
                       double duration = 5.0, double dt = 0.05)
{
  TimedTrajectory trajectory;
  trajectory.duration = duration;
  for (double t = 0.0; t <= duration + 1e-9; t += dt)
  {
    trajectory.t.push_back(std::min(t, duration));
    trajectory.pts.push_back(start + velocity * std::min(t, duration));
    trajectory.vel.push_back(velocity);
    trajectory.acc.push_back(Eigen::Vector3d::Zero());
    trajectory.yaw.push_back(0.0);
  }
  return trajectory;
}

PairPrediction onlyPair(const SafetyPrediction& prediction)
{
  EXPECT_EQ(prediction.pairs.size(), 1u);
  return prediction.pairs.front();
}
}  // namespace

TEST(SafetyIdentity, ParsesNumericSuffixAndDeterminesRole)
{
  int id = -1;
  EXPECT_TRUE(SafetyPredictor::parseNumericId("A01", id)); EXPECT_EQ(id, 1);
  EXPECT_TRUE(SafetyPredictor::parseNumericId("A2", id)); EXPECT_EQ(id, 2);
  EXPECT_TRUE(SafetyPredictor::parseNumericId("UAV10", id)); EXPECT_EQ(id, 10);
  EXPECT_FALSE(SafetyPredictor::parseNumericId("UAV", id));
  EXPECT_FALSE(SafetyPredictor::parseNumericId("bad/name2", id));
  EXPECT_EQ(SafetyPredictor::determineRole("A01", "UAV2"), PairRole::STAND_ON);
  EXPECT_EQ(SafetyPredictor::determineRole("UAV10", "A2"), PairRole::YIELD);
  EXPECT_EQ(SafetyPredictor::determineRole("A2", "UAV2"), PairRole::UNKNOWN);
}

TEST(SafetyPredictor, ComputesHeadOnCpaTtcAndDynamicThresholds)
{
  SafetyPredictorConfig config;
  config.control_delay_s = 0.2;
  config.local_compute_budget_s = 0.1;
  config.peer_data_age_cap_s = 2.0;
  config.emergency_decel_mps2 = 2.0;
  SafetyPredictor predictor(config);
  PeerTrajectoryInput peer;
  peer.uav_id = "A02";
  peer.traj_id = 7;
  peer.activation_epoch_s = 100.0;
  peer.receive_epoch_s = 99.6;
  peer.trajectory = linear(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(-1, 0, 0));
  const PairPrediction pair = onlyPair(predictor.evaluate(
      linear(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 0, 0)),
      100.0, "A01", {peer}, 100.0));
  EXPECT_NEAR(pair.closing_speed_mps, 2.0, 1e-6);
  EXPECT_NEAR(pair.t_cpa_s, 2.5, 0.05);
  EXPECT_NEAR(pair.cpa_distance_m, 0.0, 1e-6);
  EXPECT_NEAR(pair.ttc_s, 2.0, 0.06);
  EXPECT_GT(pair.warning_distance_m, pair.emergency_distance_m);
  EXPECT_EQ(pair.role, PairRole::STAND_ON);
  EXPECT_EQ(pair.level, SafetyLevel::EMERGENCY);
}

TEST(SafetyPredictor, UsesCommonAbsoluteEpochAndDetectsOrthogonalCrossing)
{
  SafetyPredictor predictor{SafetyPredictorConfig()};
  PeerTrajectoryInput peer;
  peer.uav_id = "UAV1";
  peer.activation_epoch_s = 101.0;
  peer.receive_epoch_s = 101.0;
  peer.trajectory = linear(Eigen::Vector3d(2, -1, 0), Eigen::Vector3d(0, 1, 0));
  const PairPrediction pair = onlyPair(predictor.evaluate(
      linear(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 0, 0), 6.0),
      100.0, "UAV2", {peer}, 101.0));
  EXPECT_NEAR(pair.min_distance_m, 0.0, 0.08);
  EXPECT_EQ(pair.role, PairRole::YIELD);
  EXPECT_TRUE(pair.level == SafetyLevel::WARNING ||
              pair.level == SafetyLevel::EMERGENCY);
}

TEST(SafetyPredictor, ReportsSafeDivergingAndInsufficientWindow)
{
  SafetyPredictorConfig config;
  config.warning_ttc_s = 0.5;
  SafetyPredictor predictor(config);
  PeerTrajectoryInput peer;
  peer.uav_id = "UAV2";
  peer.activation_epoch_s = 0.0;
  peer.receive_epoch_s = 0.0;
  peer.trajectory = linear(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(1, 0, 0));
  PairPrediction pair = onlyPair(predictor.evaluate(
      linear(Eigen::Vector3d::Zero(), Eigen::Vector3d(-1, 0, 0)),
      0.0, "UAV1", {peer}, 0.0));
  EXPECT_EQ(pair.level, SafetyLevel::SAFE);
  EXPECT_DOUBLE_EQ(pair.closing_speed_mps, 0.0);

  peer.activation_epoch_s = 10.0;
  pair = onlyPair(predictor.evaluate(
      linear(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      0.0, "UAV1", {peer}, 0.0));
  EXPECT_EQ(pair.level, SafetyLevel::INSUFFICIENT_DATA);
}

TEST(SafetyPredictor, ThresholdsGrowWithAgeAndClosingSpeed)
{
  SafetyPredictor predictor{SafetyPredictorConfig()};
  PeerTrajectoryInput peer;
  peer.uav_id = "UAV2";
  peer.activation_epoch_s = 10.0;
  peer.receive_epoch_s = 10.0;
  peer.trajectory = linear(Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(-0.5, 0, 0));
  const TimedTrajectory self = linear(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0, 0));
  const double fresh = onlyPair(predictor.evaluate(self, 10.0, "UAV1", {peer}, 10.0)).warning_distance_m;
  peer.receive_epoch_s = 9.0;
  const double aged = onlyPair(predictor.evaluate(self, 10.0, "UAV1", {peer}, 10.0)).warning_distance_m;
  EXPECT_GT(aged, fresh);
}

TEST(ConflictEpisode, LatchesPeerAndReleasesAfterHysteresis)
{
  ConflictEpisode episode;
  SafetyPrediction warning;
  warning.level = SafetyLevel::WARNING;
  warning.primary_peer_id = "UAV2";
  PairPrediction pair; pair.peer_id = "UAV2"; pair.role = PairRole::STAND_ON;
  warning.pairs.push_back(pair);
  episode.update(warning, 1.0, 0.5);
  ASSERT_TRUE(episode.active());
  EXPECT_EQ(episode.peerId(), "UAV2");
  SafetyPrediction safe;
  episode.update(safe, 1.2, 0.5);
  EXPECT_TRUE(episode.active());
  episode.update(safe, 1.8, 0.5);
  EXPECT_FALSE(episode.active());
}

TEST(SafetyLogger, SanitizesOverwritesAndDrains)
{
  const std::string dir = "/home/ub20tg/catkin_swarm6-2/.tmp/safety_logger_test_" +
      std::to_string(static_cast<long long>(::getpid()));
  EXPECT_EQ(SafetyLogger::sanitizeIdentity("UAV1"), "UAV1");
  EXPECT_EQ(SafetyLogger::sanitizeIdentity("../UAV1"), "UNKNOWN");
  {
    SafetyLogger logger;
    ASSERT_TRUE(logger.start(dir, "UAV1", 16));
    logger.enqueue("old-line");
  }
  {
    SafetyLogger logger;
    ASSERT_TRUE(logger.start(dir, "UAV1", 16));
    logger.enqueue("new-line");
  }
  std::ifstream stream(dir + "/UAV1-ego-planner.log");
  std::string all((std::istreambuf_iterator<char>(stream)),
                  std::istreambuf_iterator<char>());
  EXPECT_EQ(all, "new-line\n");
}

TEST(SafetyLogger, BoundedQueueDropsOldestWithoutBlockingProducer)
{
  const std::string dir = "/home/ub20tg/catkin_swarm6-2/.tmp/safety_logger_queue_" +
      std::to_string(static_cast<long long>(::getpid()));
  SafetyLogger logger;
  ASSERT_TRUE(logger.start(dir, "UAV2", 1));
  for (int i = 0; i < 10000; ++i) logger.enqueue(std::to_string(i));
  logger.stop();
  EXPECT_GT(logger.droppedCount(), 0u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}