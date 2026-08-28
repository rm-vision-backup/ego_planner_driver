// 短末段内存回归（implementation_plan_26082722 §13.1）。
// 父进程 fork 功能 worker 与压力 worker：
//   - functional worker：RLIMIT_AS=512 MiB、RLIMIT_CPU=10 s，父进程 wall timeout 15 s；
//   - stress worker：RLIMIT_AS=512 MiB、RLIMIT_CPU=30 s，父进程 wall timeout 45 s；
//   - 父进程超时后终止 worker（SIGKILL）。
// worker 在受限子进程内遍历分类矩阵/压力用例，逐个调用 core_->planOnce：
//   - 通过 pipe 返回结构化分类计数与内存检查点（/proc/self/statm 当前 RSS、
//     getrusage ru_maxrss）；
//   - 父进程 wait4 获取退出状态、signal 与最终 child ru_maxrss。
#include <gtest/gtest.h>

#include <signal.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#include "ego_planner_driver/ego_planner_core.h"
#include "ego_planner_driver/polynomial_traj.h"

using ego_planner_driver::EgoPlannerConfig;
using ego_planner_driver::EgoPlannerCore;
using ego_planner_driver::GoalPoint;
using ego_planner_driver::PlanResult;
using ego_planner_driver::TimedTrajectory;
using ego_planner_driver::WayPoint;

namespace
{
constexpr long kAsLimitBytes = 512L * 1024L * 1024L;   // 512 MiB
constexpr long kRssGateKb = 384L * 1024L;               // 最终 child ru_maxrss < 384 MiB
constexpr long kRssPlateauKb = 32L * 1024L;             // 后半段当前 RSS max-min <= 32 MiB
constexpr long kRssFinalHeadroomKb = 16L * 1024L;       // 最后检查点 <= 中点+16 MiB

constexpr std::size_t kMaxCheckpoints = 64;

struct WorkerReport
{
  char magic[8];
  uint64_t degenerate = 0;
  uint64_t success = 0;
  uint64_t retryable = 0;
  uint64_t resource = 0;
  uint64_t total = 0;
  long ru_maxrss_kb = 0;
  std::size_t checkpoints = 0;
  long rss_kb[kMaxCheckpoints];
  bool violations = false;
  char detail[256];
};

// 读取 /proc/self/statm 当前 RSS（KB 近似：页数 * 页大小/1024）。
long currentRssKb()
{
  FILE* f = std::fopen("/proc/self/statm", "r");
  if (!f)
    return -1;
  long size = 0, resident = 0;
  if (std::fscanf(f, "%ld %ld", &size, &resident) != 2)
  {
    std::fclose(f);
    return -1;
  }
  std::fclose(f);
  return resident * (sysconf(_SC_PAGESIZE) / 1024);
}

EgoPlannerConfig makeConfig()
{
  EgoPlannerConfig config;
  config.ts = 0.4;
  config.max_vel = 1.0;
  config.max_acc = 6.0;
  config.swarm_clearance = 0.5;
  config.planning_range = 30.0;
  config.lambda_smooth = 1.0;
  config.lambda_swarm = 1.0;
  config.lambda_feasibility = 0.5;
  config.lambda_terminal = 1.0;
  config.enable_rebound = false;
  config.enable_time_reparam = false;
  config.cruise_velocity = 1.0;
  config.acceleration_limit = 2.0;
  config.max_iterations = 200;
  config.mem_size = 16;
  config.g_epsilon = 0.01;
  // 生产 launch 值（§11.1 默认）：
  config.max_arc_samples = 4096;
  config.max_parameterization_points = 256;
  config.max_trajectory_samples = 4096;
  config.max_neighbor_intent_samples = 256;
  config.segment_direction_epsilon_m = 0.05;
  return config;
}

// 分类矩阵单次用例：record 分类计数并做部分断言。
void runCase(EgoPlannerCore& core, WorkerReport& rep, double distance_m,
             double speed_mps, const Eigen::Vector3d& dir, bool final_segment,
             std::vector<long>& rss_samples)
{
  const Eigen::Vector3d start(0.0, 0.0, 15.0);
  // distance==0：start==end（必退化）；否则方向退化阈值两侧。
  const Eigen::Vector3d end = start + dir * distance_m;
  const std::vector<WayPoint> waypoints{WayPoint{end, 0.0}};
  const Eigen::Vector3d start_vel = dir * speed_mps;
  TimedTrajectory traj;
  double cost = 0.0;
  const PlanResult result =
      core.planOnce(GoalPoint{start, 0.0}, waypoints, {}, traj, cost,
                    start_vel, Eigen::Vector3d::Zero(), final_segment);

  ++rep.total;
  switch (result)
  {
    case PlanResult::DEGENERATE_SEGMENT: ++rep.degenerate; break;
    case PlanResult::SUCCESS: ++rep.success; break;
    case PlanResult::RETRYABLE_FAILURE: ++rep.retryable; break;
    case PlanResult::RESOURCE_LIMIT: ++rep.resource; break;
  }

  // 阈值两侧断言（§13.1）：d<0.05 必须 DEGENERATE；d==0.05 且 >0.05 不得
  // DEGENERATE（只允许 SUCCESS 或 RETRYABLE_FAILURE）。
  const double eps = core.config().segment_direction_epsilon_m;
  if (distance_m < eps && result != PlanResult::DEGENERATE_SEGMENT)
  {
    std::snprintf(rep.detail, sizeof(rep.detail),
                  "d=%.6f<eps must be DEGENERATE (got %d)", distance_m,
                  static_cast<int>(result));
    rep.violations = true;
    return;
  }
  if (distance_m >= eps && result == PlanResult::DEGENERATE_SEGMENT)
  {
    std::snprintf(rep.detail, sizeof(rep.detail),
                  "d=%.6f>=eps must not be DEGENERATE", distance_m);
    rep.violations = true;
    return;
  }

  if (result == PlanResult::SUCCESS)
  {
    // 成功轨迹：所有数组同长且 finite；首末位置准确；样本数不超上限。
    if (traj.pts.size() != traj.t.size() || traj.vel.size() != traj.t.size() ||
        traj.acc.size() != traj.t.size() || traj.yaw.size() != traj.t.size() ||
        traj.t.empty())
    {
      std::snprintf(rep.detail, sizeof(rep.detail), "array length mismatch");
      rep.violations = true;
      return;
    }
    if (traj.t.size() > core.config().max_trajectory_samples)
    {
      std::snprintf(rep.detail, sizeof(rep.detail), "trajectory samples overflow");
      rep.violations = true;
      return;
    }
    if ((traj.pts.front() - start).norm() > 1e-4 ||
        (traj.pts.back() - end).norm() > 1e-4)
    {
      std::snprintf(rep.detail, sizeof(rep.detail), "endpoint anchor mismatch");
      rep.violations = true;
      return;
    }
    for (std::size_t i = 0; i < traj.t.size(); ++i)
    {
      if (!std::isfinite(traj.t[i]) || !traj.pts[i].allFinite() ||
          !traj.vel[i].allFinite() || !traj.acc[i].allFinite() ||
          !std::isfinite(traj.yaw[i]) ||
          (i > 0 && traj.t[i] <= traj.t[i - 1]))
      {
        std::snprintf(rep.detail, sizeof(rep.detail), "non-finite sample at %zu", i);
        rep.violations = true;
        return;
      }
    }
  }
  rss_samples.push_back(currentRssKb());
}

void runWorker(bool stress, int out_fd)
{
  WorkerReport rep;
  std::memset(&rep, 0, sizeof(rep));
  std::memcpy(rep.magic, "SFPX1", 6);
  std::vector<long> rss_samples;
  rss_samples.reserve(1024);

  EgoPlannerCore core(makeConfig());
  const std::vector<double> distances{0.0, 1e-6, 0.001, 0.01, 0.049, 0.05,
                                      0.051, 0.10, 0.19, 0.21, 0.49, 1.0};
  const std::vector<double> speeds{0.0, 0.2, 0.5, 1.0};
  const std::vector<Eigen::Vector3d> dirs{
      Eigen::Vector3d(1.0, 0.0, 0.0),  // 正向
      Eigen::Vector3d(-1.0, 0.0, 0.0), // 反向
      Eigen::Vector3d(0.0, 1.0, 0.0)}; // 正交

  if (!stress)
  {
    // 功能 worker：完整分类矩阵一次遍历。
    for (double dist : distances)
      for (double speed : speeds)
        for (const auto& dir : dirs)
          for (bool final_segment : {false, true})
          {
            runCase(core, rep, dist, speed, dir, final_segment, rss_samples);
            if (rep.violations)
              break;
          }
    // §4.2（260828 方式 B）：资源注入已从普通功能矩阵拆出，独立
    // ResourceTightCapRejectsOversize 测试（见下）。普通矩阵不再执行资源注入，
    // 保持 rep.resource==0 断言覆盖普通（default-limit）用例。
  }
  else
  {
    // 压力 worker：代表性退化/0.05 阈值边界/普通短段/正常段，各累计至少 500 次。
    // SHORT_FINAL_STRESS_LOOPS 可覆盖循环次数（§14.11 2000 次长压力测试）；
    // 默认 500（§13.1），单测稳定不依赖环境。
    const char* env_loops = std::getenv("SHORT_FINAL_STRESS_LOOPS");
    const int loops = env_loops ? std::max(1, std::atoi(env_loops)) : 500;
    for (int k = 0; k < loops; ++k)
    {
      // 退化段（<0.05）
      runCase(core, rep, 0.049, 0.5, dirs[0], k % 2 == 0, rss_samples);
      // 0.05 阈值边界
      runCase(core, rep, 0.05, 0.5, dirs[0], k % 2 == 0, rss_samples);
      // 普通短段 0.1 / 0.2
      runCase(core, rep, k % 2 == 0 ? 0.1 : 0.2, 0.5, dirs[1], k % 2 == 0, rss_samples);
      // 正常段 3.0 m
      runCase(core, rep, 3.0, 1.0, dirs[0], false, rss_samples);
      // 资源/边界：远距离 30 m（在 256 点上限内，正常成功）
      runCase(core, rep, 30.0, 1.0, dirs[0], true, rss_samples);
      if (rep.violations)
        break;
    }
  }

  struct rusage ru;
  if (getrusage(RUSAGE_SELF, &ru) == 0)
    rep.ru_maxrss_kb = ru.ru_maxrss;

  // 内存检查点：预热结束、中点与后半段每 25 次记录当前 RSS，最多 kMaxCheckpoints 个。
  const std::size_t step = std::max<std::size_t>(1u, rss_samples.size() / kMaxCheckpoints);
  for (std::size_t i = 0; i < rss_samples.size() && rep.checkpoints < kMaxCheckpoints; i += step)
    rep.rss_kb[rep.checkpoints++] = rss_samples[i];
  // 后半段当前 RSS 平台判定：max-min <= 32 MiB。
  if (rep.checkpoints >= 4)
  {
    long rss_min = std::numeric_limits<long>::max();
    long rss_max = std::numeric_limits<long>::min();
    const std::size_t half = rep.checkpoints / 2;
    for (std::size_t i = half; i < rep.checkpoints; ++i)
    {
      rss_min = std::min(rss_min, rep.rss_kb[i]);
      rss_max = std::max(rss_max, rep.rss_kb[i]);
    }
    // 最后一个检查点不得高于中点超过 16 MiB。
    const long mid = rep.rss_kb[rep.checkpoints / 2];
    const long last = rep.rss_kb[rep.checkpoints - 1];
    if (rss_max - rss_min > kRssPlateauKb || last > mid + kRssFinalHeadroomKb)
    {
      std::snprintf(rep.detail, sizeof(rep.detail),
                    "RSS plateau violation: max-min=%ldKB last-mid=%ldKB",
                    rss_max - rss_min, last - mid);
      rep.violations = true;
    }
  }

  // 把结构化报告写回父进程（write 自含 POD）。
  const ssize_t w = write(out_fd, &rep, sizeof(rep));
  (void)w;
  close(out_fd);
  _exit(0);
}

// fork 受限 worker 并等待。返回 (worker_exit_ok, report)。
bool runRestrictedWorker(bool stress, int wall_timeout_ms, WorkerReport& rep)
{
  int pfd[2];
  if (pipe(pfd) != 0)
  {
    ADD_FAILURE() << "pipe failed";
    return false;
  }
  pid_t pid = fork();
  if (pid < 0)
  {
    ADD_FAILURE() << "fork failed";
    close(pfd[0]);
    close(pfd[1]);
    return false;
  }
  if (pid == 0)
  {
    close(pfd[0]);
    // 受限子进程：AS 512 MiB；功能 worker CPU 10 s、压力 worker CPU 30 s。
    // §4.2（260828）：setrlimit 失败必须用明确 exit code 报告（不静默放行），
    // 否则限制未安装时仍可能报告测试成功。
    struct rlimit lim_as;
    lim_as.rlim_cur = kAsLimitBytes;
    lim_as.rlim_max = kAsLimitBytes;
    if (setrlimit(RLIMIT_AS, &lim_as) != 0)
      _exit(110);  // RLIMIT_AS 安装失败
    struct rlimit lim_cpu;
    lim_cpu.rlim_cur = stress ? 30 : 10;
    lim_cpu.rlim_max = stress ? 30 : 10;
    if (setrlimit(RLIMIT_CPU, &lim_cpu) != 0)
      _exit(111);  // RLIMIT_CPU 安装失败
    runWorker(stress, pfd[1]);
    _exit(1);  // not reached
  }

  close(pfd[1]);
  // 父进程 wall timeout：poll 等待子进程写报告并退出（POLLHUP 表示写端关闭）。
  struct pollfd fds;
  fds.fd = pfd[0];
  fds.events = POLLIN | POLLHUP;
  const int pr = poll(&fds, 1, wall_timeout_ms);
  bool timeout = (pr == 0);
  if (pr < 0)
    timeout = true;
  if (timeout)
  {
    kill(pid, SIGKILL);
    int st = 0;
    waitpid(pid, &st, 0);
    close(pfd[0]);
    ADD_FAILURE() << "worker wall timeout";
    return false;
  }
  // 读取报告（worker 已写并关闭；读 0 表示没有完整报告）。
  ssize_t n = read(pfd[0], &rep, sizeof(rep));
  close(pfd[0]);
  if (n != static_cast<ssize_t>(sizeof(rep)) ||
      std::memcmp(rep.magic, "SFPX1", 6) != 0)
  {
    ADD_FAILURE() << "worker report malformed (n=" << n << ")";
    return false;
  }
  // 等待并统计（wait4 提供最终 child ru_maxrss）。
  struct rusage ru;
  int status = 0;
  if (wait4(pid, &status, 0, &ru) < 0)
  {
    ADD_FAILURE() << "wait4 failed";
    return false;
  }
  const bool exited_ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
  const bool signaled = WIFSIGNALED(status);
  const long child_ru_maxrss_kb = ru.ru_maxrss;
  if (!exited_ok || signaled)
  {
    ADD_FAILURE() << "worker abnormal exit: signaled=" << signaled
                  << " status=" << status;
    return false;
  }
  // §13.1 内存门禁：最终 child ru_maxrss < 384 MiB。
  if (child_ru_maxrss_kb >= kRssGateKb)
  {
    ADD_FAILURE() << "child ru_maxrss " << child_ru_maxrss_kb
                  << "KB >= 384MiB gate";
    return false;
  }
  return !rep.violations;
}
}  // namespace

TEST(ShortFinalSegment, FunctionalMatrixBoundedAndClassified)
{
  WorkerReport rep;
  ASSERT_TRUE(runRestrictedWorker(false, 15000, rep));
  EXPECT_GT(rep.total, 0u);
  // 退化分类：距离 < 0.05 的笛卡尔积用例必须返回 DEGENERATE_SEGMENT。
  // （0.0~0.049 共 5 个距离 × 4 speed × 3 dir × 2 final = 120 次。）
  EXPECT_GT(rep.degenerate, 0u);
  EXPECT_GT(rep.success + rep.retryable, 0u);
  EXPECT_EQ(rep.resource, 0u);
  // 常规（default-limit）用例不允许 RESOURCE_LIMIT（§13.1 断言）。
  EXPECT_FALSE(rep.violations);
  if (rep.violations)
    FAIL() << rep.detail;
}

TEST(ShortFinalSegment, StressLoopMemoryPlateau)
{
  WorkerReport rep;
  ASSERT_TRUE(runRestrictedWorker(true, 45000, rep));
  EXPECT_GE(rep.total, 500u * 5u);
  EXPECT_FALSE(rep.violations);
  if (rep.violations)
    FAIL() << rep.detail;
  // 压力 worker 自身 ru_maxrss 记录也应低于 384 MiB（§13.1 门禁）。
  EXPECT_LT(rep.ru_maxrss_kb, kRssGateKb);
}

TEST(ShortFinalSegment, ThresholdSidesAreClassifiedCorrectly)
{
  // 直接（非受限子进程）验证阈值两侧：<0.05 必 DEGENERATE，==/>=0.05 非
  // DEGENERATE（§5.2/§13.1）。此测试嵌套于功能 worker 矩阵，此处做白盒复核。
  EgoPlannerCore core(makeConfig());
  const Eigen::Vector3d start(0.0, 0.0, 15.0);
  for (double dist : {0.0, 1e-6, 0.001, 0.01, 0.049})
  {
    const std::vector<WayPoint> wp{WayPoint{start + Eigen::Vector3d(dist, 0.0, 0.0), 0.0}};
    TimedTrajectory traj;
    double cost = 0.0;
    EXPECT_EQ(core.planOnce(GoalPoint{start, 0.0}, wp, {}, traj, cost),
              PlanResult::DEGENERATE_SEGMENT);
  }
  for (double dist : {0.05, 0.051, 0.10, 0.21, 1.0})
  {
    const std::vector<WayPoint> wp{WayPoint{start + Eigen::Vector3d(dist, 0.0, 0.0), 0.0}};
    TimedTrajectory traj;
    double cost = 0.0;
    const PlanResult result =
        core.planOnce(GoalPoint{start, 0.0}, wp, {}, traj, cost);
    EXPECT_NE(result, PlanResult::DEGENERATE_SEGMENT);
    EXPECT_NE(result, PlanResult::RESOURCE_LIMIT);
  }
}

TEST(ShortFinalSegment, ParameterizationCapRejectsOversize)
{
  // §13.2：parameterization cap 边界和 cap+1。cap=8 正常生产段仍成功
  // （3m 约 8-10 点会被 cap 拒绝），cap=256 覆盖 60m 正常路径见 test_trajectory。
  EgoPlannerConfig config = makeConfig();
  config.max_parameterization_points = 256;
  EgoPlannerCore core(config);
  const Eigen::Vector3d start(0.0, 0.0, 15.0);
  TimedTrajectory traj;
  double cost = 0.0;
  const std::vector<WayPoint> wp{WayPoint{Eigen::Vector3d(60.0, 0.0, 15.0), 0.0}};
  const PlanResult result =
      core.planOnce(GoalPoint{start, 0.0}, wp, {}, traj, cost);
  EXPECT_EQ(result, PlanResult::SUCCESS);  // 256 点上限覆盖 60m 长段（§13.2）
  EXPECT_LE(traj.pts.size(), config.max_trajectory_samples);
}

// §4.2（260828 方式 B）：资源注入独立测试，只接受 PlanResult::RESOURCE_LIMIT，
// 且失败输出为空（不发布部分 intent）。不放进普通矩阵的 rep.resource 计数。
TEST(ShortFinalSegment, ResourceTightCapRejectsOversize)
{
  // max_parameterization_points=8：正常 3m 段约 8-10 点，6m 段必超 → 资源失败。
  EgoPlannerConfig tight = makeConfig();
  tight.max_parameterization_points = 8;
  EgoPlannerCore tight_core(tight);
  const Eigen::Vector3d start(0.0, 0.0, 15.0);
  TimedTrajectory traj;
  double cost = 0.0;
  const std::vector<WayPoint> wp{WayPoint{Eigen::Vector3d(6.0, 0.0, 15.0), 0.0}};
  const PlanResult result =
      tight_core.planOnce(GoalPoint{start, 0.0}, wp, {}, traj, cost);
  EXPECT_EQ(result, PlanResult::RESOURCE_LIMIT);  // 只接受 RESOURCE_LIMIT
  EXPECT_TRUE(traj.pts.empty());                   // 失败输出为空（不发布部分 intent）
  EXPECT_TRUE(traj.t.empty());
}

TEST(ShortFinalSegment, DegenerateDoesNotTriggerBrake)
{
  // §5.2/§9.4 契约：DEGENERATE_SEGMENT 只是输入分类——不生成新轨迹、不设置
  // 零速 steady target、不触发紧急制动（这些属于 node 状态机，见
  // test_ego_state_machine.cpp 对应用例）。此处验证 core 层不因退化段抛异常。
  EgoPlannerCore core(makeConfig());
  const Eigen::Vector3d start(0.0, 0.0, 15.0);
  const std::vector<WayPoint> wp{WayPoint{start + Eigen::Vector3d(0.01, 0.0, 0.0), 0.0}};
  TimedTrajectory traj;
  double cost = 0.0;
  EXPECT_NO_THROW({
    const PlanResult r = core.planOnce(GoalPoint{start, 0.0}, wp, {}, traj, cost);
    EXPECT_EQ(r, PlanResult::DEGENERATE_SEGMENT);
  });
}

TEST(ShortFinalSegment, FiniteAndOverflowInputsRejected)
{
  // §13.2：NaN/Inf、非法 duration/spacing 不得构造出非有限轨迹。
  EgoPlannerCore core(makeConfig());
  const Eigen::Vector3d start(0.0, 0.0, 15.0);
  TimedTrajectory traj;
  double cost = 0.0;
  std::vector<WayPoint> nan_wp{WayPoint{
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 15.0), 0.0}};
  const PlanResult nan_result =
      core.planOnce(GoalPoint{start, 0.0}, nan_wp, {}, traj, cost);
  EXPECT_NE(nan_result, PlanResult::SUCCESS);
  std::vector<WayPoint> inf_wp{WayPoint{
      Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 15.0), 0.0}};
  const PlanResult inf_result =
      core.planOnce(GoalPoint{start, 0.0}, inf_wp, {}, traj, cost);
  EXPECT_NE(inf_result, PlanResult::SUCCESS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
