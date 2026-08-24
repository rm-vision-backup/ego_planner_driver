#ifndef EGO_PLANNER_DRIVER_SAFETY_LOGGER_H_
#define EGO_PLANNER_DRIVER_SAFETY_LOGGER_H_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace ego_planner_driver
{

class SafetyLogger
{
public:
  SafetyLogger();
  ~SafetyLogger();

  bool start(const std::string& directory, const std::string& identity,
             size_t capacity = 2048);
  void enqueue(const std::string& line);
  void stop();
  bool enabled() const;
  size_t droppedCount() const;

  static std::string sanitizeIdentity(const std::string& identity);
  static bool ensureDirectory(const std::string& directory);

private:
  void writerLoop();

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::string> queue_;
  std::ofstream stream_;
  std::thread thread_;
  size_t capacity_ = 0;
  size_t dropped_ = 0;
  bool stopping_ = false;
  bool enabled_ = false;
};

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_SAFETY_LOGGER_H_