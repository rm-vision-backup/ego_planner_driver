#include "ego_planner_driver/safety_logger.h"

#include <cerrno>
#include <cctype>
#include <sys/stat.h>
#include <sys/types.h>

namespace ego_planner_driver
{

SafetyLogger::SafetyLogger() = default;
SafetyLogger::~SafetyLogger() { stop(); }

std::string SafetyLogger::sanitizeIdentity(const std::string& identity)
{
  if (identity.empty()) return "UNKNOWN";
  for (char c : identity)
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-')
      return "UNKNOWN";
  return identity;
}

bool SafetyLogger::ensureDirectory(const std::string& directory)
{
  if (directory.empty()) return false;
  std::string current;
  if (directory.front() == '/') current = "/";
  size_t begin = directory.front() == '/' ? 1 : 0;
  while (begin <= directory.size())
  {
    const size_t slash = directory.find('/', begin);
    const std::string part = directory.substr(begin, slash - begin);
    if (!part.empty())
    {
      if (current.size() > 1 && current.back() != '/') current += '/';
      current += part;
      struct stat info;
      if (::stat(current.c_str(), &info) != 0)
      {
        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
      }
      else if (!S_ISDIR(info.st_mode))
      {
        return false;
      }
    }
    if (slash == std::string::npos) break;
    begin = slash + 1;
  }
  return true;
}

bool SafetyLogger::start(const std::string& directory,
                         const std::string& identity, size_t capacity)
{
  stop();
  if (!ensureDirectory(directory)) return false;
  const std::string filename = directory + "/" +
      sanitizeIdentity(identity) + "-ego-planner.log";
  stream_.open(filename, std::ios::out | std::ios::trunc);
  if (!stream_.is_open()) return false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = std::max<size_t>(1, capacity);
    dropped_ = 0;
    stopping_ = false;
    enabled_ = true;
  }
  thread_ = std::thread(&SafetyLogger::writerLoop, this);
  return true;
}

void SafetyLogger::enqueue(const std::string& line)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || stopping_) return;
  if (queue_.size() >= capacity_)
  {
    queue_.pop_front();
    ++dropped_;
  }
  queue_.push_back(line);
  cv_.notify_one();
}

void SafetyLogger::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ && !thread_.joinable()) return;
    stopping_ = true;
    cv_.notify_all();
  }
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_.is_open()) stream_.close();
  queue_.clear();
  enabled_ = false;
  stopping_ = false;
}

bool SafetyLogger::enabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

size_t SafetyLogger::droppedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_;
}

void SafetyLogger::writerLoop()
{
  for (;;)
  {
    std::deque<std::string> batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return stopping_ || !queue_.empty(); });
      batch.swap(queue_);
      if (stopping_ && batch.empty()) break;
    }
    for (const std::string& line : batch) stream_ << line << '\n';
    stream_.flush();
  }
}

}  // namespace ego_planner_driver