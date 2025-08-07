#include "common/timer.h"

#include <iomanip>
#include <map>
#include <sstream>

#include "common/log.h"
namespace e2e_noa {

static constexpr int64_t kNsInUs = 1000;
static constexpr int64_t kNsInMs = 1000 * kNsInUs;
static constexpr int64_t kNsInSec = 1000 * kNsInMs;
static constexpr int64_t kNsInMin = 60 * kNsInSec;
static constexpr int64_t kNsInHour = 60 * kNsInMin;

Timer::Timer(const std::string& name, bool log_auto)
    : start_time_(std::chrono::system_clock::now()),
      name_(name),
      log_auto_(log_auto) {}

Timer::~Timer() {
  if (log_auto_) {
    Log();
  }
}

void Timer::Reset(const std::string& name) {
  Reset();
  name_ = name;
}

void Timer::Reset() {
  if (log_auto_) {
    Log();
  }
  start_time_ = std::chrono::system_clock::now();
}

void Timer::Log() {
  if (!name_.empty()) {
    std::string c_out =
        name_ + ",time:" +
        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now() - start_time_)
                           .count()) +
        "ms.\n";

    VLOG(0) << c_out;
  }
}

std::string Timer::WrapDuration(int64_t ns) {
  static const std::map<int64_t, std::string> units{
      {kNsInHour, "h"}, {kNsInMin, "min"}, {kNsInSec, "s"},
      {kNsInMs, "ms"},  {kNsInUs, "us"},   {1, "ns"}};
  std::ostringstream stream;

  for (auto it = units.rbegin(); it != units.rend(); it++) {
    if (0 != ns / it->first) {
      stream << std::fixed << std::setprecision(2)
             << ns / static_cast<double>(it->first) << it->second;
      return stream.str();
    }
  }
  return "0";
}

const std::chrono::system_clock::time_point& Timer::start_time() const {
  return start_time_;
}

}  // namespace e2e_noa
