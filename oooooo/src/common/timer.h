#ifndef PLANNING_COMMON_TIMER
#define PLANNING_COMMON_TIMER
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/time/time.h"
namespace e2e_noa {
class Timer {
 public:
  Timer(const std::string& name, bool log_auto = true);
  Timer() : Timer("", false){};
  ~Timer();

  double TimeS() const { return static_cast<double>(TimeMs()) / 1e3; }

  int64_t TimeMs() const { return TimeSinceStart<std::chrono::milliseconds>(); }
  int64_t TimeUs() const { return TimeSinceStart<std::chrono::microseconds>(); }
  int64_t TimeNs() const { return TimeSinceStart<std::chrono::nanoseconds>(); }

  std::string Time() const { return WrapDuration(TimeNs()); }

  const std::chrono::system_clock::time_point& start_time() const;

  void Reset(const std::string& name);
  void Reset();

 private:
  void Log();

  template <typename T>
  int64_t TimeSinceStart() const {
    return std::chrono::duration_cast<T>(std::chrono::system_clock::now() -
                                         start_time_)
        .count();
  }

  static std::string WrapDuration(int64_t ns);

  std::chrono::system_clock::time_point start_time_;
  std::string name_;
  bool log_auto_{true};
};
}  // namespace e2e_noa

#endif
