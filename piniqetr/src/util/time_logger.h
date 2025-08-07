#ifndef AD_E2E_PLANNING_UTILS_TIME_LOGGER_H
#define AD_E2E_PLANNING_UTILS_TIME_LOGGER_H

#include <chrono>
#include <string>

namespace ad_e2e {
namespace planning {

class TimeLogger {
 public:
  TimeLogger(const std::string& logger_name);
  TimeLogger(const std::string& logger_name,
             const std::string& father_segment_name);

  void ResetStartTime();

  void RegisterTime(const std::string& time_seg_name);

  void RegisterTimeAndPrint(const std::string& time_seg_name);

  static int64_t GetCurrentTimeStamp();

  void SetDisable(bool disable);

  ~TimeLogger();

 private:
  void ChangeSegment(const std::string& old_seg_name, int64_t timestamp);

  std::string _logger_name;
  int64_t _start_time;
  int64_t _total;
  bool _disable = false;
  bool _printed = false;
};

}  // namespace planning
}  // namespace ad_e2e

#endif
