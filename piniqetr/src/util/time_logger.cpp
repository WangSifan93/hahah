#include "util/time_logger.h"

#include <sys/time.h>

#include "boost/ptr_container/ptr_vector.hpp"
#include "common/log.h"
#include "common/planning_macros.h"
#include "util/logger_stack.h"

namespace ad_e2e {
namespace planning {

TimeLogger::TimeLogger(const std::string& logger_name)
    : _logger_name(logger_name), _start_time(GetCurrentTimeStamp()), _total(0) {
  LoggerStack::GetStack()
      .StartSegment(_start_time)
      ->SetName(logger_name + ":total");
  LoggerStack::GetStack().StartSegment(_start_time);
}

void TimeLogger::ResetStartTime() {
  _start_time = GetCurrentTimeStamp();
  _total = 0.0;
}

void TimeLogger::ChangeSegment(const std::string& old_seg_name,
                               int64_t timestamp) {
  LoggerStack::GetStack().EndSegment(timestamp, old_seg_name);
  LoggerStack::GetStack().StartSegment(timestamp);
}

void TimeLogger::RegisterTime(const std::string& time_seg_name) {
  if (_disable) {
    return;
  }
  int64_t end = GetCurrentTimeStamp();
  _total += (end - _start_time);
  _start_time = end;
  ChangeSegment(time_seg_name, end);
}

void TimeLogger::RegisterTimeAndPrint(const std::string& time_seg_name) {
  if (_disable) {
    return;
  }
  RegisterTime(time_seg_name);
  _printed = true;
}

void TimeLogger::SetDisable(bool disable) { _disable = disable; }

int64_t TimeLogger::GetCurrentTimeStamp() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

TimeLogger::~TimeLogger() {
  int64_t end = GetCurrentTimeStamp();

  LoggerStack::GetStack().EndSegment(end, _logger_name + ":end");
  if (_printed) {
    LoggerStack::GetStack().EndSegment(end);
  } else {
    LoggerStack::GetStack().AbandonSegment();
  }
}

TimeLogger::TimeLogger(const std::string& logger_name,
                       const std::string& father_segment_name)
    : _logger_name(logger_name), _start_time(GetCurrentTimeStamp()), _total(0) {
  LoggerStack& stack = LoggerStack::GetStack();
  stack.StartSegment(_start_time)->SetName(logger_name + ":total");
  stack.StartSegment(_start_time);
}

}  // namespace planning
}  // namespace ad_e2e
