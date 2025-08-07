//////////////////////////////////////////////////////////////////////////////
//
// Base class for all named objects which depend upon the initial time and need
// to convert between absolute times and time steps. Examples of derived classes
// are Cost.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_RELATIVE_TIME_TRACKER_H
#define ILQGAMES_UTILS_RELATIVE_TIME_TRACKER_H

#include <glog/logging.h>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class RelativeTimeTracker {
 public:
  virtual ~RelativeTimeTracker() {}

  // Access and reset initial time.
  static void ResetInitialTime(Time t0) { initial_time_ = t0; };
  static Time InitialTime() { return initial_time_; }

  // Convert between time step and initial time.
  static Time RelativeTime(size_t kk) {
    return static_cast<Time>(kk) * time::getTimeStep();
  }
  static Time AbsoluteTime(size_t kk) {
    return initial_time_ + static_cast<Time>(kk) * time::getTimeStep();
  }
  static size_t TimeIndex(Time t) {
    CHECK_GE(t, initial_time_);
    return static_cast<size_t>((t - initial_time_) / time::getTimeStep());
  }

  // Access the name of this object.
  const std::string& Name() const { return name_; }

 protected:
  RelativeTimeTracker(const std::string& name) : name_(name) {}

  // Name associated to every cost.
  const std::string name_;

  // Initial time.
  static Time initial_time_;
};  //\class Cost

}  // namespace e2e_noa::planning

#endif
