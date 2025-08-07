/////////////////////////////////////////////////////////////////////////////
//
// Keeps track of elapsed time (e.g., during loops) and provides an upper bound
// on the runtime of the next loop. To reduce memory consumption and adapt to
// changing processor activity, computes statistics based on a moving window of
// specified length.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_LOOP_TIMER_H
#define ILQGAMES_UTILS_LOOP_TIMER_H

#include <glog/logging.h>

#include <chrono>
#include <list>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class LoopTimer {
 public:
  ~LoopTimer() {}
  LoopTimer(size_t max_samples = 10)
      : max_samples_(max_samples), total_time_(0.0) {
    CHECK_GT(max_samples, 1);

    // For defined behavior, starting with a Tic().
    Tic();
  }

  // Tic and toc. Start and stop loop timer.
  void Tic();
  Time Toc();

  // High probability upper bound on next loop runtime, with initial guess to be
  // returned if not enough data has been observed yet.
  Time RuntimeUpperBound(float num_stddevs = 3.0,
                         Time initial_guess = 0.02) const;

 private:
  // Maximum number of samples used to compute mean and variance.
  const size_t max_samples_;

  // Most recent timer start time.
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;

  // Queue of observed loop times.
  std::list<Time> loop_times_;

  // Running sum of times in the queue.
  Time total_time_;
};  // class LoopTimer

}  // namespace e2e_noa::planning

#endif
