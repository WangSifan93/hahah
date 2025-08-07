//////////////////////////////////////////////////////////////////////////////
//
// Keeps track of elapsed time (e.g., during loops) and provides an upper bound
// on the runtime of the next loop. To reduce memory consumption and adapt to
// changing processor activity, computes statistics based on a moving window of
// specified length.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/utils/loop_timer.h"

#include <glog/logging.h>

#include <chrono>
#include <list>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

void LoopTimer::Tic() { start_ = std::chrono::high_resolution_clock::now(); }

Time LoopTimer::Toc() {
  // Elapsed time in seconds.
  const Time elapsed = (std::chrono::duration<Time>(
                            std::chrono::high_resolution_clock::now() - start_))
                           .count();

  // Add to queue and pop if queue is too long.
  loop_times_.push_back(elapsed);
  total_time_ += elapsed;

  if (loop_times_.size() > max_samples_) {
    total_time_ -= loop_times_.front();
    loop_times_.pop_front();
  }

  return elapsed;
}

Time LoopTimer::RuntimeUpperBound(float num_stddevs, Time initial_guess) const {
  // Handle not enough data.
  if (loop_times_.size() < 2) return initial_guess;

  // Compute mean and variance.
  const Time mean = total_time_ / static_cast<Time>(loop_times_.size());
  Time variance = 0.0;
  for (const Time entry : loop_times_) {
    const Time diff = entry - mean;
    variance += diff * diff;
  }

  // Unbiased estimator of variance should divide by N - 1, not N.
  variance /= static_cast<Time>(loop_times_.size() - 1);

  // Compute upper bound.
  return mean + num_stddevs * std::sqrt(variance);
}

}  // namespace e2e_noa::planning
