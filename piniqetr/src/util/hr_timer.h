#ifndef AD_E2E_PLANNING_UTILS_HRTIMER_H
#define AD_E2E_PLANNING_UTILS_HRTIMER_H

#include <chrono>

namespace ad_e2e {
namespace planning {

class HRTimer {
 public:
  HRTimer() { Restart(); }

  void Restart();
  long ElapsedNs() const;
  long ElapsedUs() const;
  long ElapsedMs() const;
  long ElapsedS() const;

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

}  // namespace planning
}  // namespace ad_e2e

#endif
