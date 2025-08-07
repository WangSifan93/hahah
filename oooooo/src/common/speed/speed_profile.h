#ifndef AD_E2E_PLANNING_COMMON_SPEED_SPEED_PROFILE_H
#define AD_E2E_PLANNING_COMMON_SPEED_SPEED_PROFILE_H
#include "common/type_def.h"

namespace ad_e2e {
namespace planning {
class SpeedProfile {
 public:
  SpeedProfile() = default;
  explicit SpeedProfile(std::vector<SpeedPoint> &&points,
                        bool re_evaluate = true);
  void SetPoints(const std::vector<SpeedPoint> &points,
                 bool re_evaluate = true);
  ~SpeedProfile() = default;

  bool IsValid() const { return speed_points_.size() >= 2; }
  void Reset() { speed_points_.clear(); }

  double GetTimeLength() const;

  SpeedPoint GetSpeedPointAtTime(const double &t) const;

  std::vector<SpeedPoint> SampleSpeedPointsByS(const double &start,
                                               const double &length,
                                               const double &interval) const;

  bool ComputeStopPoint(SpeedPoint &stop_point) const;

 private:
  std::vector<SpeedPoint> speed_points_;

  bool ComputeSpeedProfile();

  SpeedPoint GetSpeedPointAtTime(const double &t, const size_t &start_idx,
                                 const bool sequential_search,
                                 size_t &found_idx) const;

  SpeedPoint GetSpeedPointAtS(const double &s, const size_t &start_idx,
                              const bool sequential_search,
                              size_t &found_idx) const;

  SpeedPoint CalConstAccSpeedPoint(const SpeedPoint &start, const double a,
                                   const double t) const;

  SpeedPoint CalConstJerkSpeedPoint(const SpeedPoint &start, const double jerk,
                                    const double t) const;
};
using SpeedProfilePtr = std::shared_ptr<SpeedProfile>;
}  // namespace planning
}  // namespace ad_e2e
#endif
