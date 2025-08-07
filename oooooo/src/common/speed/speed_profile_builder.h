#ifndef PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_BUILDER_H_
#define PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_BUILDER_H_
#include "common/speed/speed_profile.h"

namespace ad_e2e {
namespace planning {
class SpeedProfileBuilder {
 public:
  SpeedProfileBuilder() = default;
  ~SpeedProfileBuilder() = default;

  virtual SpeedProfilePtr Build(const SpeedPoint &start, const SpeedPoint &end,
                                const double &time_length) = 0;

 private:
};
}  
}  

#endif
