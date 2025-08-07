#ifndef PILOT_PLANNING_COMMON_SPEED_CONSTANT_ACC_BUILDER_H_
#define PILOT_PLANNING_COMMON_SPEED_CONSTANT_ACC_BUILDER_H_
#include "common/speed/speed_profile_builder.h"

namespace ad_e2e {
namespace planning {
class ConstantAccBuilder : public SpeedProfileBuilder {
 public:
  ConstantAccBuilder() = default;
  ~ConstantAccBuilder() = default;

  SpeedProfilePtr Build(const SpeedPoint &start, const SpeedPoint &end,
                        const double &time_length) override;

  void set_interval(const double &interval) { interval_ = interval; };

 private:
  double interval_ = 0.1;

  double ComputeJerk(const SpeedPoint &pt, const double &target_a,
                     const double &t_interval) const;
};
}  
}  

#endif
