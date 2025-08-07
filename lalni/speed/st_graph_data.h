#ifndef PLANNER_SPEED_ST_GRAPH_DATA_H_
#define PLANNER_SPEED_ST_GRAPH_DATA_H_

#include <vector>

#include "speed/speed_limit.h"
#include "speed/speed_limit_provider.h"
#include "speed/st_boundary.h"

namespace e2e_noa::planning {

class StGraphData {
 public:
  StGraphData(const SpeedLimitProvider* speed_limit_provider,
              double cruise_speed, double path_length, double total_time)
      : speed_limit_provider_(CHECK_NOTNULL(speed_limit_provider)),
        cruise_speed_(cruise_speed),
        path_length_(path_length),
        total_time_(total_time) {}

  const SpeedLimitProvider& speed_limit_provider() const {
    return *speed_limit_provider_;
  }

  double cruise_speed() const { return cruise_speed_; }

  double path_length() const { return path_length_; }

  double total_time() const { return total_time_; }

 private:
  const SpeedLimitProvider* speed_limit_provider_ = nullptr;
  double cruise_speed_ = 0.0;
  double path_length_ = 0.0;
  double total_time_ = 0.0;
};

}  // namespace e2e_noa::planning

#endif
