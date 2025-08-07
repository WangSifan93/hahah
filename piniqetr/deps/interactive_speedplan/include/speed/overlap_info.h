#ifndef PLANNER_SPEED_OVERLAP_INFO_H_
#define PLANNER_SPEED_OVERLAP_INFO_H_

namespace e2e_noa::planning {

struct OverlapInfo {
  double time = 0.0;

  int obj_idx = 0;

  int av_start_idx = 0;

  int av_end_idx = 0;
};

}  // namespace e2e_noa::planning

#endif
