#ifndef AD_E2E_PLANNING_EGO_HISTORY_H
#define AD_E2E_PLANNING_EGO_HISTORY_H

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "util/time_util.h"

namespace e2e_noa {
namespace planning {

#define MAX_EGO_FRAME_NUM (20)

struct V2SpeedLimitInfo {
  bool is_generate_small_speed_limit = false;
  bool is_close_curb = false;
  bool is_in_uturn = false;
};

struct EgoFrame {
  V2SpeedLimitInfo v2_speed_limit_info;
};

class EgoHistory {
 public:
  EgoHistory();

  ~EgoHistory();

  std::deque<EgoFrame>& GetFrames() { return frames_; }
  const EgoFrame* GetOldestFrame() const {
    if (Empty()) {
      return nullptr;
    }
    return &(frames_.front());
  }
  const EgoFrame* GetLatestFrame() const {
    if (Empty()) {
      return nullptr;
    }
    return &(frames_.back());
  }

  const EgoFrame* GetSecondLatestFrame() const {
    if (Size() < 2) {
      return nullptr;
    }
    return &(frames_[Size() - 2]);
  }

  const int Size() const { return frames_.size(); }
  bool Empty() const { return frames_.empty(); }

  void CleanExceeded();

  void AddNewFrame(EgoFrame ego_new_frame);

  void UpdateEgoHistory(EgoFrame ego_curr_frame);

  std::deque<EgoFrame> frames_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
