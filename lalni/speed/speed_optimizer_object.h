#ifndef PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_H_
#define PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "math/util.h"
#include "planner_params.pb.h"
#include "speed_planning.pb.h"
#include "util/map_util.h"

namespace e2e_noa::planning {

struct ObjectOverlapState {
  double bound = 0.0;
  double speed = 0.0;
  double prob = 0.0;
  double delta_speed_factor = 0.0;
  double lon_buffer = 0.0;
};

class SpeedOptimizerObject {
 public:
  SpeedOptimizerObject(
      std::vector<std::optional<ObjectOverlapState>> overlap_states,
      double state_time_interval, absl::string_view id, double standstill,
      StBoundaryProto::ObjectType object_type,
      StBoundarySourceTypeProto::Type source_type,
      StBoundaryProto::ProtectionType protection_type)
      : overlap_states_(std::move(overlap_states)),
        state_time_interval_(state_time_interval),
        id_(id),
        standstill_(standstill),
        object_type_(object_type),
        source_type_(source_type),
        protection_type_(protection_type) {}

  const std::optional<ObjectOverlapState>& GetOverlapStateByIndex(
      int idx) const {
    CHECK_LT(idx, overlap_states_.size());
    return overlap_states_[idx];
  }

  const std::optional<ObjectOverlapState>& GetOverlapStateByTime(
      double time) const {
    const int idx = FloorToInt(time / state_time_interval_);
    CHECK_LT(idx, overlap_states_.size());
    return overlap_states_[idx];
  }

  absl::string_view id() const { return id_; }

  bool is_protective() const {
    return protection_type_ != StBoundaryProto::NON_PROTECTIVE;
  }

  StBoundaryProto::ProtectionType protection_type() const {
    return protection_type_;
  }

  StBoundaryProto::ObjectType object_type() const { return object_type_; }

  StBoundarySourceTypeProto::Type source_type() const { return source_type_; }

  double standstill() const { return standstill_; }

 private:
  std::vector<std::optional<ObjectOverlapState>> overlap_states_;
  double state_time_interval_ = 0.0;

  std::string id_;
  double standstill_ = 0.0;
  StBoundaryProto::ObjectType object_type_;
  StBoundarySourceTypeProto::Type source_type_;
  StBoundaryProto::ProtectionType protection_type_;
};

}  // namespace e2e_noa::planning

#endif
