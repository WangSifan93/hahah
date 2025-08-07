#ifndef AD_E2E_PLANNING_OBJECT_OBJECT_HISTORY_H
#define AD_E2E_PLANNING_OBJECT_OBJECT_HISTORY_H
#include <string_view>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "planner_object.pb.h"
#include "speed_planning.pb.h"
#include "util/hmi_content_util.h"
#include "util/string_string_view.h"
#include "util/time_util.h"

namespace e2e_noa {
namespace planning {

class StBoundaryWithDecision;

struct ObjectFrame {
  std::string id;
  int64_t timestamp;
  ObjectProto object_proto;
  bool is_leading = true;
  bool is_stalled = false;
  bool is_nudge = false;

  StBoundaryProto::DecisionType lon_decision = StBoundaryProto::UNKNOWN;
  SpacetimePlannerObjectTrajectoryReason::Type lat_decision =
      SpacetimePlannerObjectTrajectoryReason::NONE;
};

class ObjectHistory {
 public:
  std::deque<ObjectFrame>& GetFrames() { return frames_; }
  const ObjectFrame* GetOldestFrame() const {
    if (Empty()) {
      return nullptr;
    }
    return &(frames_.front());
  }
  const ObjectFrame* GetLatestFrame() const {
    if (Empty()) {
      return nullptr;
    }
    return &(frames_.back());
  }
  const ObjectFrame* GetFrameByTimestamp(const int64_t timestamp) const {
    if (Empty()) {
      return nullptr;
    }
    auto it = std::lower_bound(frames_.begin(), frames_.end(), timestamp,
                               [](const ObjectFrame& frame, int64_t ts) {
                                 return frame.timestamp < ts;
                               });

    if (it == frames_.end()) {
      return GetLatestFrame();
    } else if (it == frames_.begin()) {
      return GetOldestFrame();
    } else {
      auto prev_it = std::prev(it);
      return (timestamp - prev_it->timestamp < it->timestamp - timestamp)
                 ? &(*prev_it)
                 : &(*it);
    }
  }
  const size_t Size() const { return frames_.size(); }
  bool Empty() const { return frames_.empty(); }
  void CleanExceeded(const int64_t exceeded_timestamp) {
    auto iter =
        std::lower_bound(frames_.begin(), frames_.end(), exceeded_timestamp,
                         [](const ObjectFrame& data, int64_t value) {
                           return data.timestamp < value;
                         });
    frames_.erase(frames_.begin(), iter);
  }

 private:
  std::deque<ObjectFrame> frames_;
};

class ObjectHistoryController {
 public:
  using ObjectsFrame = absl::flat_hash_map<std::string, ObjectFrame>;
  ObjectHistoryController() = default;
  ~ObjectHistoryController() = default;

  bool HasObject(std::string_view obj_id) {
    return obj_history_map_.find(obj_id) != obj_history_map_.end();
  }
  const ObjectHistory* GetObjHistory(std::string_view obj_id) const {
    const auto iter = obj_history_map_.find(obj_id);
    if (iter == obj_history_map_.end()) return nullptr;
    return &(iter->second);
  }
  const ObjectFrame* GetObjLatestFrame(std::string_view obj_id) const {
    const auto iter = obj_history_map_.find(obj_id);
    if (iter == obj_history_map_.end()) return nullptr;
    return iter->second.GetLatestFrame();
  }
  const ObjectFrame* GetFrameByTimestamp(std::string_view obj_id,
                                         const int64_t timestamp) const {
    return GetObjHistory(obj_id)->GetFrameByTimestamp(timestamp);
  }

  void CleanExceeded(const int64_t cur_timestamp) {
    int64_t exceeded_timestamp = cur_timestamp - kMaxHistoryLengthUs;
    std::vector<std::string> to_erase;
    for (auto& [id, his] : obj_history_map_) {
      his.CleanExceeded(exceeded_timestamp);
      if (his.Empty()) {
        to_erase.push_back(id);
      }
    }
    for (auto& id : to_erase) {
      obj_history_map_.erase(id);
    }
  }
  void Clear() { obj_history_map_.clear(); }

  void AddObjectsFrameToHistory(
      absl::flat_hash_map<std::string, ObjectFrame>&& objects_frame) {
    for (auto& [obj_id, obj_frame] : objects_frame) {
      auto [iter, _] = obj_history_map_.try_emplace(obj_id);
      iter->second.GetFrames().emplace_back(std::move(obj_frame));
    }
  }

 private:
  absl::flat_hash_map<std::string, ObjectHistory, TransparentHash,
                      TransparentEqual>
      obj_history_map_;
  static constexpr int64_t kMaxHistoryLengthUs = 1e6;
};

void UpdateObjectsHistory(
    ObjectHistoryController& obj_history_mgr,
    const std::optional<ObjectsProto>& objects_proto,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimePlannerObjectTrajectoriesProto& st_planner_obj_trjs,
    const std::map<std::string, bool>& obj_leading,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const std::optional<NudgeObjectInfo>& nudge_info);

}  // namespace planning
}  // namespace e2e_noa

#endif
