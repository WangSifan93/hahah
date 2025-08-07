#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_H_

#include <string_view>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "object/object_vector.h"
#include "object/planner_object.h"
#include "util/map_util.h"
#include "util/string_string_view.h"

namespace e2e_noa {
namespace planning {

class PlannerObjectController {
 public:
  PlannerObjectController() = default;

  explicit PlannerObjectController(ObjectVector<PlannerObject> planner_objects);

  const ObjectVector<PlannerObject>& planner_objects() const {
    return planner_objects_;
  }

  absl::Span<const PlannerObject* const> stationary_objects() const {
    return stationary_objects_;
  }

  const PlannerObject* FindObjectById(const absl::string_view id) const {
    const auto* found_ptr = FindOrNull(object_map_, std::string(id));
    if (found_ptr == nullptr) return nullptr;
    return *found_ptr;
  }

  absl::Span<const PlannerObject* const> moving_objects() const {
    return moving_objects_;
  }

  const PlannerObject& planner_object(const ObjectIndex index) const {
    return planner_objects_[index];
  }

  int size() const { return planner_objects_.size(); }

 private:
  ObjectVector<PlannerObject> planner_objects_;
  std::vector<const PlannerObject*> stationary_objects_;
  std::vector<const PlannerObject*> moving_objects_;
  absl::flat_hash_map<std::string, const PlannerObject*, TransparentHash,
                      TransparentEqual>
      object_map_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
