#include "object_history.h"

#include "speed/st_boundary_with_decision.h"
namespace e2e_noa::planning {

void UpdateObjectsHistory(
    ObjectHistoryController& obj_history_mgr,
    const std::optional<ObjectsProto>& objects_proto,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimePlannerObjectTrajectoriesProto& st_planner_obj_trjs,
    const std::map<std::string, bool>& obj_leading,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const std::optional<NudgeObjectInfo>& nudge_info) {
  const int64_t cur_timestamp = objects_proto->header().timestamp();
  obj_history_mgr.CleanExceeded(cur_timestamp);
  absl::flat_hash_map<std::string, ObjectFrame> objects_frame;

  for (const auto& object : objects_proto->objects()) {
    objects_frame.try_emplace(object.id(),
                              ObjectFrame{.id = object.id(),
                                          .timestamp = cur_timestamp,
                                          .object_proto = object});

    if (nudge_info.has_value() && (object.id() == nudge_info->id)) {
      objects_frame[object.id()].is_nudge = true;
      const std::string obs_debug = "id: " + object.id() + " is_nudge";
    }
  }

  for (const auto& lead : obj_leading) {
    objects_frame.at(lead.first).is_leading = lead.second;
    const std::string obs_debug =
        absl::StrCat("id: ", lead.first, "unleading: ", lead.second);
  }
  for (const auto& stalled : stalled_objects) {
    objects_frame.at(stalled).is_stalled = true;
    const std::string obs_debug = absl::StrCat("id: ", stalled, "is_stalled");
  }

  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (!boundary_with_decision.object_id().has_value()) {
      continue;
    }
    std::string obj_id = boundary_with_decision.object_id().value();
    objects_frame.at(obj_id).lon_decision =
        boundary_with_decision.decision_type();
  }

  for (size_t i = 0; i < st_planner_obj_trjs.trajectory_size(); ++i) {
    std::string obj_id = st_planner_obj_trjs.trajectory(i).id();
    objects_frame.at(obj_id).lat_decision =
        st_planner_obj_trjs.trajectory(i).reason();
  }

  obj_history_mgr.AddObjectsFrameToHistory(std::move(objects_frame));
}
}  // namespace e2e_noa::planning
