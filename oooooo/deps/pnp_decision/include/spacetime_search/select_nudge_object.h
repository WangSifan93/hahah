#ifndef SPACETIME_SEARCH_SELECT_NUDGE_OBJECT_H_
#define SPACETIME_SEARCH_SELECT_NUDGE_OBJECT_H_

#include <vector>

#include "common/path_sl_boundary.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "plan/planner_defs.h"
#include "plan/trajectory_point.h"
#include "router/plan_passage.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {
struct NudgeInfos {
  std::vector<NudgeObjectInfo> nudgeInfos;
  void addNudgeInfo(const NudgeObjectInfo& info) { nudgeInfos.push_back(info); }
  std::optional<NudgeObjectInfo> findNudgeInfoById(
      const std::string& id) const {
    for (const auto& info : nudgeInfos) {
      if (info.id == id) {
        return info;
      }
    }
    return std::nullopt;
  }
};
namespace spacetime_search {
absl::StatusOr<NudgeInfos> SelectNudgeObjectId(
    int trajectory_steps, double trajectory_time_step, bool is_lane_change,
    const PlanPassage& plan_passage, const PathSlBoundary& path_sl_boundary,
    const std::vector<TrajectoryPoint>& result_points,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const int plan_id);
}
}  
}  

#endif
