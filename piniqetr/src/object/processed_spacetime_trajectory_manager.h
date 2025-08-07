#ifndef ONBOARD_PLANNER_OBJECT_PROCESSED_SPACETIME_TRAJECTORY_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_PROCESSED_SPACETIME_TRAJECTORY_MANAGER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "async/thread_pool.h"
#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "planner_object.pb.h"
#include "prediction/predicted_trajectory.h"
#include "speed/speed_planning_input.h"
#include "speed_planning_params.pb.h"
#include "util/map_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {

namespace planning {

class ProcessedSpacetimeTrajectoryController
    : public SpacetimeTrajectoryManager {
 public:
  ProcessedSpacetimeTrajectoryController() {}
  ProcessedSpacetimeTrajectoryController(
      const SpacetimeTrajectoryManager& other)
      : SpacetimeTrajectoryManager(other) {}

  void ModifySpacetimeTrajectory(
      const SpeedPlanningInput& input,
      const VehicleGeometryParamsProto& vehicle_geometry_params,
      const SpeedPlanningParamsProto& speed_default_params);
};

}  // namespace planning
}  // namespace e2e_noa

#endif
