#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_

#include <map>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "common/constants.h"
#include "common/path_sl_boundary.h"
#include "common/type_def.h"
#include "descriptor/constraint_manager.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "object/spacetime_trajectory_manager.h"
#include "optimization/ddp/trajectory_optimizer_state.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/st_planner_output.h"
#include "plan/trajectory_point.h"
#include "planner.pb.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "spacetime_search/select_nudge_object.h"
#include "spacetime_search/spacetime_search_input.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

struct TrajectoryOptimizerInput {
  absl::Span<const ApolloTrajectoryPointProto> trajectory;
  NudgeInfos nudge_info;
  absl::Span<const ApolloTrajectoryPointProto> previous_trajectory;

  std::optional<TrajectoryOptimizerState> trajectory_optimizer_state;

  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_mgr = nullptr;
  ApolloTrajectoryPointProto plan_start_point;
  absl::Time plan_start_time;
  int plan_id = 0;
  bool borrow_lane = false;
  LaneChangeStage lc_stage;
  e2e_noa::planning::PushDirection push_dir =
      e2e_noa::planning::PushDirection::Push_None;

  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  const VehicleGeometryParamsProto* veh_geo_params = nullptr;
  const VehicleDriveParamsProto* veh_drive_params = nullptr;
  const NudgeObjectInfo* nudge_object_info = nullptr;
};

}  

#endif
