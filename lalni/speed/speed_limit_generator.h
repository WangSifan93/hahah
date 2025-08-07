#ifndef PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_
#define PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_

#include <map>
#include <vector>

#include "behavior.pb.h"
#include "descriptor/constraint_manager.h"
#include "lane_change.pb.h"
#include "plan/discretized_path.h"
#include "plan/ego_history.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "speed/object_scene_recognition.h"
#include "speed/open_loop_speed_limit.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/speed_limit.h"
#include "speed/st_close_trajectory.h"
#include "speed/st_graph_defs.h"
#include "speed/vt_speed_limit.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

struct SpeedLimitMapInput {
  const SpeedPlanningParamsProto::SpeedLimitParamsProto* speed_limit_config =
      nullptr;
  const VehicleGeometryParamsProto* veh_geo_params = nullptr;
  const VehicleDriveParamsProto* veh_drive_params = nullptr;

  const double max_speed_limit = 0.0;
  const double av_max_acc = 0.0;
  const bool is_narrow_near_large_vehicle = false;
  const double av_speed = 0.0;
  const double av_acc = 0.0;

  const DiscretizedPath* discretized_points = nullptr;
  const std::vector<PathPoint>* st_path_points = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const Behavior* behavior = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
  const NudgeObjectInfo* nudge_object_info = nullptr;
  const EgoHistory* ego_history = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::vector<DistanceInfo>* distance_info_to_impassable_path_boundaries =
      nullptr;
  const std::vector<StCloseTrajectory>* moving_close_trajs = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const std::vector<TurnTypeInfo>* turn_type_info = nullptr;
  const DiscretizedPath* ego_predict_trajectory = nullptr;
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const std::vector<StBoundaryWithDecision>* st_boundaries_with_decision =
      nullptr;
  const std::vector<DrivingProcess>* driving_process_seq = nullptr;
};

struct VtSpeedLimitMapInput {
  const int traj_steps = 0;
  const double time_step = 0.0;
  const double plan_start_v = 0.0;
  const double plan_start_a = 0.0;
  const double soft_acc_jerk = 0.0;

  const ConstraintManager* constraint_mgr = nullptr;
  const SpacetimeConstraintParamsProto* spacetime_constraint_params = nullptr;
  const std::optional<VtSpeedLimit> parallel_cut_in_speed_limit = std::nullopt;
  const std::optional<VtSpeedLimit> ignore_speed_limit = std::nullopt;
};

struct OpenLoopSpeedLimitInput {
  const bool is_narrow_near_large_vehicle = false;
  const bool is_left_turn_ignore = false;
  const bool is_av_occupy_target_lane = false;
  const bool consider_lane_change_gap = true;
  const int trajectory_steps = 0;
  const double plan_start_v = 0.0;
  const double plan_start_a = 0.0;
  const double av_max_acc = 0.0;
  const ConstraintManager* constraint_mgr = nullptr;
  const LaneChangeStage* lc_stage = nullptr;
  const std::vector<StBoundaryWithDecision>* st_boundaries_with_decision =
      nullptr;
};

std::map<SpeedLimitTypeProto::Type, SpeedLimit> GetSpeedLimitMap(
    const SpeedLimitMapInput& input, EgoFrame* curr_ego_frame);

std::map<SpeedLimitTypeProto::Type, VtSpeedLimit> GetVtSpeedLimitMap(
    const VtSpeedLimitMapInput& input);

std::optional<VtSpeedLimit> GetOpenLoopVtSpeedLimit(
    const OpenLoopSpeedLimitInput& input,
    OpenLoopSpeedLimit& open_loop_speed_limit);
}  // namespace e2e_noa::planning
#endif
