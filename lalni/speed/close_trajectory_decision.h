#ifndef PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_
#define PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "object_scene_recognition.h"
#include "plan/discretized_path.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/speed_limit.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_close_trajectory.h"
#include "speed/st_graph.h"
#include "vehicle.pb.h"
namespace e2e_noa::planning {
struct RightTurnCloseSpeedLimitInput {
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  double current_v = 0.0;
  double current_a = 0.0;
  const DiscretizedPath* path = nullptr;
  const DiscretizedPath* ego_predict_path = nullptr;
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const PlanPassage* plan_passage = nullptr;
};

std::vector<std::optional<SpeedLimit>> GetMovingCloseTrajSpeedLimits(
    absl::Span<const StCloseTrajectory> st_close_trajs, double path_length,
    double av_speed, double time_step, double max_time);

std::optional<SpeedLimit> GetRightTurnCloseSpeedLimit(
    const RightTurnCloseSpeedLimitInput& input,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const std::vector<DrivingProcess>& driving_process_seq);

}  // namespace e2e_noa::planning

#endif
