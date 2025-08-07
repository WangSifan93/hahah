#ifndef ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_H_
#define ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "maps/lane_path.h"
#include "maps/map_def.h"
#include "object/object_history.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "prediction.pb.h"
#include "router/route_sections.h"
#include "scene_understanding.pb.h"
#include "trajectory.pb.h"

namespace e2e_noa::planning {

struct SceneReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const ObjectsPredictionProto* prediction = nullptr;
  const ad_e2e::planning::TrafficLightStatusMap* tl_info_map = nullptr;

  const std::vector<mapping::LanePath>* lane_paths = nullptr;

  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
};

struct SceneReasoningOutput {
  SceneOutputProto scene_output_proto;
  std::optional<double> distance_to_roadblock = std::nullopt;
};

absl::StatusOr<SceneReasoningOutput> RunSceneReasoning(
    const SceneReasoningInput& input, WorkerThreadManager* thread_pool,
    ObjectHistoryController& obj_his_manager);

}  // namespace e2e_noa::planning

#endif
