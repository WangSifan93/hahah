#ifndef ONBOARD_PLANNER_SCENE_TRAFFIC_FLOW_REASONING_H_
#define ONBOARD_PLANNER_SCENE_TRAFFIC_FLOW_REASONING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "maps/lane_path.h"
#include "maps/map_def.h"
#include "object/object_history.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "prediction.pb.h"
#include "scene/scene_understanding_output.h"
#include "trajectory.pb.h"

namespace e2e_noa::planning {

struct TrafficFlowReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const ObjectsPredictionProto* prediction = nullptr;
  const std::vector<mapping::LanePath>* lane_paths = nullptr;
  const ad_e2e::planning::TrafficLightStatusMap* tl_info_map = nullptr;

  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
};

absl::StatusOr<TrafficFlowReasoningOutput> RunTrafficFlowReasoning(
    const TrafficFlowReasoningInput& input, WorkerThreadManager* thread_pool,
    ObjectHistoryController& obj_manager);
}  // namespace e2e_noa::planning

#endif
