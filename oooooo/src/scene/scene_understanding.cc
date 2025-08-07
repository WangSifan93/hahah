#include "scene/scene_understanding.h"

#include <utility>

#include "plan/planner_flags.h"
#include "scene/scene_understanding_output.h"
#include "scene/traffic_flow_reasoning.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {

absl::StatusOr<SceneReasoningOutput> RunSceneReasoning(
    const SceneReasoningInput& input, WorkerThreadManager* thread_pool,
    ObjectHistoryController& obj_his_manager) {
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.prediction);
  CHECK_NOTNULL(input.tl_info_map);
  CHECK_NOTNULL(input.lane_paths);

  CHECK_NOTNULL(input.plan_start_point);

  const auto& psmm = *input.psmm;
  const auto& prediction = *input.prediction;
  const auto& tl_info_map = *input.tl_info_map;
  const auto& lane_paths = *input.lane_paths;

  SceneOutputProto scene_output_proto;

  const TrafficFlowReasoningInput traffic_flow_input{
      .psmm = &psmm,
      .prediction = &prediction,
      .lane_paths = &lane_paths,
      .tl_info_map = &tl_info_map,

      .plan_start_point = input.plan_start_point};

  ASSIGN_OR_RETURN(auto traffic_flow_output,
                   RunTrafficFlowReasoning(traffic_flow_input, thread_pool,
                                           obj_his_manager));

  for (auto& traffic_waiting_queue :
       traffic_flow_output.traffic_waiting_queues) {
    *scene_output_proto.add_traffic_waiting_queue() =
        std::move(traffic_waiting_queue);
  }
  for (auto& object_annotation : traffic_flow_output.object_annotations) {
    *scene_output_proto.add_objects_annotation() = std::move(object_annotation);
  }

  return SceneReasoningOutput{
      .scene_output_proto = std::move(scene_output_proto),
      .distance_to_roadblock = traffic_flow_output.distance_to_roadblock};
}

}  // namespace e2e_noa::planning
