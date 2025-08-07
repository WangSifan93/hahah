#ifndef ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_OUTPUT_H_
#define ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_OUTPUT_H_

#include <vector>

#include "scene_understanding.pb.h"

namespace e2e_noa::planning {
struct TrafficFlowReasoningOutput {
  std::vector<TrafficWaitingQueueProto> traffic_waiting_queues;
  std::vector<ObjectAnnotationProto> object_annotations;
  std::optional<double> distance_to_roadblock = std::nullopt;
};

}  // namespace e2e_noa::planning
#endif
