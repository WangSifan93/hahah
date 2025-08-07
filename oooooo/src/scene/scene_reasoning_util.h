#ifndef ONBOARD_PLANNER_SCENE_SCENE_REASONING_UTIL_H_
#define ONBOARD_PLANNER_SCENE_SCENE_REASONING_UTIL_H_

#include "object/planner_object_manager.h"
#include "planner.pb.h"
#include "scene_understanding.pb.h"

namespace e2e_noa::planning {
void ParseObjectAnnotationToDebugProto(
    const ::google::protobuf::RepeatedPtrField<ObjectAnnotationProto>&
        objects_annotation,
    const PlannerObjectController& object_manager, PlannerDebugProto* debug);

void ParseTrafficWaitingQueueToDebugProto(
    const ::google::protobuf::RepeatedPtrField<TrafficWaitingQueueProto>&
        traffic_waiting_queues,
    const PlannerObjectController& object_manager, PlannerDebugProto* debug);
}  // namespace e2e_noa::planning

#endif
