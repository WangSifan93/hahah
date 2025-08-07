#ifndef ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "constraint.pb.h"
#include "crosswalk_state.pb.h"
#include "maps/lane_path.h"
#include "object/planner_object_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct CrosswalkDescriptor {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> speed_regions;
  std::vector<CrosswalkStateProto> crosswalk_states;
};

struct CrosswalkDescriptorInput {
  const e2e_noa::VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const PlanPassage* passage = nullptr;
  const mapping::LanePath* lane_path_from_start = nullptr;
  const PlannerObjectController* obj_mgr = nullptr;
  const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>*
      last_crosswalk_states = nullptr;

  double now_in_seconds;
  double s_offset;
  bool has_traffic_light;
};

absl::StatusOr<CrosswalkDescriptor> BuildCrosswalkConstraints(
    const CrosswalkDescriptorInput& input);
}  // namespace planning
}  // namespace e2e_noa
#endif
