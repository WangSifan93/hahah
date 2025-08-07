#ifndef PLANNER_SPEED_OBJECT_SCENE_RECOGNITION_H_
#define PLANNER_SPEED_OBJECT_SCENE_RECOGNITION_H_

#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "math/vec.h"
#include "plan/planner_defs.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "speed/st_graph.h"

namespace e2e_noa {
namespace planning {

struct DrivingProcess {
  double start_s = 0.0;
  double end_s = 0.0;
  e2e_noa::planning::MergeTopology merge_topology =
      e2e_noa::planning::MergeTopology::TOPOLOGY_MERGE_NONE;
  e2e_noa::planning::SplitTopology split_topology =
      e2e_noa::planning::SplitTopology::TOPOLOGY_SPLIT_NONE;
  InteractionZone zone_type = InteractionZone::Unknown;
  mapping::ElementId lane_id = "";
};

struct TurnTypeInfo {
  double start_s = 0.0;
  double end_s = 0.0;
  ad_e2e::planning::TurnType turn_type = ad_e2e::planning::NO_TURN;
};

void MakeObjectSceneRecognition(
    const PlannerSemanticMapManager& psmm, const PlanPassage& plan_passage,
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedPlanningParamsProto::ObjectSceneParamsProto& obj_scene_params,
    std::vector<StBoundaryRef>* st_boundaries, double av_speed,
    std::vector<DrivingProcess>* driving_process_seq,
    std::vector<TurnTypeInfo>& turn_type_info);

}  // namespace planning
}  // namespace e2e_noa
#endif
