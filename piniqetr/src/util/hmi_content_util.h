#ifndef ONBOARD_PLANNER_UTIL_HMI_CONTENT_UTIL_H_
#define ONBOARD_PLANNER_UTIL_HMI_CONTENT_UTIL_H_

#include <string>
#include <vector>

#include "common/path_sl_boundary.h"
#include "hmi_content.pb.h"
#include "maps/lane_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "selector_state.pb.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {

struct NudgeObjectInfo {
  enum Type {
    NORMAL = 0,
    LARGE_VEHICLE = 1,
  };
  enum NudgeState {
    NUDGE = 0,
    BORROW = 1,
  };
  std::string id;
  int direction;
  double arc_dist_to_object;
  ObjectType type;
  NudgeState nudge_state;
};

struct HmiContentInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const mapping::LanePath* lane_path = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* traj_points = nullptr;
  const std::string* alerted_front_vehicle = nullptr;
  LaneChangeReason lane_change_reason;
  bool borrow_lane = false;
  bool request_help_lane_change_by_route = false;
  const double* distance_to_traffic_light_stop_line = nullptr;
  const double* distance_to_roadblock = nullptr;
  const std::vector<std::string>* unsafe_object_ids = nullptr;
  const mapping::LanePath* plc_target_lane_path = nullptr;
  const NudgeObjectInfo* nudge_object_info = nullptr;
};

HmiContentProto ReportHmiContent(const HmiContentInput& input);

HmiPathBoundaryProto ReportBoundaryPointsToHmiContent(
    const std::vector<Vec2d>& points, bool is_left,
    HmiPathBoundaryProto::BoundaryRenderStyle style);

HmiPathBoundaryProto ReportPathBoundaryToHmiContent(
    const PathSlBoundary* sl_boundary,
    HmiPathBoundaryProto::BoundaryRenderStyle left_style =
        HmiPathBoundaryProto::STYLE_NORMAL,
    HmiPathBoundaryProto::BoundaryRenderStyle right_style =
        HmiPathBoundaryProto::STYLE_NORMAL);
}  // namespace planning
}  // namespace e2e_noa

#endif
