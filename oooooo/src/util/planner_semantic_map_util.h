#ifndef ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "lane_path.pb.h"
#include "maps/lane_path.h"
#include "maps/semantic_map_defs.h"
#include "math/geometry/box2d.h"
#include "math/vec.h"
#include "plan/planner_defs.h"
#include "plan/planner_semantic_map_manager.h"

namespace e2e_noa::planning {

struct SamplePathPointsResult {
  std::vector<Vec2d> points;
  bool is_partial;
  std::string message;
};

bool IsOutgoingLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const ad_e2e::planning::Lane& source_lane, mapping::ElementId out_lane_id);

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s);

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id);

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s);

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id);

bool IsLanePathBlockedByBox2d(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const Box2d& box, const mapping::LanePath& lane_path, double lat_thres);

std::vector<Vec2d> SampleLanePathPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path);

absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePathProto& lane_path);

absl::StatusOr<mapping::LanePath> ClampLanePathFromPos(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, const Vec2d& pos);

absl::StatusOr<mapping::LanePoint> FindOutgoingLanePointWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm, mapping::ElementId id);

}  // namespace e2e_noa::planning

#endif
