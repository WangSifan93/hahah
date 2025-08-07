#ifndef ONBOARD_PLANNER_UTIL_LANE_PATH_UTIL_H_
#define ONBOARD_PLANNER_UTIL_LANE_PATH_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"
#include "maps/lane_path.h"
#include "maps/lane_path_data.h"
#include "plan/planner_semantic_map_manager.h"
#include "positioning.pb.h"

namespace e2e_noa::planning {

absl::StatusOr<mapping::LanePath> BuildLanePathFromData(
    const mapping::LanePathData& data, const PlannerSemanticMapManager& psmm);

bool IsLanePathConnectedTo(const mapping::LanePath& lane_path,
                           const mapping::LanePath& other,
                           const PlannerSemanticMapManager& psmm,
                           double distance_threshold = 0.01);

absl::StatusOr<mapping::LanePath> ConnectLanePath(
    const mapping::LanePath& lane_path, const mapping::LanePath& other,
    const PlannerSemanticMapManager& psmm, double distance_threshold = 0.01);

mapping::LanePath BackwardExtendTargetAlignedRouteLanePath(
    const PlannerSemanticMapManager& psmm, bool left,
    const mapping::LanePoint start_point, const mapping::LanePath& target);

mapping::LanePath BackwardExtendLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len,
    const std::function<bool(const ad_e2e::planning::Lane&)>*
        nullable_should_stop_and_avoid_extend = nullptr);

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len);

mapping::LanePath ForwardExtendLanePathWithoutFork(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> FindNearestLanePathFromEgoPose(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    double required_min_length);

Vec2d ArclengthToPos(const PlannerSemanticMapManager& psmm,
                     const mapping::LanePath& lane_path, double s);

double ArclengthToLerpTheta(const PlannerSemanticMapManager& psmm,
                            const mapping::LanePath& lane_path, double s);

double LaneIndexPointToLerpTheta(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const mapping::LanePath::LaneIndexWaypoint& lane_index_point);

std::vector<ad_e2e::planning::LaneConstPtr> GetLanesInfoBreakIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

std::vector<ad_e2e::planning::LaneConstPtr> GetLanesInfoContinueIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

absl::StatusOr<mapping::LanePath> TrimTrailingNotFoundLanes(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

mapping::LanePath ForwardExtendLanePath(const PlannerSemanticMapManager& psmm,
                                        const mapping::LanePath& raw_lane_path,
                                        double extend_len,
                                        bool report_issue = true);

std::vector<mapping::LanePath> CollectAllLanePathFromStartLane(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& start_lane,
    double max_search_len);

absl::StatusOr<std::vector<Vec2d>> SampleLanePathByStep(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step);

}  // namespace e2e_noa::planning

#endif
