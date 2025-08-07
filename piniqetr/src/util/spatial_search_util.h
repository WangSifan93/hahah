#ifndef ONBOARD_PLANNER_UTIL_SPATIAL_SEARCH_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SPATIAL_SEARCH_UTIL_H_

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "maps/semantic_map_defs.h"
#include "math/vec.h"
#include "plan/planner_semantic_map_manager.h"

namespace e2e_noa::planning {

constexpr double kSearchRadiusThreshold = 10.0;

absl::StatusOr<mapping::LanePoint> FindClosestLanePointToSmoothPointAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    Vec2d* closest_point_on_lane = nullptr,
    double cutoff_distance = kSearchRadiusThreshold);

absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    double heading, double heading_penalty_weight = 0.0,
    Vec2d* closest_point_on_lane = nullptr,
    double cutoff_distance = kSearchRadiusThreshold);

absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const mapping::LanePath& lane_path, double heading,
    double heading_penalty_weight = 0.0, Vec2d* closest_point_on_lane = nullptr,
    double cutoff_distance = kSearchRadiusThreshold);

std::optional<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    mapping::ElementId lane_id, double heading,
    double heading_penalty_weight = 0.0, Vec2d* closest_point_on_lane = nullptr,
    double start_fraction = 0.0, double end_fraction = 1.0,
    double cutoff_distance = kSearchRadiusThreshold);

template <typename T>
absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const T& lane_ids, double heading, double heading_penalty_weight = 0.0,
    Vec2d* closest_point_on_lane = nullptr, double start_fraction = 0.0,
    double end_fraction = 1.0, double cutoff_distance = kSearchRadiusThreshold);

std::vector<std::pair<double, mapping::LanePoint>>
FindCloseLanePointsAndDistanceToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    double heading, double heading_penalty_weight = 0.0,
    double spatial_distance_threshold = 1.5,
    double angle_error_threshold = 0.35);

std::vector<mapping::LanePoint>
FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    double heading, double heading_penalty_weight = 0.0,
    double spatial_distance_threshold = 1.5,
    double angle_error_threshold = 0.35);

bool IsPointOnLanePathAtLevel(const PlannerSemanticMapManager& psmm,
                              const Vec2d& query_point,
                              const mapping::LanePath& lane_path,
                              double* arc_len_on_lane_path,
                              double lateral_error_buffer = 2.0);

}  // namespace e2e_noa::planning

#endif
