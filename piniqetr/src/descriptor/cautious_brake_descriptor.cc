#include "descriptor/cautious_brake_descriptor.h"

#include <algorithm>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "container/strong_int.h"
#include "descriptor/descriptor_util.h"
#include "maps/lane_path.h"
#include "maps/map_def.h"
#include "math/geometry/polygon2d.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/planner_flags.h"
#include "plan/planner_util.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "util/status_macros.h"

namespace e2e_noa {
namespace planning {
namespace {
constexpr double kCrosswalkAssociationDistance = 2.0;
constexpr double kEpsilon = 0.1;
constexpr double kPlanPassageEndPointEpsilon = 0.1;
constexpr double kIntersectionSpeedReductionRatio = 0.75;
constexpr double kIntersectionLowerSpeedLimit = 60.0;
constexpr double kIntersectionUpperSpeedLimit = 80.0;
constexpr double kEmptyCrossWalkSpeedReductionRatio = 0.9;
constexpr double kVRUNearCrossWalkSpeedReductionRatio = 0.8;
constexpr double kVRUOnCrossWalkSpeedReductionRatio = 0.7;
constexpr double kBrakeDisInAdvance = 10.0;
constexpr double kEhpV2TurnSpeedLimit = Kph2Mps(40.0);
constexpr double kEhpV2TurnRightSpeedLimit = Kph2Mps(20.0);
constexpr double kEhpV2SplitSpeedLimit = Kph2Mps(15.0);

constexpr double kMinCautiousSpeed = 3.0;

std::optional<double> GetDistanceToSplit(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path_from_start, double* const lane_length,
    Vec2d* const start_pt, Vec2d* const end_pt) {
  if ((!lane_length) || (!start_pt) || (!end_pt)) return std::nullopt;
  double distance_to_split = 0.0;
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      return std::nullopt;
    }
    if (lane_info_ptr->split_topology() ==
        e2e_noa::planning::SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
      *lane_length = lane_info_ptr->curve_length() * (1.0 - seg.start_fraction);
      *start_pt = lane_info_ptr->LerpPointFromFraction(seg.start_fraction);
      *end_pt = lane_info_ptr->LerpPointFromFraction(1.0);
      return distance_to_split;
    }
    if (lane_info_ptr->turn_type() != e2e_noa::planning::TurnType::NO_TURN) {
      return std::nullopt;
    }
    distance_to_split += (seg.end_s - seg.start_s);
  }
  return std::nullopt;
}

std::optional<double> GetDistanceToUturn(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path_from_start, double* const lane_length,
    Vec2d* const start_pt, Vec2d* const end_pt) {
  if ((!lane_length) || (!start_pt) || (!end_pt)) return std::nullopt;
  double distance_to_split = 0.0;
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      return std::nullopt;
    }
    if (lane_info_ptr->turn_type() == e2e_noa::planning::TurnType::U_TURN) {
      *lane_length = lane_info_ptr->curve_length() * (1.0 - seg.start_fraction);
      *start_pt = lane_info_ptr->LerpPointFromFraction(seg.start_fraction);
      *end_pt = lane_info_ptr->LerpPointFromFraction(1.0);
      return distance_to_split;
    }
    if (lane_info_ptr->turn_type() != e2e_noa::planning::TurnType::NO_TURN) {
      return std::nullopt;
    }
    distance_to_split += (seg.end_s - seg.start_s);
  }
  return std::nullopt;
}

absl::StatusOr<ConstraintProto::SpeedRegionProto> ProcessMapElement(
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    const mapping::LanePath::LaneSegment& seg,
    const ad_e2e::planning::Lane& lane_info,
    const std::pair<mapping::ElementId, Vec2d>& element, double s_offset,
    double lane_speed_limit, double speed_reduction_ratio) {
  if (seg.start_fraction > element.second.y()) {
    return absl::NotFoundError("Map Element not on current seg.");
  }

  const double passage_min_s = passage.front_s();
  const double passage_max_s = passage.end_s();
  const auto start_point = lane_info.LerpPointFromFraction(element.second.x());

  const auto end_point = lane_info.LerpPointFromFraction(element.second.y());
  ConstraintProto::SpeedRegionProto cautious_brake_constraint;
  start_point.ToProto(cautious_brake_constraint.mutable_start_point());
  end_point.ToProto(cautious_brake_constraint.mutable_end_point());

  const double start_point_s = lane_path_from_start.LaneIndexPointToArclength(
                                   {seg.lane_index, element.second.x()}) +
                               s_offset + passage.lane_path_start_s();
  const double end_point_s = lane_path_from_start.LaneIndexPointToArclength(
                                 {seg.lane_index, element.second.y()}) +
                             s_offset + passage.lane_path_start_s();

  if (start_point_s >= passage_max_s - kPlanPassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element beyond current plan passage. "
                        "start_point_s: %f, passage_max_s:%f",
                        start_point_s, passage_max_s));
  } else if (end_point_s <= passage_min_s + kPlanPassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element behind current plan passage. "
                        "end_point_s:%f, plan passage_min_s: %f",
                        end_point_s, passage_min_s));
  } else if (start_point_s + kPlanPassageEndPointEpsilon >= end_point_s) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element range on plan passage is too short. "
                        "start_pont_s: %f, end_point_s: %f",
                        start_point_s, end_point_s));
  }
  cautious_brake_constraint.set_start_s(std::max(start_point_s, passage_min_s));
  cautious_brake_constraint.set_end_s(std::min(end_point_s, passage_max_s));
  const auto speed_limit =
      std::max(kMinCautiousSpeed, lane_speed_limit * speed_reduction_ratio);
  cautious_brake_constraint.set_max_speed(speed_limit);

  return cautious_brake_constraint;
}
}  // namespace

std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const SpacetimeTrajectoryManager& st_mgr) {
  absl::flat_hash_map<std::string,
                      std::vector<ConstraintProto::SpeedRegionProto>>
      id_element_map;
  for (const auto& seg : lane_path_from_start) {
    const auto lane_info_ptr =
        planner_semantic_map_manager.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    const double speed_limit = std::fmax(
        Kph2Mps(kIntersectionLowerSpeedLimit),
        planner_semantic_map_manager.QueryLaneSpeedLimitById(seg.lane_id));
  }

  std::vector<ConstraintProto::SpeedRegionProto> cautious_brake_constraints;
  cautious_brake_constraints.reserve(id_element_map.size());
  for (const auto& pair : id_element_map) {
    cautious_brake_constraints.push_back(MergeSameElement(pair.second));
  }
  return cautious_brake_constraints;
}

}  // namespace planning
}  // namespace e2e_noa
