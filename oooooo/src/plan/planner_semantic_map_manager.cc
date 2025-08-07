#include "plan/planner_semantic_map_manager.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>

#include "async/parallel_for.h"
#include "base/macros.h"
#include "maps/map.h"
#include "maps/maps_helper.h"
#include "math/geometry/segment2d.h"
#include "math/piecewise_const_function.h"
#include "math/util.h"
#include "plan/planner_flags.h"
#include "util/map_util.h"
namespace e2e_noa::planning {

namespace {

const PiecewiseConstFunction<int, double> kModSpeedLimitFraction(
    {0, 40, 60, 70, 80, 320}, {0.3, 0.2, 0.1, 0.05, 0.0});

std::optional<double> GetDistanceBetweenLinesWithTangents(
    const Vec2d& point, PlannerSemanticMapManager::Side side,
    const Segment2d& boundary_seg, const Segment2d& norm_seg,
    double lane_heading) {
  Vec2d intersection;
  if (!norm_seg.GetIntersect(boundary_seg, &intersection)) {
    return std::nullopt;
  }

  Segment2d aligned_boundary_seg = boundary_seg;
  if (std::fabs(NormalizeAngle(aligned_boundary_seg.heading() - lane_heading)) >
      M_PI_2) {
    aligned_boundary_seg.Reverse();
  }
  double prod = aligned_boundary_seg.ProductOntoUnit(point);
  if (side == PlannerSemanticMapManager::Side::kLEFT) prod *= -1.0;
  return std::copysign(aligned_boundary_seg.DistanceTo(point), prod);
}

}  // namespace

PlannerSemanticMapManager::PlannerSemanticMapManager(
    ad_e2e::planning::MapPtr map_ptr)
    : map_ptr_(std::move(map_ptr)) {}

PlannerSemanticMapManager::PlannerSemanticMapManager(
    ad_e2e::planning::MapPtr map_ptr, PlannerSemanticMapModification modifier)
    : map_ptr_(std::move(map_ptr)), modifier_(std::move(modifier)) {}

double PlannerSemanticMapManager::QueryLaneSpeedLimitById(
    mapping::ElementId id) const {
  const auto& lane_ptr = FindLaneByIdOrNull(id);
  if (lane_ptr == nullptr) {
    return modifier_.max_speed_limit;
  }

  const auto& lane = *lane_ptr;

  const int lane_speed_limit_kph = RoundToInt(Mps2Kph(lane.speed_limit()));

  const double speed_limit_crease_factor =
      FLAGS_planner_override_lane_speed_limit_proportion == 0.0
          ? (FLAGS_planner_enable_dynamic_lane_speed_limit
                 ? kModSpeedLimitFraction(lane_speed_limit_kph)
                 : 0.0)
          : FLAGS_planner_override_lane_speed_limit_proportion;

  const double increased_lane_speed_limit =
      lane.speed_limit() * (1.0 + speed_limit_crease_factor);

  double speed_limit =
      std::min(modifier_.max_speed_limit, increased_lane_speed_limit);

  if (const auto it = modifier_.lane_speed_limit_map.find(id);
      it != modifier_.lane_speed_limit_map.end()) {
    speed_limit = std::min(it->second, speed_limit);
  }

  return speed_limit;
}

std::vector<ad_e2e::planning::LaneConstPtr>
PlannerSemanticMapManager::GetLanesInRadius(const Vec2d& smooth_coord,
                                            double radius) const {
  const auto lane_vec =
      map_ptr_->FindLanesInRadius(smooth_coord.x(), smooth_coord.y(), radius);
  return lane_vec;
}

ad_e2e::planning::LaneConstPtr
PlannerSemanticMapManager::GetNearestLaneWithHeading(
    const Vec2d& smooth_coord, double theta, double radius,
    double max_heading_diff) const {
  const auto found_segments = map_ptr_->FindLaneSegmentsInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);
  if (UNLIKELY(found_segments.empty())) return nullptr;

  double min_dist = std::numeric_limits<double>::infinity();
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  for (const auto& segment : found_segments) {
    const auto& lane_ptr = map_ptr_->GetLaneById(segment.element_id);
    if (LIKELY(lane_ptr != nullptr)) {
      DCHECK_GE(segment.segment_id.value(), 0);
      DCHECK_LT(segment.segment_id.value(), lane_ptr->points().size() - 1);
      const Segment2d smooth_seg(
          lane_ptr->points()[segment.segment_id.value()],
          lane_ptr->points()[segment.segment_id.value() + 1]);
      const double heading_diff = NormalizeAngle(theta - smooth_seg.heading());
      if (std::fabs(heading_diff) > max_heading_diff) {
        continue;
      }
      const double dist = smooth_seg.DistanceTo(smooth_coord);
      if (dist < min_dist) {
        min_dist = dist;
        lane_id = segment.element_id;
      }
    }
  }
  return map_ptr_->GetLaneById(lane_id);
}

bool PlannerSemanticMapManager::GetLaneProjection(
    const Vec2d& smooth_coord, mapping::ElementId lane_id,
    double* const fraction, Vec2d* const point, double* const min_dist,
    Segment2d* const segment) const {
  const auto& lane_info_ptr = FindLaneByIdOrNull(lane_id);
  if (UNLIKELY(lane_info_ptr == nullptr)) return false;

  const auto& points = lane_info_ptr->points();
  if (UNLIKELY(points.size() < 2)) return false;

  std::vector<Segment2d> segments = mapping::Vec2dToSegments(points);

  double min_distance = std::numeric_limits<double>::infinity();
  Vec2d proj_point;
  Vec2d* const point_ptr = point == nullptr ? nullptr : &proj_point;
  int min_idx = -1;
  for (int i = 0; i < segments.size(); ++i) {
    const double cur_dist = segments[i].DistanceTo(smooth_coord, point_ptr);
    if (cur_dist < min_distance) {
      min_idx = i;
      min_distance = cur_dist;
      if (point != nullptr) {
        *point = proj_point;
      }
    }
  }
  if (segment != nullptr) {
    *segment = segments[min_idx];
  }
  if (min_dist != nullptr) {
    *min_dist = min_distance;
  }

  if (fraction != nullptr) {
    double accum_s = 0.0;
    for (int i = 0; i < min_idx; ++i) {
      accum_s += segments[i].length();
    }
    accum_s += segments[min_idx].ProjectOntoUnit(smooth_coord);
    *fraction = std::clamp(accum_s / lane_info_ptr->curve_length(), 0.0, 1.0);
  }
  return true;
}

ad_e2e::planning::LaneConstPtr PlannerSemanticMapManager::GetNearestLane(
    const Vec2d& smooth_coord) const {
  const auto lane =
      map_ptr_->FindNearestLane(smooth_coord.x(), smooth_coord.y());
  return lane;
}

std::optional<double> PlannerSemanticMapManager::ComputeLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id, Side side) const {
  ad_e2e::planning::LaneConstPtr lane = FindLaneByIdOrNull(lane_id);
  if (lane == nullptr) return std::nullopt;

  Segment2d nearest_lane_segment;
  if (!GetLaneProjection(smooth_coord, lane_id, nullptr, nullptr, nullptr,
                         &nearest_lane_segment)) {
    return std::nullopt;
  }

  constexpr double kMaxLaneWidth = 5.0;
  const Vec2d normal =
      Vec2d::FastUnitFromAngle(nearest_lane_segment.heading()).Perp();
  const Segment2d normal_seg(smooth_coord + normal * kMaxLaneWidth,
                             smooth_coord - normal * kMaxLaneWidth);

  const auto& boundaries =
      side == Side::kLEFT ? lane->left_boundary() : lane->right_boundary();
  std::vector<Segment2d> boundary_segments;
  for (const auto& boundary : boundaries->lane_boundaries()) {
    if (boundary == nullptr || boundary->points().size() < 2) continue;
    const auto segs = mapping::Vec2dToSegments(boundary->points());
    boundary_segments.insert(boundary_segments.end(), segs.begin(), segs.end());
  }

  std::optional<double> width;
  for (const auto& boundary_seg : boundary_segments) {
    if (boundary_seg.DistanceTo(smooth_coord) > kMaxLaneWidth) {
      continue;
    }
    const auto dist = GetDistanceBetweenLinesWithTangents(
        smooth_coord, side, boundary_seg, normal_seg,
        nearest_lane_segment.heading());
    if (!dist.has_value()) continue;
    if (!width.has_value() || std::fabs(*dist) < std::fabs(*width)) {
      width = *dist;
    }
  }
  return width;
}

std::optional<double> PlannerSemanticMapManager::GetLeftLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id) const {
  return ComputeLaneWidth(smooth_coord, lane_id, Side::kLEFT);
}

std::optional<double> PlannerSemanticMapManager::GetRightLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id) const {
  return ComputeLaneWidth(smooth_coord, lane_id, Side::kRIGHT);
}

std::vector<ad_e2e::planning::LaneBoundaryConstPtr>
PlannerSemanticMapManager::GetLaneBoundaries(const Vec2d& smooth_coord,
                                             double radius) const {
  const auto boundary_vec = map_ptr_->FindLaneBoundariesInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_e2e::planning::LaneBoundaryConstPtr> lane_boundaries;
  lane_boundaries.reserve(boundary_vec.size());
  for (const auto& lane_boundary : boundary_vec) {
    const auto& lane_boundary_ptr =
        FindLaneBoundaryByIdOrNull(lane_boundary->id());
    if (lane_boundary_ptr == nullptr) continue;
    lane_boundaries.push_back(lane_boundary_ptr);
  }
  return lane_boundaries;
}

std::vector<ad_e2e::planning::RoadBoundaryConstPtr>
PlannerSemanticMapManager::GetRoadBoundaries(const Vec2d& smooth_coord,
                                             double radius) const {
  const auto boundary_vec = map_ptr_->FindRoadBoundariesInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_e2e::planning::RoadBoundaryConstPtr> road_boundaries;
  road_boundaries.reserve(boundary_vec.size());
  for (const auto& road_boundary : boundary_vec) {
    const auto& road_boundary_ptr =
        FindRoadBoundaryByIdOrNull(road_boundary->id());
    if (road_boundary_ptr == nullptr) continue;
    road_boundaries.push_back(road_boundary_ptr);
  }
  return road_boundaries;
}

std::vector<ad_e2e::planning::ClearAreaConstPtr>
PlannerSemanticMapManager::GetClearAreas(const Vec2d& smooth_coord,
                                         double radius) const {
  const auto clear_area_vec = map_ptr_->FindClearAreasInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_e2e::planning::ClearAreaConstPtr> clear_areas;
  clear_areas.reserve(clear_area_vec.size());
  for (const auto& clear_area : clear_area_vec) {
    const auto& clear_area_ptr = FindClearAreaByIdOrNull(clear_area->id());
    if (nullptr == clear_area_ptr) continue;
    clear_areas.push_back(clear_area_ptr);
  }
  return clear_areas;
}

std::vector<ad_e2e::planning::CrosswalkConstPtr>
PlannerSemanticMapManager::GetCrosswalks(const Vec2d& smooth_coord,
                                         double radius) const {
  const auto crosswalk_vec = map_ptr_->FindCrosswalksInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_e2e::planning::CrosswalkConstPtr> crosswalks;
  crosswalks.reserve(crosswalk_vec.size());
  for (const auto& crosswalk : crosswalk_vec) {
    const auto& crosswalk_ptr = FindCrosswalkByIdOrNull(crosswalk->id());
    if (crosswalk_ptr == nullptr) continue;
    crosswalks.push_back(crosswalk_ptr);
  }
  return crosswalks;
}

ad_e2e::planning::JunctionConstPtr
PlannerSemanticMapManager::GetNearestJunction(const Vec2d& smooth_coord) const {
  const auto& intersection =
      map_ptr_->FindNearestJunction(smooth_coord.x(), smooth_coord.y());
  return intersection;
}
std::vector<Segment2d> PlannerSemanticMapManager::GetImpassableBoundaries(
    const Vec2d& smooth_coord, double radius) const {
  const auto curb_vec = map_ptr_->FindRoadBoundarySegmentsInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);
  std::vector<Segment2d> segments;
  segments.reserve(curb_vec.size());
  for (const auto& curb_seg : curb_vec) {
    const auto& lane_boundary = FindRoadBoundaryByIdOrNull(curb_seg.element_id);
    if (lane_boundary != nullptr) {
      segments.emplace_back(
          lane_boundary->points()[curb_seg.segment_id.value()],
          lane_boundary->points()[curb_seg.segment_id.value() + 1]);
    }
  }
  return segments;
}
std::vector<ImpassableBoundaryInfo>
PlannerSemanticMapManager::GetImpassableBoundariesInfo(
    const Vec2d& smooth_coord, double radius) const {
  const auto curb_vec = map_ptr_->FindRoadBoundarySegmentsInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ImpassableBoundaryInfo> boundary_infos;
  boundary_infos.reserve(curb_vec.size());
  for (const auto& curb_seg : curb_vec) {
    const auto& lane_boundary = FindRoadBoundaryByIdOrNull(curb_seg.element_id);
    if (lane_boundary != nullptr) {
      auto& boundary_info = boundary_infos.emplace_back();
      boundary_info.segment =
          Segment2d(lane_boundary->points()[curb_seg.segment_id.value()],
                    lane_boundary->points()[curb_seg.segment_id.value() + 1]);
      boundary_info.height = lane_boundary->has_height()
                                 ? std::make_optional(lane_boundary->height())
                                 : std::nullopt;
      boundary_info.id = absl::StrFormat("CURB|%s|%d", curb_seg.element_id,
                                         curb_seg.segment_id);
      boundary_info.type = lane_boundary->type();
    }
  }
  return boundary_infos;
}

ad_e2e::planning::LaneConstPtr PlannerSemanticMapManager::FindLaneByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetLaneById(id);
}
ad_e2e::planning::LaneConstPtr
PlannerSemanticMapManager::FindCurveLaneByIdOrNull(
    mapping::ElementId id) const {
  const auto& lane = FindLaneByIdOrNull(id);
  if (!lane || lane->points().empty()) {
    return nullptr;
  }
  return lane;
}
ad_e2e::planning::LaneBoundaryConstPtr
PlannerSemanticMapManager::FindLaneBoundaryByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetLaneBoundaryById(id);
}
ad_e2e::planning::RoadBoundaryConstPtr
PlannerSemanticMapManager::FindRoadBoundaryByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetRoadBoundaryById(id);
}
ad_e2e::planning::JunctionConstPtr
PlannerSemanticMapManager::FindJunctionByIdOrNull(mapping::ElementId id) const {
  return map_ptr_->GetJunctionById(id);
}
ad_e2e::planning::StopLineConstPtr
PlannerSemanticMapManager::FindStopLineByIdOrNull(mapping::ElementId id) const {
  return map_ptr_->GetStopLineById(id);
}
ad_e2e::planning::CrosswalkConstPtr
PlannerSemanticMapManager::FindCrosswalkByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetCrosswalkById(id);
}
ad_e2e::planning::SpeedBumpConstPtr
PlannerSemanticMapManager::FindSpeedBumpByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetSpeedBumpById(id);
}
ad_e2e::planning::ClearAreaConstPtr
PlannerSemanticMapManager::FindClearAreaByIdOrNull(
    mapping::ElementId id) const {
  return map_ptr_->GetClearAreaById(id);
}
ad_e2e::planning::SectionConstPtr
PlannerSemanticMapManager::FindSectionByIdOrNull(mapping::ElementId id) const {
  return map_ptr_->GetSectionById(id);
};
std::vector<ad_e2e::planning::CrosswalkConstPtr>
PlannerSemanticMapManager::GetCrosswalksInRadius(const Vec2d& smooth_coord,
                                                 double radius) const {
  const auto crosswalk_vec = map_ptr_->FindCrosswalksInRadius(
      smooth_coord.x(), smooth_coord.y(), radius);

  std::vector<ad_e2e::planning::CrosswalkConstPtr> crosswalk_infos;
  crosswalk_infos.reserve(crosswalk_vec.size());
  for (const auto& crosswalk : crosswalk_vec) {
    const auto& crosswalk_ptr = FindCrosswalkByIdOrNull(crosswalk->id());
    if (crosswalk_ptr == nullptr) continue;
    crosswalk_infos.push_back(crosswalk_ptr);
  }
  return crosswalk_infos;
};

}  // namespace e2e_noa::planning
