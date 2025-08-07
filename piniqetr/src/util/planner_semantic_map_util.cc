#include "util/planner_semantic_map_util.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_util.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {
constexpr double kEpsilon = 1e-6;
bool IsOutgoingLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const ad_e2e::planning::Lane& source_lane, mapping::ElementId out_lane_id) {
  for (const auto& id : source_lane.next_lane_ids()) {
    if (out_lane_id == id) {
      return true;
    }
  }
  return false;
}

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s) {
  double accum_len = 0.0;
  for (const auto& seg : lane_path) {
    accum_len += seg.length();
    if (accum_len >= s) {
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                      seg.lane_id, false);
      return lane_info.right_lane_id().empty();
    }
  }
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(last_lane_info, planner_semantic_map_manager,
                                  lane_path.back().lane_id(), false);
  return last_lane_info.right_lane_id().empty();
}

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                  lane_id, false);
  return lane_info.right_lane_id().empty();
}

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s) {
  double accum_len = 0.0;
  for (const auto& seg : lane_path) {
    accum_len += seg.length();
    if (accum_len >= s) {
      SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                      seg.lane_id, false);
      return lane_info.left_lane_id().empty();
    }
  }
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(last_lane_info, planner_semantic_map_manager,
                                  lane_path.back().lane_id(), false);
  return last_lane_info.left_lane_id().empty();
}

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, planner_semantic_map_manager,
                                  lane_id, false);
  return lane_info.left_lane_id().empty();
}

bool IsLanePathBlockedByBox2d(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const Box2d& box, const mapping::LanePath& lane_path, double lat_thres) {
  ASSIGN_OR_DIE(
      const auto ff,
      BuildBruteForceFrenetFrame(
          SampleLanePathPoints(planner_semantic_map_manager, lane_path), true));
  bool reached_left = false, reached_right = false;
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    const double lat_offset = ff.XYToSL(pt).l;
    if (lat_offset >= lat_thres) reached_left = true;
    if (lat_offset <= -lat_thres) reached_right = true;

    if (reached_left && reached_right) return true;
  }
  return false;
}

std::vector<Vec2d> SampleLanePathPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path) {
  std::vector<Vec2d> sample_points;
  for (const auto& seg : lane_path) {
    if (seg.end_fraction <= seg.start_fraction + kEpsilon) continue;

    const auto& lane_info_ptr =
        planner_semantic_map_manager.FindCurveLaneByIdOrNull(seg.lane_id);
    if (lane_info_ptr == nullptr) {
      break;
    }

    auto tmp_points =
        (seg.start_fraction == 0.0 && seg.end_fraction == 1.0)
            ? lane_info_ptr->points()
            : mapping::ResampleLanePoints(*lane_info_ptr, seg.start_fraction,
                                          seg.end_fraction, nullptr);

    if (!sample_points.empty()) sample_points.pop_back();
    sample_points.insert(sample_points.end(), tmp_points.begin(),
                         tmp_points.end());
  }
  return sample_points;
}

absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePathProto& lane_path) {
  std::vector<Vec2d> sample_points;
  bool is_partial = false;
  std::string message = "";
  constexpr auto kFractionEpsilon = 1E-6;
  for (int i = 0; i < lane_path.lane_ids().size(); i++) {
    const auto lane_id = lane_path.lane_ids()[i];
    const auto& lane_info_ptr =
        planner_semantic_map_manager.FindCurveLaneByIdOrNull(
            mapping::ElementId(lane_id));
    if (lane_info_ptr == nullptr) {
      is_partial = true;
      message = absl::StrCat("Cannot find lane_id:", lane_id);
      break;
    }
    const double start_fraction = (i == 0) ? lane_path.start_fraction() : 0.0;
    const double end_fraction =
        (i == lane_path.lane_ids().size() - 1) ? lane_path.end_fraction() : 1.0;
    if (end_fraction <= start_fraction + kFractionEpsilon) continue;
    auto tmp_points =
        mapping::TruncatePoints(lane_info_ptr->points(), start_fraction,
                                end_fraction, kFractionEpsilon);
    if (!sample_points.empty()) sample_points.pop_back();
    sample_points.insert(sample_points.end(), tmp_points.begin(),
                         tmp_points.end());
  }

  return SamplePathPointsResult{.points = std::move(sample_points),
                                .is_partial = is_partial,
                                .message = std::move(message)};
}

absl::StatusOr<mapping::LanePath> ClampLanePathFromPos(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, const Vec2d& pos) {
  if (lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("Empty lane path.");
  }

  const auto points =
      SampleLanePathPoints(planner_semantic_map_manager, lane_path);
  ASSIGN_OR_RETURN(const auto ff, BuildBruteForceFrenetFrame(points, true));
  const auto sl = ff.XYToSL(pos);

  if (sl.s >= lane_path.length()) {
    return absl::NotFoundError(
        absl::StrFormat("Queried pos %s beyond lane path %s.",
                        pos.DebugString(), lane_path.DebugString()));
  }

  return lane_path.AfterArclength(sl.s);
}

absl::StatusOr<mapping::LanePoint> FindOutgoingLanePointWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm, mapping::ElementId id) {
  SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, id);
  if (lane_info.next_lane_ids().empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Can not find outgoing lane. Lane id:", lane_info.id(), "."));
  }

  if (lane_info.next_lane_ids().size() == 1) {
    return mapping::LanePoint(lane_info.next_lane_ids().front(), 0.0);
  }

  constexpr double kSampleLen = 4.0;
  const Vec2d origin_pt = lane_info.points().back();
  const Vec2d prev_pt = lane_info.LerpPointFromFraction(
      std::max(0.0, 1.0 - kSampleLen / lane_info.curve_length()));
  const Vec2d heading = (origin_pt - prev_pt).normalized();

  struct OutgoingLaneData {
    mapping::ElementId lane_id;
    double project_value;
    bool is_virtual;
  };

  std::vector<OutgoingLaneData> outgoing_lane_data;
  outgoing_lane_data.reserve(lane_info.next_lane_ids().size());

  for (int i = 0; i < lane_info.next_lane_ids().size(); ++i) {
    const auto temp_id = lane_info.next_lane_ids()[i];
    SMM_ASSIGN_LANE_OR_CONTINUE(temp_lane_info, psmm, temp_id);
    const Vec2d next_pt = temp_lane_info.LerpPointFromFraction(
        std::min(1.0, kSampleLen / temp_lane_info.curve_length()));
    const Vec2d tmp_heading = (next_pt - origin_pt).normalized();
    const double proj = heading.Dot(tmp_heading);
    outgoing_lane_data.push_back({temp_id, proj, temp_lane_info.IsVirtual()});
  }

  std::stable_sort(outgoing_lane_data.begin(), outgoing_lane_data.end(),
                   [](const auto& lhs, const auto& rls) {
                     return lhs.project_value > rls.project_value;
                   });

  auto opt_outgoing_id = outgoing_lane_data.front().lane_id;
  for (const auto& mgr : outgoing_lane_data) {
    if (mgr.is_virtual == false) {
      opt_outgoing_id = mgr.lane_id;
      break;
    }
  }

  return mapping::LanePoint(opt_outgoing_id, 0.0);
}
}  // namespace e2e_noa::planning
