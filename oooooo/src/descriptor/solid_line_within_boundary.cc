#include "descriptor/solid_line_within_boundary.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "absl/types/span.h"
#include "math/frenet_common.h"
#include "math/geometry/util.h"
#include "math/vec.h"
#include "plan/planner_defs.h"
#include "util/status_builder.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {

namespace {

void ConvertBoundaryToProto(
    const PlanPassage& plan_passage,
    absl::Span<const std::pair<int, StationBoundary>> solid_lines,
    int start_idx, int end_idx,
    std::vector<ConstraintProto::AvoidLineProto>* constraints) {
  ConstraintProto::AvoidLineProto proto;
  auto* xy_points = proto.mutable_xy_points();
  auto* sl_points = proto.mutable_sl_points();
  xy_points->Reserve(end_idx - start_idx + 1);
  sl_points->Reserve(end_idx - start_idx + 1);
  for (int j = start_idx; j <= end_idx; ++j) {
    const auto& station =
        plan_passage.station(StationIndex(solid_lines[j].first));
    station.lat_point(solid_lines[j].second.lat_offset)
        .ToProto(xy_points->Add());
    auto* sl_point = sl_points->Add();
    sl_point->set_s(station.accumulated_s());
    sl_point->set_l(solid_lines[j].second.lat_offset);
  }
  proto.set_id("Solid Line");
  proto.mutable_source()->mutable_solid_line_within_boundary()->set_id(
      "Solid Line");
  constraints->emplace_back(std::move(proto));
}

void AddBoundariesToConstraints(
    const PlanPassage& plan_passage, const PathSlBoundary& path_boundary,
    absl::Span<const std::pair<int, StationBoundary>> solid_lines,
    std::vector<ConstraintProto::AvoidLineProto>* constraints) {
  const int solid_line_size = solid_lines.size();
  if (solid_line_size < 2) return;

  int start_idx = -1;
  for (int i = 0; i < solid_line_size; ++i) {
    const auto station_idx = solid_lines[i].first;
    const bool is_within_boundary =
        solid_lines[i].second.lat_offset <
            path_boundary.left_l_vector()[station_idx] &&
        solid_lines[i].second.lat_offset >
            path_boundary.right_l_vector()[station_idx];

    if (start_idx == -1 && is_within_boundary) {
      start_idx = i;
    } else if (start_idx != -1 && !is_within_boundary) {
      const int end_idx = std::min(i + 1, solid_line_size - 1);
      start_idx = std::max(start_idx - 1, 0);
      if (end_idx > start_idx) {
        ConvertBoundaryToProto(plan_passage, solid_lines, start_idx, end_idx,
                               constraints);
      }
      start_idx = -1;
    }
  }

  if (start_idx != -1) {
    const int end_idx = solid_line_size - 1;
    start_idx = std::max(start_idx - 1, 0);
    if (end_idx > start_idx) {
      ConvertBoundaryToProto(plan_passage, solid_lines, start_idx, end_idx,
                             constraints);
    }
  }
}

}  // namespace

absl::StatusOr<std::vector<ConstraintProto::AvoidLineProto>>
BuildSolidLineWithinBoundaryConstraint(
    const PlanPassage& plan_passage, const PathSlBoundary& path_boundary,
    const ApolloTrajectoryPointProto& plan_start_point) {
  std::vector<ConstraintProto::AvoidLineProto> solid_line_constraints;
  std::vector<std::vector<std::pair<int, StationBoundary>>> active_intervals;

  ASSIGN_OR_RETURN(const auto cur_sl,
                   plan_passage.QueryFrenetCoordinateAt(
                       Extract2dVectorFromApolloProto(plan_start_point)),
                   _ << "BuildSolidLineWithinBoundaryConstraint: Fail to "
                        "project ego pos on plan passage.");

  for (int i = 0; i < path_boundary.size(); ++i) {
    const auto& station = plan_passage.station(StationIndex(i));
    std::vector<StationBoundary> new_boundaries;
    for (const auto& boundary : station.boundaries()) {
      if (!boundary.IsSolid(cur_sl.l) ||
          boundary.type == StationBoundaryType::VIRTUAL_CURB ||
          boundary.type == StationBoundaryType::CURB) {
        continue;
      }

      double match_dist = kDefaultHalfLaneWidth;
      int match_idx = -1;
      for (int j = 0; j < active_intervals.size(); ++j) {
        const auto& interval_back = active_intervals[j].back();
        if (interval_back.first + 1 == i &&
            interval_back.second.type == boundary.type) {
          const double lat_dist =
              std::abs(interval_back.second.lat_offset - boundary.lat_offset);
          if (lat_dist < match_dist) {
            match_dist = lat_dist;
            match_idx = j;
          }
        }
      }
      if (match_idx == -1) {
        new_boundaries.push_back(boundary);
      } else {
        active_intervals[match_idx].emplace_back(i, boundary);
      }
    }
    for (auto it = active_intervals.begin(); it != active_intervals.end();) {
      if (it->back().first != i) {
        AddBoundariesToConstraints(plan_passage, path_boundary, *it,
                                   &solid_line_constraints);
        it = active_intervals.erase(it);
      } else {
        ++it;
      }
    }
    for (auto& new_boundary : new_boundaries) {
      active_intervals.emplace_back().push_back({i, new_boundary});
    }
  }

  for (const auto& interval : active_intervals) {
    AddBoundariesToConstraints(plan_passage, path_boundary, interval,
                               &solid_line_constraints);
  }

  return solid_line_constraints;
}

}  // namespace e2e_noa::planning
