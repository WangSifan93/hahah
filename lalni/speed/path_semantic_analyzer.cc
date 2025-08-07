#include "speed/path_semantic_analyzer.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_set>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "async/parallel_for.h"
#include "base/macros.h"
#include "container/strong_int.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_frame.h"
#include "math/util.h"
#include "plan/planner_defs.h"
#include "trajectory_point.pb.h"
#include "util/lane_point_util.h"
#include "util/map_util.h"
#include "util/path_util.h"
#include "util/spatial_search_util.h"

namespace e2e_noa {
namespace planning {

namespace {

constexpr double kMinDistSqrThreshold = Sqr(0.5 * kPathSampleInterval);
constexpr double kEps = 1e-3;

inline LaneSemantic QueryLaneSemantic(const ad_e2e::planning::Lane& lane_info) {
  if (!lane_info.junction_id().empty()) {
    switch (lane_info.turn_type()) {
      case ad_e2e::planning::TurnType::NO_TURN:
        return LaneSemantic::INTERSECTION_STRAIGHT;
      case ad_e2e::planning::TurnType::LEFT_TURN:
        return LaneSemantic::INTERSECTION_LEFT_TURN;
      case ad_e2e::planning::TurnType::RIGHT_TURN:
        return LaneSemantic::INTERSECTION_RIGHT_TURN;
      case ad_e2e::planning::TurnType::U_TURN:
        return LaneSemantic::INTERSECTION_UTURN;
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  } else {
    switch (lane_info.turn_type()) {
      case ad_e2e::planning::TurnType::NO_TURN:
        return LaneSemantic::ROAD;
      case ad_e2e::planning::TurnType::LEFT_TURN:
      case ad_e2e::planning::TurnType::RIGHT_TURN:
      case ad_e2e::planning::TurnType::U_TURN:
        return LaneSemantic::ROAD;
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  }
}

void AnalyzePrimaryPathSemantic(
    const PathPoint& path_point,
    const std::vector<RouteLaneInfo>& route_lanes_info,
    const PlannerSemanticMapManager& psmm, PathPointSemantic* path_semantic) {
  const Vec2d path_point_xy = ToVec2d(path_point);
  double min_dist_sqr = std::numeric_limits<double>::infinity();
  mapping::LanePoint closest_lane_point(mapping::kInvalidElementId, 0.0);
  Vec2d closest_lane_point_pos;
  for (const auto& route_lane_info : route_lanes_info) {
    if (route_lane_info.lane == nullptr) {
      continue;
    }
    Vec2d closest_point;
    const auto lane_point_opt =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm, path_point_xy, route_lane_info.lane->id(), path_point.theta(),
            0.0, &closest_point, route_lane_info.start_fraction,
            route_lane_info.end_fraction);
    if (!lane_point_opt.has_value()) continue;
    const double dist_sqr = path_point_xy.DistanceSquareTo(closest_point);
    if (dist_sqr < min_dist_sqr) {
      min_dist_sqr = dist_sqr;
      closest_lane_point = *lane_point_opt;
      closest_lane_point_pos = closest_point;
    }

    if (min_dist_sqr < kMinDistSqrThreshold) {
      break;
    }
  }
  if (!closest_lane_point.Valid()) {
    return;
  }
  path_semantic->closest_lane_point = closest_lane_point;
  path_semantic->closest_lane_point_pos = closest_lane_point_pos;
  path_semantic->deviation_distance = std::sqrt(min_dist_sqr);
  const auto& lane_info_ptr =
      psmm.FindCurveLaneByIdOrNull(closest_lane_point.lane_id());
  if (UNLIKELY(lane_info_ptr == nullptr)) {
    LOG(ERROR) << "Cannot find semantic id: " << closest_lane_point.lane_id();
    return;
  }
  path_semantic->lane_semantic = QueryLaneSemantic(*lane_info_ptr);
  path_semantic->lane_info = lane_info_ptr;
  return;
}

void AnalyzeSecondaryPathSemantic(
    const PathPoint& path_point,
    const std::vector<RouteLaneInfo>& route_lanes_info,
    const PlannerSemanticMapManager& psmm,
    const PathPointSemantic& prev_primary_path_semantic,
    const PathPointSemantic& next_primary_path_semantic, double ratio,
    PathPointSemantic* path_semantic) {
  if (!prev_primary_path_semantic.closest_lane_point.Valid() ||
      !next_primary_path_semantic.closest_lane_point.Valid()) {
    AnalyzePrimaryPathSemantic(path_point, route_lanes_info, psmm,
                               path_semantic);
    return;
  }
  if (prev_primary_path_semantic.closest_lane_point.lane_id() !=
      next_primary_path_semantic.closest_lane_point.lane_id()) {
    AnalyzePrimaryPathSemantic(path_point, route_lanes_info, psmm,
                               path_semantic);
    return;
  }
  const mapping::LanePoint closest_lane_point(
      prev_primary_path_semantic.closest_lane_point.lane_id(),
      Lerp(prev_primary_path_semantic.closest_lane_point.fraction(),
           next_primary_path_semantic.closest_lane_point.fraction(), ratio));
  path_semantic->deviation_distance =
      Lerp(prev_primary_path_semantic.deviation_distance,
           next_primary_path_semantic.deviation_distance, ratio);
  const auto closest_lane_point_pos =
      ComputeLanePointPos(psmm, closest_lane_point);
  path_semantic->closest_lane_point = closest_lane_point;
  path_semantic->closest_lane_point_pos = closest_lane_point_pos;
  path_semantic->lane_semantic = prev_primary_path_semantic.lane_semantic;
  path_semantic->lane_info = prev_primary_path_semantic.lane_info;
  return;
}

}  // namespace

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    const DiscretizedPath& path, int max_analyze_path_index,
    const PlannerSemanticMapManager& psmm,
    const DrivingMapTopo* driving_map_topo, WorkerThreadManager* thread_pool) {
  CHECK(!path.empty());
  CHECK_LT(max_analyze_path_index, path.size());

  std::atomic_int path_semantic_size = max_analyze_path_index + 1;
  std::vector<PathPointSemantic> path_semantics(path_semantic_size);
  if (!psmm.map_ptr()) {
    return absl::NotFoundError("map_ptr is nullptr!");
  } else if (!psmm.map_ptr()->route()) {
    return absl::NotFoundError("route is nullptr!");
  }
  const auto& route_info = psmm.map_ptr()->route()->GetRouteInfo();
  const auto& navi_start = route_info.navi_start;
  std::vector<RouteLaneInfo> lanes;
  for (const auto& section : route_info.sections) {
    for (const auto& lane_id : section.lane_ids) {
      const auto& lane = psmm.FindCurveLaneByIdOrNull(lane_id);
      if (lane != nullptr) {
        RouteLaneInfo route_lane_info;
        route_lane_info.lane = lane;
        double start_fraction = 0.0;
        if (section.id == navi_start.section_id) {
          const auto ff = BuildBruteForceFrenetFrame(lane->points(), false);
          if (ff.ok()) {
            FrenetCoordinate sl;
            Vec2d normal;
            std::pair<int, int> index_pair;
            double alpha;
            ff.value().XYToSL({path.front().x(), path.front().y()}, &sl,
                              &normal, &index_pair, &alpha);
            start_fraction = std::clamp(
                sl.s / std::fmax(lane->curve_length(), 1e-2), 0.0, 1.0 - 1e-2);
          }
        }
        route_lane_info.start_fraction = start_fraction;
        route_lane_info.end_fraction = 1.0;
        lanes.push_back(route_lane_info);
      }
    }
  }

  constexpr double kPrimaryPathSampleInterval = 1.0;
  std::vector<int> primary_indices;
  std::unordered_set<int> primary_index_set;
  primary_indices.reserve(CeilToInteger(path[max_analyze_path_index].s() /
                                        kPrimaryPathSampleInterval) +
                          1);
  primary_index_set.reserve(primary_indices.capacity());
  double last_primary_s = 0.0;
  for (int i = 0; i < path_semantic_size; ++i) {
    if (i == 0 || i + 1 == path_semantic_size ||
        path[i].s() - last_primary_s > kPrimaryPathSampleInterval - kEps) {
      last_primary_s = path[i].s();
      primary_indices.push_back(i);
      primary_index_set.insert(i);
    }
  }

  ParallelFor(0, primary_indices.size(), thread_pool, [&](int i) {
    const int idx = primary_indices[i];
    int current_size = path_semantic_size.load(std::memory_order_relaxed);
    if (idx > current_size) return;
    AnalyzePrimaryPathSemantic(path[idx], lanes, psmm, &path_semantics[idx]);
    if (!path_semantics[idx].closest_lane_point.Valid()) {
      do {
        if (idx > current_size) break;
      } while (!path_semantic_size.compare_exchange_weak(
          current_size, idx, std::memory_order_relaxed));
    }
    return;
  });

  ParallelFor(0, path_semantic_size, thread_pool, [&](int i) {
    int current_size = path_semantic_size.load(std::memory_order_relaxed);
    if (i > current_size) return;
    if (ContainsKey(primary_index_set, i)) {
      return;
    }
    int idx = FloorToInt(path[i].s() / kPrimaryPathSampleInterval);
    while (idx + 1 > primary_indices.size() || primary_indices[idx] > i) {
      --idx;
    }
    while (idx + 1 < primary_indices.size() && primary_indices[idx + 1] < i) {
      ++idx;
    }
    const int prev_primary_idx = primary_indices[idx];
    const int next_primary_idx = primary_indices[idx + 1];
    const double ratio =
        (path[i].s() - path[prev_primary_idx].s()) /
        (path[next_primary_idx].s() - path[prev_primary_idx].s());
    AnalyzeSecondaryPathSemantic(
        path[i], lanes, psmm, path_semantics[prev_primary_idx],
        path_semantics[next_primary_idx], ratio, &path_semantics[i]);
    if (!path_semantics[i].closest_lane_point.Valid()) {
      do {
        if (i > current_size) break;
      } while (!path_semantic_size.compare_exchange_weak(
          current_size, i, std::memory_order_relaxed));
    }
    return;
  });

  if (path_semantic_size == 0) {
    return absl::NotFoundError("Failed to analyze path semantics!");
  }
  path_semantics.erase(path_semantics.begin() + path_semantic_size,
                       path_semantics.end());

  constexpr double kClosestLanePointSqrDistThres = Sqr(2.8);
  std::vector<int> curr_lane_path_id_history = {0};
  path_semantics[0].lane_path_id_history = curr_lane_path_id_history;
  for (int i = 1; i < path_semantic_size; ++i) {
    if (path_semantics[i].closest_lane_point.lane_id() !=
            path_semantics[i - 1].closest_lane_point.lane_id() &&
        path_semantics[i].closest_lane_point_pos.DistanceSquareTo(
            path_semantics[i - 1].closest_lane_point_pos) >
            kClosestLanePointSqrDistThres) {
      const auto closest_lane_point_tangent =
          ComputeLanePointTangent(psmm, path_semantics[i].closest_lane_point);

      if (closest_lane_point_tangent.CrossProd(
              path_semantics[i].closest_lane_point_pos -
              path_semantics[i - 1].closest_lane_point_pos) > 0.0) {
        curr_lane_path_id_history.push_back(curr_lane_path_id_history.back() +
                                            1);
      } else {
        curr_lane_path_id_history.push_back(curr_lane_path_id_history.back() -
                                            1);
      }
    }
    path_semantics[i].lane_path_id_history = curr_lane_path_id_history;
  }

  return path_semantics;
}

}  // namespace planning
}  // namespace e2e_noa
