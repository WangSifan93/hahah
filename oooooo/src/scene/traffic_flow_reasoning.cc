#include "scene/traffic_flow_reasoning.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "async/parallel_for.h"
#include "box2d.pb.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/util.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "plan/planner_flags.h"
#include "scene_understanding.pb.h"
#include "traffic_light_info.pb.h"
#include "trajectory.pb.h"
#include "util/lane_path_util.h"
#include "util/map_util.h"
#include "util/perception_util.h"
#include "util/planner_semantic_map_util.h"
#include "util/spatial_search_util.h"

namespace e2e_noa::planning {
namespace {
constexpr double kIgnoreUnknownStaticDist = 100.0;
constexpr double kMaxLonDistBetweenObjects = 10.0;
constexpr double kTrafficLightDistMinThreshold = 10.0;
double kTrafficLightDistMaxThreshold = 80.0;
constexpr double kDefaultStopTimeThreshold = 10.0;
constexpr double kSpecialZoneDistThreshold = 30.0;
constexpr double kStalledProbThreshold = 0.6;
constexpr double kDefaultRedTrafficLightMultiplier = 1.5;
constexpr double kEpsilon = 0.1;
constexpr double kMaxTrafficWaitingDistance = 300.0;

constexpr double kMinTrafficWaitingDistance = 20.0;

constexpr double kMinMovingSpeedThes = 0.6;

const PiecewiseLinearFunction kCalculateFactorAccordHeadingDiffPlf(
    std::vector<double>{M_PI / 6.0, M_PI / 2.0}, std::vector<double>{1.0, 0.3});

using LanePath = mapping::LanePath;
using ObjectsOnLane =
    std::vector<std::pair<double, const ObjectPredictionProto*>>;

struct AnalyzeOnLaneOutput {
  TrafficWaitingQueueProto traffic_waiting_queue;
  std::vector<ObjectAnnotationProto> object_annotations;
  std::optional<double> distance_to_roadblock = std::nullopt;
};

double HalfLength(const Box2dProto& box_proto) {
  return box_proto.length() * 0.5;
}

double GetObjectFrontS(double object_s, const ObjectProto& object) {
  return object_s + HalfLength(object.bounding_box());
}

double GetObjectBackS(double object_s, const ObjectProto& object) {
  return object_s - HalfLength(object.bounding_box());
}

bool IsStationary(const ObjectPredictionProto& object_pred) {
  const auto object = object_pred.perception_object();
  double speed = std::sqrt(object.vel().x() * object.vel().x() +
                           object.vel().y() * object.vel().y());
  return speed < kMinMovingSpeedThes;
}

std::vector<double> CollectTrafficLightStopLine(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path) {
  std::vector<double> tl_stop_lines;
  for (auto it = lane_path.begin(); it != lane_path.end(); ++it) {
    const auto& lane_info_ptr = psmm.FindCurveLaneByIdOrNull((*it).lane_id);
    if (lane_info_ptr == nullptr) continue;

    if (!lane_info_ptr->next_lane_ids().empty()) {
      for (const auto& out_lane_id : lane_info_ptr->next_lane_ids()) {
        SMM_ASSIGN_LANE_OR_CONTINUE(out_lane_info, psmm, out_lane_id);
        if (out_lane_info.IsValid() && out_lane_info.IsVirtual()) {
          tl_stop_lines.push_back((*it).end_s);
          break;
        }
      }
    }
  }

  std::stable_sort(tl_stop_lines.begin(), tl_stop_lines.end());
  auto iter = std::unique(
      tl_stop_lines.begin(), tl_stop_lines.end(),
      [](double lsh, double rsh) { return std::fabs(lsh - rsh) < kEpsilon; });
  tl_stop_lines.erase(iter, tl_stop_lines.end());
  return tl_stop_lines;
}

std::vector<std::pair<double, double>> CollectSpecialZonesInSegment(
    const std::vector<std::pair<mapping::ElementId, Vec2d>>& zones_info,
    const LanePath::LaneSegment& seg, double lane_length) {
  std::vector<std::pair<double, double>> zones;
  zones.reserve(zones_info.size());
  for (const auto& [_, frac] : zones_info) {
    if (seg.start_fraction <= frac[1] && frac[1] <= seg.end_fraction) {
      const double start_s =
          seg.start_s + lane_length * (frac[0] - seg.start_fraction);
      const double end_s =
          seg.start_s + lane_length * (frac[1] - seg.start_fraction);
      zones.push_back({start_s, end_s});
    }
  }
  return zones;
}

std::vector<double> CollectSpecialZonesStartSOnLanePath(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path) {
  std::vector<double> arc_lens;
  auto collect_zone_start_s =
      [&arc_lens](const std::vector<std::pair<double, double>>& zones,
                  double* prev_end_s) {
        for (const auto& [start_s, end_s] : zones) {
          if (std::fabs(start_s - *prev_end_s) > kEpsilon) {
            arc_lens.push_back(start_s);
          }
          *prev_end_s = end_s;
        }
      };

  double prev_crosswalk_zone_end_s = -std::numeric_limits<double>::infinity();
  double prev_intersection_zone_end_s =
      -std::numeric_limits<double>::infinity();

  for (const auto& seg : lane_path) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, seg.lane_id);

    std::vector<std::pair<mapping::ElementId, Vec2d>> filtered_intersections;
    std::vector<std::string> junction_ids = {lane_info.junction_id()};
    for (const auto& intersection : junction_ids) {
      const auto intersection_info_ptr =
          psmm.FindJunctionByIdOrNull(intersection);
      if (intersection_info_ptr == nullptr) continue;

      const bool traffic_light_controlled_intersection =
          intersection_info_ptr->traffic_light_controlled();
      if (!FLAGS_planner_enable_un_tl_controlled_intersection_reasoning &&
          !traffic_light_controlled_intersection) {
        continue;
      }
      filtered_intersections.push_back(
          std::make_pair(intersection, Vec2d(0.0, 1.0)));
    }
    collect_zone_start_s(
        CollectSpecialZonesInSegment(filtered_intersections, seg,
                                     lane_info.curve_length()),
        &prev_intersection_zone_end_s);
  }
  std::stable_sort(arc_lens.begin(), arc_lens.end());
  return arc_lens;
}

std::optional<double> CalcSOnLanePathByPose(
    const PlannerSemanticMapManager& psmm, const LanePath& lane_path,
    const Vec2d& ego_pos) {
  double arc_len;
  if (IsPointOnLanePathAtLevel(psmm, ego_pos, lane_path, &arc_len)) {
    return arc_len;
  }

  return std::nullopt;
}

bool IsObjectOnLanePathByPose(const PlannerSemanticMapManager& psmm,
                              const LanePath& lane_path,
                              const ObjectProto& object, double* arc_len) {
  if (lane_path.IsEmpty()) return false;

  const bool is_on_lane = IsPointOnLanePathAtLevel(
      psmm, Vec2dFromProto(object.pos()), lane_path, arc_len);

  if (object.type() == ObjectType::OT_UNKNOWN_STATIC &&
      *arc_len >= kIgnoreUnknownStaticDist) {
    return false;
  }

  return is_on_lane;
}

bool IsObjectOnLanePathByBoundingBox(const PlannerSemanticMapManager& psmm,
                                     const LanePath& lane_path,
                                     const ObjectProto& object,
                                     double* arc_len) {
  if (lane_path.IsEmpty()) return false;

  const auto closest_lane_point_or =
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm, Vec2dFromProto(object.pos()), lane_path, object.yaw());
  if (!closest_lane_point_or.ok()) return false;

  *arc_len =
      lane_path.FirstOccurrenceOfLanePointToArclength(*closest_lane_point_or);

  if (object.type() == ObjectType::OT_UNKNOWN_STATIC &&
      *arc_len >= kIgnoreUnknownStaticDist) {
    return false;
  }

  const auto bounding_box =
      ComputeObjectContour(object).BoundingBoxWithHeading(object.yaw());
  for (const auto& pt : bounding_box.GetCornersCounterClockwise()) {
    double s;
    if (IsPointOnLanePathAtLevel(psmm, pt, lane_path, &s)) {
      return true;
    }
  }

  return false;
}

bool IsTrafficWaiting(absl::Span<const double> tls_s,
                      double traffic_chain_front_object_front_s,
                      double traffic_chain_front_object_back_s,
                      double object_back_s, const std::string obj_id) {
  const auto it_tl_front =
      std::upper_bound(tls_s.begin(), tls_s.end(), object_back_s);

  if (it_tl_front == tls_s.end()) return false;

  const double dist_to_front_edge =
      *it_tl_front - traffic_chain_front_object_front_s;
  const double dist_to_back_edge =
      *it_tl_front - traffic_chain_front_object_back_s;
  const std::string dist_to_tls =
      "it_tl_front: " + std::to_string(*it_tl_front) +
      " dist_to_front_edge: " + std::to_string(dist_to_front_edge) +
      " dist_to_back_edge: " + std::to_string(dist_to_back_edge) +
      " obj_id: " + obj_id;

  return (0.0 < dist_to_front_edge &&
          dist_to_front_edge < kTrafficLightDistMinThreshold) ||
         (dist_to_front_edge * dist_to_back_edge < 0.0);
}

double CalculateFactorForSpecialZone(absl::Span<const double> zone_s_vec,
                                     absl::Span<const double> object_s_vec,
                                     double distance_threshold,
                                     double max_factor,
                                     const std::string& obj_id) {
  double factor = 1.0;
  for (const double s : object_s_vec) {
    const auto it_prev_special_zone_s =
        std::upper_bound(zone_s_vec.begin(), zone_s_vec.end(), s);
    if (it_prev_special_zone_s == zone_s_vec.end()) continue;
    const double distance = *it_prev_special_zone_s - s;
    const std::string st_l_distance =
        "obj_id: " + obj_id + " distance: " + std::to_string(distance) +
        " s: " + std::to_string(s) +
        " it_prev_special_zone_s: " + std::to_string(*it_prev_special_zone_s);

    if (distance > distance_threshold) continue;

    factor = std::max(factor, 1.0 + (max_factor - 1.0) *
                                        (1.0 - distance / distance_threshold));
  }
  return factor;
}

double CalculateFactorForMovingNeighbor(
    const ObjectStopTimeProto& stop_time_info) {
  const double factor =
      std::max(0.0, stop_time_info.previous_stop_time_duration() -
                        stop_time_info.last_move_time_duration()) /
      stop_time_info.last_move_time_duration();
  return std::min(1.0, factor);
}

std::vector<const ObjectPredictionProto*> CollectContinuousObjectsAheadOfS(
    const ObjectsOnLane& objects, double s) {
  std::vector<const ObjectPredictionProto*> objects_ahead;
  if (objects.empty() ||
      GetObjectFrontS(objects.back().first,
                      objects.back().second->perception_object()) < s) {
    return objects_ahead;
  }

  auto start_it = objects.rbegin(), end_it = objects.rbegin() + 1;
  for (; end_it != objects.rend(); ++end_it) {
    const double object_front_s =
        GetObjectFrontS(end_it->first, end_it->second->perception_object());
    if (object_front_s < s) break;

    const double prev_object_back_s = GetObjectBackS(
        (end_it - 1)->first, (end_it - 1)->second->perception_object());

    if (prev_object_back_s - object_front_s > kMaxLonDistBetweenObjects) {
      start_it = end_it;
    }
  }

  const double end_back_s = GetObjectBackS(
      (end_it - 1)->first, (end_it - 1)->second->perception_object());
  if (end_back_s - s > kMaxLonDistBetweenObjects) {
    return objects_ahead;
  }

  objects_ahead.reserve(end_it - start_it);
  for (auto it = start_it; it != end_it; ++it) {
    objects_ahead.push_back(it->second);
  }
  return objects_ahead;
}

bool CheckWhetherLanePathControlledByRedTrafficLight(
    const ad_e2e::planning::TrafficLightStatusMap& tl_info_map,
    const mapping::LanePath& lane_path) {
  for (const auto id : lane_path.lane_ids()) {
    const auto iter = tl_info_map.find(id);
    if (iter == tl_info_map.end()) continue;
    const auto& tl_info = iter->second;

    if (tl_info.junction_id.has_value() &&
        tl_info.light_status == ad_e2e::planning::LightStatus::RED_LIGHT) {
      return true;
    }
  }
  return false;
}

double CalculateFactorAccordHeadingDiff(double object_theta,
                                        double lane_theta) {
  return kCalculateFactorAccordHeadingDiffPlf(
      std::fabs(NormalizeAngle(object_theta - lane_theta)));
}

double CalculateFactorForTrafficLightControlledLeftmostLane(
    absl::Span<const double> tls_s) {
  return tls_s.empty() || tls_s.front() > kMaxTrafficWaitingDistance ? 1.0
                                                                     : 100.0;
}

bool IsIsolatedLaneAtS(const PlannerSemanticMapManager& psmm,
                       const LanePath& lane_path, double s) {
  const auto& lane_ids = lane_path.lane_ids();
  const auto lane_idx = lane_path.ArclengthToLaneIndex(s);
  const auto lane_info = psmm.FindCurveLaneByIdOrNull(lane_ids[lane_idx]);
  if (lane_info == nullptr) return false;
  return lane_info->left_lane_id().empty() &&
         lane_info->right_lane_id().empty();
}

bool IsFarAwayFromTlAtS(absl::Span<const double> tls_s, double s,
                        double max_dist_threshold) {
  if (tls_s.empty()) return true;
  const auto iter_tl_front = std::upper_bound(tls_s.begin(), tls_s.end(), s);
  if (iter_tl_front == tls_s.end()) return true;
  return *iter_tl_front - s > max_dist_threshold;
}

AnalyzeOnLaneOutput AnalyzeOnLane(
    const PlannerSemanticMapManager& psmm, const ObjectsOnLane& objects_on_lane,
    const LanePath& lane_path,
    const ad_e2e::planning::TrafficLightStatusMap& tl_info_map,
    const Vec2d& ego_pos, absl::Span<const ObjectsOnLane* const> neighbors,
    ObjectHistoryController& obj_manager,
    const BruteForceFrenetCoordinate& ff) {
  const auto extend_lane_path = ForwardExtendLanePathWithoutFork(
      psmm, lane_path, kMaxTrafficWaitingDistance - lane_path.length());
  auto tls_s = CollectTrafficLightStopLine(psmm, extend_lane_path);
  const auto zones_s = CollectSpecialZonesStartSOnLanePath(psmm, lane_path);
  const auto ego_s_on_lane_path =
      CalcSOnLanePathByPose(psmm, lane_path, ego_pos);
  double dis_tls = 0.0;
  const auto& map = psmm.map_ptr();
  const auto& ehp_v2_info = map->v2_info();
  double dist_to_junction_v2 = DBL_MAX;
  if (ehp_v2_info.has_navigation) {
    for (const auto& turn_info : ehp_v2_info.turn_info) {
      if (!turn_info.is_valid) break;
      if ((turn_info.detail_turn_type ==
               ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_LEFT ||
           turn_info.detail_turn_type ==
               ad_e2e::planning::V2TurnInfo::V2DetailTurnType::TURN_RIGHT ||
           turn_info.detail_turn_type ==
               ad_e2e::planning::V2TurnInfo::V2DetailTurnType::UTURN ||
           turn_info.detail_turn_type ==
               ad_e2e::planning::V2TurnInfo::V2DetailTurnType::CONTINUE ||
           turn_info.detail_turn_type ==
               ad_e2e::planning::V2TurnInfo::V2DetailTurnType::
                   TURN_RIGHT_ONLY)) {
        dist_to_junction_v2 = turn_info.dist;
        break;
      }
    }
  }
  double ego_s = 0.0;
  std::string debug_stop_s = "ego_s: ";
  if (ego_s_on_lane_path.has_value()) {
    ego_s = ego_s_on_lane_path.value();
    debug_stop_s =
        debug_stop_s + std::to_string(ego_s_on_lane_path.value()).substr(0, 4);
  }
  debug_stop_s = debug_stop_s + " stop_s: ";

  if (!tls_s.empty()) {
    dis_tls = tls_s.front();
  } else if (dist_to_junction_v2 != DBL_MAX) {
    dis_tls = dist_to_junction_v2;
    tls_s.emplace_back(dist_to_junction_v2 + ego_s);
  }
  const std::string tls_s_size = "size: " + std::to_string(tls_s.size()) +
                                 " s_value: " + std::to_string(dis_tls);

  for (auto& tls : tls_s) {
    debug_stop_s = debug_stop_s + " " + std::to_string(tls).substr(0, 4);
  }
  for (auto& zone : zones_s) {
    debug_stop_s = debug_stop_s + " " + std::to_string(zone).substr(0, 4);
  }

  absl::flat_hash_map<std::string, double> stall_analysis;

  std::vector<std::pair<double, std::string>> traffic_waiting_analysis;

  std::optional<double> distance_to_roadblock = std::nullopt;

  auto prev_cont_it = objects_on_lane.rbegin();
  bool has_stalled_ahead = false;

  for (auto it = objects_on_lane.rbegin(); it != objects_on_lane.rend(); ++it) {
    std::string stop_time_threshold_debug;
    const auto [object_s, object_pred_ptr] = *it;
    const auto& object = object_pred_ptr->perception_object();
    const double object_half_length = HalfLength(object.bounding_box());
    const double object_front_s = object_s + object_half_length;
    const double object_back_s = object_s - object_half_length;
    const auto& stop_time_info = object_pred_ptr->stop_time();

    if (it != objects_on_lane.rbegin()) {
      const auto it_prev = it - 1;
      const double prev_back_s =
          GetObjectBackS(it_prev->first, it_prev->second->perception_object());
      if (prev_back_s - object_front_s > kMaxLonDistBetweenObjects) {
        prev_cont_it = it;
        has_stalled_ahead = false;
      }
    }
    const auto& object_id = object.id();

    double speed = std::sqrt(object.vel().x() * object.vel().x() +
                             object.vel().y() * object.vel().y());
    std::string stall_speed_debug =
        "id: " + object_id + " speed: " + std::to_string(speed);

    if (!IsStationary(*object_pred_ptr)) continue;

    auto obs_histroy_info = obj_manager.GetObjLatestFrame(object_id);

    if (object.type() == ObjectType::OT_BARRIER ||
        object.type() == ObjectType::OT_CONE ||
        object.type() == ObjectType::OT_WARNING_TRIANGLE ||
        object.type() == ObjectType::OT_UNKNOWN_STATIC ||
        (obs_histroy_info && obs_histroy_info->is_stalled)) {
      stall_analysis[object_id] = 1.0;

      if (ego_s_on_lane_path.has_value() && object_s > *ego_s_on_lane_path) {
        distance_to_roadblock = object_s - *ego_s_on_lane_path;
      }
      continue;
    }

    const bool is_leftmost_lane = IsLeftMostLane(psmm, lane_path, object_s);
    const bool is_rightmost_lane = IsRightMostLane(psmm, lane_path, object_s);
    if (is_rightmost_lane && !is_leftmost_lane)
      kTrafficLightDistMaxThreshold = 60.0;
    double stop_time_threshold =
        (is_leftmost_lane && is_rightmost_lane)
            ? 3.0 * kDefaultStopTimeThreshold
        : is_leftmost_lane  ? 7.0 * kDefaultStopTimeThreshold
        : is_rightmost_lane ? 0.6 * kDefaultStopTimeThreshold
                            : 3.0 * kDefaultStopTimeThreshold;
    stop_time_threshold_debug = stop_time_threshold_debug + " rel_lane: " +
                                std::to_string(stop_time_threshold);
    const double traffic_chain_front_object_front_s = GetObjectFrontS(
        prev_cont_it->first, prev_cont_it->second->perception_object());
    const double traffic_chain_front_object_back_s = GetObjectBackS(
        prev_cont_it->first, prev_cont_it->second->perception_object());
    bool traffic_wait = IsTrafficWaiting(
        tls_s, traffic_chain_front_object_front_s,
        traffic_chain_front_object_back_s, object_back_s, object_id);

    std::string traffic_wait_debug = object_id + " " +
                                     std::to_string(traffic_wait) + " " +
                                     std::to_string(has_stalled_ahead);

    const auto it_tl_front =
        std::upper_bound(tls_s.begin(), tls_s.end(), object_front_s);
    const double dist_to_tl = it_tl_front != tls_s.end()
                                  ? *it_tl_front - object_front_s
                                  : std::numeric_limits<double>::infinity();
    bool within_stl_ = (dist_to_tl > kMinTrafficWaitingDistance &&
                        dist_to_tl < kTrafficLightDistMaxThreshold);

    double distance_ = it->first - ego_s;
    double isolated_w = object.type() == ObjectType::OT_LARGE_VEHICLE ? 0.5
                        : distance_ > 0.0 && distance_ < 50.0         ? 0.5
                                                                      : 0.3;
    double traffic_w = isolated_w;
    if ((it + 1) != objects_on_lane.rend()) {
      const auto it_back = it + 1;
      if (IsStationary(*(it_back->second)) && (it_back)->first > ego_s) {
        bool is_back_brakelight =
            it_back->second->perception_object()
                .obstacle_light()
                .brake_lights() == ObstacleLightType::LIGHT_ON;
        if (is_back_brakelight) {
          traffic_w *= 1.5;
          isolated_w *= 2.0;
        }
      }
    }

    if (!has_stalled_ahead && !tls_s.empty() &&
        IsTrafficWaiting(tls_s, traffic_chain_front_object_front_s,
                         traffic_chain_front_object_back_s, object_back_s,
                         object_id)) {
      traffic_waiting_analysis.push_back({object_s, object_id});
    } else {
      if (within_stl_) {
        if ((it == objects_on_lane.rbegin() ||
             (it != objects_on_lane.rbegin() &&
              ((it - 1)->first - it->first) > kMinTrafficWaitingDistance)) &&
            stop_time_info.time_duration_since_stop() > 5.0 &&
            object.obstacle_light().brake_lights() ==
                ObstacleLightType::LIGHT_OFF) {
          stop_time_threshold *=
              std::max(traffic_w,
                       (1.0 - stop_time_info.time_duration_since_stop() / 8.0));
        }
      }
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " traffic_light: " +
                                std::to_string(stop_time_threshold) +
                                " traffic_w: " + std::to_string(traffic_w);

    stop_time_threshold *= CalculateFactorForSpecialZone(
        zones_s, {traffic_chain_front_object_back_s, object_back_s},
        kSpecialZoneDistThreshold, 2.0, object_id);
    stop_time_threshold_debug = stop_time_threshold_debug + " zones_s: " +
                                std::to_string(stop_time_threshold);

    const double max_factor_for_tl =
        (!is_leftmost_lane && !is_rightmost_lane) ? 13.0 : 10.0;

    stop_time_threshold *= CalculateFactorForSpecialZone(
        tls_s, {traffic_chain_front_object_back_s, object_back_s},
        kTrafficLightDistMaxThreshold, max_factor_for_tl, object_id);
    stop_time_threshold_debug = stop_time_threshold_debug + " tls_s: " +
                                std::to_string(stop_time_threshold);

    for (const auto& neighbor_lane : neighbors) {
      const auto neighbor_objects = CollectContinuousObjectsAheadOfS(
          *neighbor_lane, object_s - object_half_length);
      for (const auto& neighbor_ptr : neighbor_objects) {
        double factor = 0.0;
        const auto& neighbor_stop_time_info = neighbor_ptr->stop_time();
        if (IsStationary(*neighbor_ptr)) {
          factor = neighbor_stop_time_info.time_duration_since_stop() /
                   kDefaultStopTimeThreshold;
        } else {
          factor = CalculateFactorForMovingNeighbor(neighbor_stop_time_info);
        }
        stop_time_threshold *= 1.0 + std::min(1.0, factor);
      }
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " neighbors: " +
                                std::to_string(stop_time_threshold);

    if (CheckWhetherLanePathControlledByRedTrafficLight(tl_info_map,
                                                        lane_path)) {
      stop_time_threshold *= kDefaultRedTrafficLightMultiplier;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " red_light: " +
                                std::to_string(stop_time_threshold);

    if (stop_time_info.last_move_time_duration() > 0.0) {
      stop_time_threshold *=
          std::clamp(stop_time_info.last_move_time_duration() /
                         stop_time_info.time_duration_since_stop(),
                     1.0, 3.0);
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " moved: " +
                                std::to_string(stop_time_threshold);

    if (has_stalled_ahead) {
      stop_time_threshold *= 0.3;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " stalled: " +
                                std::to_string(stop_time_threshold);

    if (object.has_parked() && object.parked()) {
      stop_time_threshold *= 0.3;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " parked: " +
                                std::to_string(stop_time_threshold);
    if (FLAGS_planner_ignore_stalled_objects_on_tl_controlled_leftmost_lane &&
        is_leftmost_lane) {
      stop_time_threshold *=
          CalculateFactorForTrafficLightControlledLeftmostLane(tls_s);
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " tl_control: " +
                                std::to_string(stop_time_threshold);

    if (it == prev_cont_it && IsIsolatedLaneAtS(psmm, lane_path, object_s) &&
        IsFarAwayFromTlAtS(tls_s, object_s, kTrafficLightDistMaxThreshold)) {
      stop_time_threshold *= isolated_w;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " isolated: " +
                                std::to_string(stop_time_threshold) +
                                " isolated_w: " + std::to_string(isolated_w) +
                                " distance_: " + std::to_string(distance_);

    double theta_ = ArclengthToLerpTheta(psmm, lane_path, object_s);
    double theta_diff_ = std::fabs(NormalizeAngle(object.yaw() - theta_));
    stop_time_threshold *= CalculateFactorAccordHeadingDiff(
        object.yaw(), ArclengthToLerpTheta(psmm, lane_path, object_s));
    stop_time_threshold_debug =
        stop_time_threshold_debug + " theta_: " + std::to_string(theta_) +
        " theta_diff_: " + std::to_string(theta_diff_) +
        " heading: " + std::to_string(stop_time_threshold) +
        " object.yaw:" + std::to_string(object.yaw());

    if (!is_rightmost_lane) {
      for (auto it_front = prev_cont_it; it_front != it; ++it_front) {
        const auto front_ptr = it_front->second;
        if (IsStationary(*front_ptr)) {
          stop_time_threshold *=
              1.0 + (it - it_front) * std::min(1.0, FindOrDie(stall_analysis,
                                                              front_ptr->id()));
        } else {
          stop_time_threshold *=
              1.0 + (it - it_front) * CalculateFactorForMovingNeighbor(
                                          front_ptr->stop_time());
        }
      }
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " multiple: " +
                                std::to_string(stop_time_threshold);

    Box2d obstacle_box(object.bounding_box());
    double left_l = -std::numeric_limits<double>::infinity();
    double right_l = std::numeric_limits<double>::infinity();
    for (const auto& ps : obstacle_box.GetCornersCounterClockwise()) {
      const double corner_l = ff.XYToSL(ps).l;
      left_l = std::fmax(corner_l, left_l);
      right_l = std::fmin(corner_l, right_l);
    }
    if (is_rightmost_lane) {
      if (left_l < 0.0) {
        stop_time_threshold *= 0.3;
      } else if (left_l < 0.3 && object.obstacle_light().brake_lights() !=
                                     ObstacleLightType::LIGHT_ON) {
        stop_time_threshold *= 0.3;
      } else {
        stop_time_threshold *=
            (1.4 * std::fmin(0.5, std::fabs(left_l) / (std::fabs(left_l) +
                                                       std::fabs(right_l))) +
             0.3);
      }
    }

    stop_time_threshold_debug =
        stop_time_threshold_debug + " offset: " + std::to_string(left_l) + " " +
        std::to_string(right_l) + " " + std::to_string(stop_time_threshold);

    if (NormalizeAngle(object.yaw() - theta_) > M_PI / 6.0)
      stop_time_threshold *= 3.0;
    stop_time_threshold_debug = stop_time_threshold_debug + " heading-inner" +
                                std::to_string(stop_time_threshold);

    if (object.obstacle_light().brake_lights() == ObstacleLightType::LIGHT_ON) {
      stop_time_threshold *= 5.0;
    }
    stop_time_threshold_debug = stop_time_threshold_debug + " brake_light: " +
                                std::to_string(stop_time_threshold);

    double hazard_lights_w_ = is_rightmost_lane && !is_leftmost_lane
                                  ? (within_stl_ ? 0.3 : 0.2)
                              : (!is_leftmost_lane) ? (within_stl_ ? 0.6 : 0.4)
                              : (is_rightmost_lane && is_leftmost_lane) ? 0.3
                                                                        : 0.5;

    if (object.obstacle_light().hazard_lights() ==
        ObstacleLightType::LIGHT_ON) {
      stop_time_threshold *= hazard_lights_w_;
    }
    stop_time_threshold_debug =
        stop_time_threshold_debug +
        " hazard_lights: " + std::to_string(stop_time_threshold) +
        " hazard_lights_w_: " + std::to_string(hazard_lights_w_) +
        " within_stl_ " + std::to_string(within_stl_);

    const double stalled_prob =
        stop_time_info.time_duration_since_stop() / stop_time_threshold;
    if (stalled_prob >= kStalledProbThreshold) {
      has_stalled_ahead = true;
    }

    stall_analysis[object_id] = std::min(1.0, stalled_prob);
    Log2FG::LogDataV0("stop_time_threshold",
                      object_pred_ptr->id() + ": " + stop_time_threshold_debug);
  }

  TrafficWaitingQueueProto traffic_waiting_queue;
  std::stable_sort(traffic_waiting_analysis.begin(),
                   traffic_waiting_analysis.end());
  for (const auto& tw_info : traffic_waiting_analysis) {
    if (stall_analysis[tw_info.second] > kStalledProbThreshold) {
      continue;
    }
    traffic_waiting_queue.add_object_id(tw_info.second);
  }
  lane_path.ToProto(traffic_waiting_queue.mutable_lane_path());

  std::vector<ObjectAnnotationProto> object_annotations;
  object_annotations.reserve(stall_analysis.size());

  for (const auto& [object_id, stall_probability] : stall_analysis) {
    if (stall_probability >= kStalledProbThreshold) {
      ObjectAnnotationProto object_annotation;
      object_annotation.set_object_id(object_id);
      object_annotation.set_stalled_vehicle_likelyhood(stall_probability);
      object_annotation.set_depart_soon_likelyhood(1.0 - stall_probability);
      object_annotations.push_back(std::move(object_annotation));
    }
  }

  return AnalyzeOnLaneOutput{
      .traffic_waiting_queue = std::move(traffic_waiting_queue),
      .object_annotations = std::move(object_annotations),
      .distance_to_roadblock = distance_to_roadblock};
}

std::optional<double> MergeDistanceToAccidentZone(
    std::optional<double> dist_1, std::optional<double> dist_2) {
  if (dist_1.has_value() && dist_2.has_value()) {
    return std::min<double>(*dist_1, *dist_2);
  } else if (dist_1.has_value()) {
    return dist_1;
  } else if (dist_2.has_value()) {
    return dist_2;
  } else {
    return std::nullopt;
  }
}

TrafficFlowReasoningOutput MergeMultiAnalyzeOutput(
    std::vector<AnalyzeOnLaneOutput> analyze_outputs) {
  TrafficFlowReasoningOutput output;
  std::map<std::string, ObjectAnnotationProto> objects_analysis_map;

  output.traffic_waiting_queues.reserve(analyze_outputs.size());

  for (auto& analysis : analyze_outputs) {
    output.traffic_waiting_queues.push_back(
        std::move(analysis.traffic_waiting_queue));

    for (auto& object_annotation : analysis.object_annotations) {
      const auto& id = object_annotation.object_id();

      if (objects_analysis_map.find(id) == objects_analysis_map.end()) {
        objects_analysis_map.emplace(id, std::move(object_annotation));
      } else {
        const auto& pre_object_annotation = objects_analysis_map[id];

        if (pre_object_annotation.stalled_vehicle_likelyhood() <
            object_annotation.stalled_vehicle_likelyhood()) {
          objects_analysis_map[id] = std::move(object_annotation);
        }
      }
    }

    output.distance_to_roadblock = MergeDistanceToAccidentZone(
        output.distance_to_roadblock, analysis.distance_to_roadblock);
  }

  for (auto& pair : objects_analysis_map) {
    output.object_annotations.push_back(std::move(pair.second));
  }

  return output;
}
}  // namespace

absl::StatusOr<TrafficFlowReasoningOutput> RunTrafficFlowReasoning(
    const TrafficFlowReasoningInput& input, WorkerThreadManager* thread_pool,
    ObjectHistoryController& obj_manager) {
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.prediction);
  CHECK_NOTNULL(input.lane_paths);
  CHECK_NOTNULL(input.tl_info_map);
  CHECK_NOTNULL(input.plan_start_point);

  const auto& psmm = *input.psmm;
  const auto& prediction = *input.prediction;

  const auto& lane_paths = *input.lane_paths;
  const auto& tl_info_map = *input.tl_info_map;

  if (lane_paths.empty()) {
    return absl::InternalError("No Lane Path to make scene understanding");
  }

  std::vector<ObjectsOnLane> objects_on_lane_vec;
  const int lane_paths_size = lane_paths.size();
  objects_on_lane_vec.resize(lane_paths_size);
  for (auto& objects_on_lane : objects_on_lane_vec) {
    objects_on_lane.reserve(prediction.objects_size());
  }

  for (const auto& object_pred : prediction.objects()) {
    const auto& object = object_pred.perception_object();

    if (object.type() == ObjectType::OT_VEGETATION ||
        object.type() == ObjectType::OT_PEDESTRIAN ||
        object.type() == ObjectType::OT_CYCLIST) {
      continue;
    }

    bool has_associated = false;
    for (int i = lane_paths_size - 1; i >= 0; --i) {
      double s;
      const auto& lane_path = lane_paths[i];
      if (IsObjectOnLanePathByPose(psmm, lane_path, object, &s)) {
        objects_on_lane_vec[i].push_back({s, &object_pred});
        has_associated = true;
        break;
      }
    }

    if (has_associated == false) {
      for (int i = lane_paths_size - 1; i >= 0; --i) {
        double s;
        const auto& lane_path = lane_paths[i];
        if (IsObjectOnLanePathByBoundingBox(psmm, lane_path, object, &s)) {
          objects_on_lane_vec[i].push_back({s, &object_pred});
          break;
        }
      }
    }
  }

  for (auto& objects_on_lane : objects_on_lane_vec) {
    std::stable_sort(objects_on_lane.begin(), objects_on_lane.end());
  }

  const auto ego_pos = Extract2dVectorFromApolloProto(*input.plan_start_point);

  std::vector<AnalyzeOnLaneOutput> analyze_outputs(lane_paths_size);
  ParallelFor(0, lane_paths_size, thread_pool, [&](int i) {
    std::vector<const ObjectsOnLane*> neighbors;

    if (i != 0) {
      neighbors.push_back(&objects_on_lane_vec[i - 1]);
    }

    if (i != lane_paths_size - 1 && i != lane_paths_size - 2) {
      neighbors.push_back(&objects_on_lane_vec[i + 1]);
    }
    const auto ff = BuildBruteForceFrenetFrame(
        SampleLanePathPoints(psmm, lane_paths[i]), true);

    analyze_outputs[i] =
        AnalyzeOnLane(psmm, objects_on_lane_vec[i], lane_paths[i], tl_info_map,
                      ego_pos, neighbors, obj_manager, ff.value());
  });

  return MergeMultiAnalyzeOutput(std::move(analyze_outputs));
}
}  // namespace e2e_noa::planning
