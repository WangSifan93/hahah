#include "maps/map.h"

#include <typeindex>
#include <unordered_set>

#include "async/async_util.h"
#include "async/future.h"
#include "async/parallel_for.h"
#include "common/gflags.h"
#include "common/log.h"
#include "glog/logging.h"
#include "math/double.h"

namespace ad_e2e {
namespace planning {

using KDPoint2D = KDPoint<double, 2, KDValue>;
using Double = math::Double;

Map::Map(const MapInfo &map_info) : thread_pool_(0) {
  ConvertMapInfo(map_info);
  UpdateLane();
  UpdateJunction();
  UpdateSection();
  UpdateBoundarySection();
  InitAABoxKDTree2d();
}

LaneConstPtr Map::GetLaneById(const std::string &id) const {
  auto it = lane_map_.find(id);
  return it != lane_map_.end() ? it->second : nullptr;
}

ClearAreaConstPtr Map::GetClearAreaById(const std::string &id) const {
  auto it = clear_area_map_.find(id);
  return it != clear_area_map_.end() ? it->second : nullptr;
}

StopLineConstPtr Map::GetStopLineById(const std::string &id) const {
  auto it = stop_line_map_.find(id);
  return it != stop_line_map_.end() ? it->second : nullptr;
}

void Map::UpdateStopLineStatus(const std::string &id,
                               const LightStatus &status) {
  auto it = stop_line_map_.find(id);
  if (it != stop_line_map_.end()) {
    it->second->set_light_type(status);
    return;
  }
}

SpeedBumpConstPtr Map::GetSpeedBumpById(const std::string &id) const {
  auto it = speed_bump_map_.find(id);
  return it != speed_bump_map_.end() ? it->second : nullptr;
}
LaneConstPtr Map::GetNearestLane(const Point2d &pt, const double &dis) const {
  if (!IsValid()) {
    return nullptr;
  }
  std::map<double, LaneConstPtr> lane_candidate_map;
  for (const auto &it : lane_map_) {
    if (!it.second->center_line().IsValid()) continue;
    auto dis_temp = it.second->center_line().GetDistance(pt);
    if (dis_temp < dis) {
      lane_candidate_map[dis_temp] = it.second;
    }
  }
  LaneConstPtr res = nullptr;
  if (!lane_candidate_map.empty()) {
    res = lane_candidate_map.begin()->second;
  }
  return res;
}

LaneConstPtr Map::GetNearestLane(
    const std::vector<LaneConstPtr> &candidate_lanes, const Point2d &pt,
    const double &heading) const {
  if (!IsValid() || candidate_lanes.empty()) {
    return nullptr;
  }
  int32_t min_idx = 0;
  double min_value = std::numeric_limits<double>::max();
  for (int32_t i = 0; i < static_cast<int32_t>(candidate_lanes.size()); ++i) {
    SLPoint sl_point(0.0, 0.0);
    if (!candidate_lanes.at(i)->GetSLWithoutLimit(pt, &sl_point)) {
      continue;
    }
    double lane_heading = 0.0;
    if (!candidate_lanes.at(i)->GetHeadingFromS(sl_point.s, &lane_heading)) {
      continue;
    }
    double heading_diff =
        std::abs(ad_e2e::planning::math::AngleDiff(lane_heading, heading));

    if (heading_diff < min_value) {
      min_value = heading_diff;
      min_idx = i;
    }
  }
  return candidate_lanes.at(min_idx);
}

LaneConstPtr Map::GetNearestLane(const Point2d &pt, const double &heading,
                                 const double &dis, bool is_navi,
                                 bool is_virtual, double angle_range) const {
  if (!IsValid()) {
    return nullptr;
  }
  std::map<double, LaneConstPtr> lane_candidate_map;
  for (const auto &it : lane_map_) {
    if (!it.second->center_line().IsValid()) continue;
    if (is_navi && !it.second->is_navigation()) continue;
    if (is_virtual && it.second->type() != LANE_VIRTUAL_JUNCTION) continue;
    double s_temp;
    int index_temp1, index_temp2;
    ad_e2e::planning::math::Vec2d nearestpoint;
    auto dis_temp =
        it.second->center_line().GetDistance(pt, &nearestpoint, &s_temp);
    index_temp1 = std::min(it.second->center_line().GetIndexByS(s_temp),
                           (int)(it.second->center_line().points().size() - 2));
    index_temp2 = it.second->center_line().GetIndexByS(s_temp + 5.0);
    index_temp2 = index_temp2 == index_temp1 ? index_temp1 + 1 : index_temp2;

    double theta_temp = ad_e2e::planning::math::NormalizeAngle(
        (it.second->center_line().points().at(index_temp2) -
         it.second->center_line().points().at(index_temp1))
            .Angle());
    if (dis_temp < dis &&
        fabs(ad_e2e::planning::math::NormalizeAngle(theta_temp - heading)) <
            (it.second->turn_type() == U_TURN ? M_PI / 3.0 : angle_range)) {
      lane_candidate_map[dis_temp] = it.second;
    }
  }
  LaneConstPtr res = nullptr;
  if (!lane_candidate_map.empty()) {
    res = lane_candidate_map.begin()->second;
  }
  return res;
}

LaneConstPtr Map::GetLeftLane(const LaneConstPtr &lane) const {
  if (!IsValid() || !lane) {
    return nullptr;
  }
  auto left_it = lane_map_.find(lane->left_lane_id());
  if (left_it != lane_map_.end()) {
    return left_it->second;
  }
  return nullptr;
}

LaneConstPtr Map::GetRightLane(const LaneConstPtr &lane) const {
  if (!IsValid() || !lane) {
    return nullptr;
  }
  auto right_it = lane_map_.find(lane->right_lane_id());
  if (right_it != lane_map_.end()) {
    return right_it->second;
  }
  return nullptr;
}

void Map::GetPrecedeLanes(const LaneConstPtr &lane,
                          std::vector<LaneConstPtr> *const lanes,
                          bool only_navi) const {
  const double min_diff_dist = 1.0;
  if (!IsValid() || !lane) return;
  if (lane->pre_lane_ids().empty()) {
    return;
  }
  for (const auto &id : lane->pre_lane_ids()) {
    auto it = lane_map_.find(id);
    if (it != lane_map_.end()) {
      bool check =
          fabs(it->second->topo_length() - it->second->curve_length()) > 1.0 ||
          fabs(lane->topo_length() - lane->curve_length()) > 1.0;
      if (lane->center_line().IsValid() &&
          it->second->center_line().IsValid() && !check &&
          (lane->center_line().begin_point().DistanceTo(
               it->second->center_line().end_point()) >= min_diff_dist)) {
        continue;
      }
      if (only_navi && !lane->is_navigation()) {
        continue;
      }
      if (only_navi && !it->second->is_navigation()) {
        continue;
      }
      lanes->emplace_back(it->second);
    } else {
    }
  }
}

std::vector<LaneConstPtr> Map::GetValidNextLanes(
    const LaneConstPtr &lane) const {
  std::vector<LaneConstPtr> lanes;

  if (!IsValid() || !lane) return lanes;
  for (const auto &id : lane->next_lane_ids()) {
    auto it = lane_map_.find(id);
    if (it != lane_map_.end() && id.find("nextvr") == id.npos) {
      lanes.emplace_back(it->second);
    }
  }
  return lanes;
}

std::vector<LaneConstPtr> Map::GetNextLanes(const LaneConstPtr &lane) const {
  std::vector<LaneConstPtr> lanes;
  const double min_diff_dist = 2.0;
  if (!IsValid() || !lane) return lanes;
  for (const auto &id : lane->next_lane_ids()) {
    auto it = lane_map_.find(id);

    if (it != lane_map_.end()) {
      bool check =
          fabs(it->second->topo_length() - it->second->curve_length()) > 1.0 ||
          fabs(lane->topo_length() - lane->curve_length()) > 1.0;
      if (lane->center_line().IsValid() &&
          it->second->center_line().IsValid() && !check &&
          (lane->center_line().end_point().DistanceTo(
               it->second->center_line().begin_point()) >= min_diff_dist)) {
        LOG(ERROR) << "[map] next connected lane diff ["
                   << lane->center_line().end_point().DistanceTo(
                          it->second->center_line().begin_point())
                   << "] > 2.0m";
        continue;
      }
      lanes.emplace_back(it->second);
    } else {
    }
  }
  return lanes;
}

LaneSequencePtr Map::GetLaneSequence(const LaneConstPtr &lane,
                                     const bool &navi_is_priority) const {
  LOG(INFO) << "Map::GetLaneSequence()";
  auto cur_lane = GetSameLane(lane);
  if (!IsValid() || !cur_lane) {
    return nullptr;
  }
  std::vector<LaneConstPtr> lanes;
  lanes.emplace_back(cur_lane);
  while (cur_lane) {
    cur_lane = GetOptimalNextLane(cur_lane, navi_is_priority);
    if (!cur_lane) {
      break;
    }
    bool flag = false;
    for (const auto &exist_lane : lanes) {
      if (exist_lane->id() == cur_lane->id()) {
        flag = true;
      }
    }
    if (flag) {
      break;
    }
    lanes.emplace_back(cur_lane);
  }
  return std::make_shared<LaneSequence>(lanes);
}

LaneSequencePtr Map::GetSameLaneSequenceV2(
    const LaneSequencePtr &lane_sequence, const double &x, const double &y,
    const double &heading, std::vector<std::string> &debug,
    std::vector<Point2d> &debug_match_point) const {
  LOG(INFO) << "Map::GetSameLaneSequenceV2()";
  if (!IsValid() || !lane_sequence || !lane_sequence->IsValid()) {
    return nullptr;
  }
  LaneConstPtr nearest_lane = lane_sequence->GetNearestLane(math::Vec2d{x, y});
  if (!nearest_lane) {
    return nullptr;
  }

  std::vector<LaneConstPtr> last_valid_lanes;
  bool add = false;
  for (const auto &lane : lane_sequence->lanes()) {
    if (!lane->center_line().IsValid()) break;
    if (!add) {
      auto d = lane->center_line().GetDistance(x, y);
      if (d < 30.0) {
        add = true;
      }
    }
    if (add) {
      last_valid_lanes.emplace_back(lane);
    }
  }
  if (last_valid_lanes.empty()) {
    LOG(ERROR) << "GetSameLaneSequenceV2(): last lanes is empty!";
    return nullptr;
  }
  LaneSequencePtr last_valid_laneseq =
      std::make_shared<ad_e2e::planning::LaneSequence>(last_valid_lanes);
  debug.emplace_back("last first id:" +
                     last_valid_laneseq->lanes().front()->id());

  const double sample_interval = 1.0;
  const double near_range_min = 20.0;
  const double near_range_max = 50.0;
  const int near_interval = 5;
  const int far_interval = 10;

  std::vector<Point2d> sample_points;
  double ego_s_offset = 0.0;
  last_valid_laneseq->GetProjectionDistance({x, y}, &ego_s_offset);
  double sample_s_min = std::fmax(ego_s_offset - near_range_min, 0.0);
  last_valid_laneseq->SamplePoints(sample_s_min, &sample_points,
                                   sample_interval);
  std::vector<Point2d> supple_sample_pts;
  for (const auto &lane : last_valid_laneseq->lanes()) {
    if (!lane || !lane->IsValid() ||
        lane->curve_length() > near_interval * sample_interval + 1.0) {
      continue;
    }
    const double sample_s = lane->curve_length() * 0.5;
    supple_sample_pts.emplace_back(lane->center_line().GetPointAtS(sample_s));
    supple_sample_pts.emplace_back(
        lane->center_line().GetPointAtS(sample_s + 1.0));
  }
  debug.emplace_back("match range s min:" + std::to_string(sample_s_min));

  std::map<std::string, int> match_lane_ids;
  double accumulate_s = sample_s_min;
  for (int i = 0; !sample_points.empty() && i < sample_points.size() - 1;) {
    auto point = sample_points[i];
    auto point_1 = sample_points[i + 1];
    auto theta = atan2(point_1.y() - point.y(), point_1.x() - point.x());
    LaneConstPtr lane =
        GetNearestLane(Point2d(point.x(), point.y()), theta, 2.0, true);
    if (lane) {
      if (match_lane_ids.count(lane->id()) == 0) match_lane_ids[lane->id()] = 1;
      if (match_lane_ids.count(lane->id()) != 0) match_lane_ids[lane->id()]++;
    }
    if (accumulate_s < ego_s_offset + near_range_max) {
      i += near_interval;
      accumulate_s += near_interval * sample_interval;
    } else {
      i += far_interval;
      accumulate_s += far_interval * sample_interval;
    }
    debug_match_point.emplace_back(point);
  }
  for (int i = 0;
       !supple_sample_pts.empty() && i < supple_sample_pts.size() - 1; i += 2) {
    auto point = supple_sample_pts[i];
    auto point_1 = supple_sample_pts[i + 1];
    auto theta = atan2(point_1.y() - point.y(), point_1.x() - point.x());
    LaneConstPtr lane =
        GetNearestLane(Point2d(point.x(), point.y()), theta, 2.0, true);
    if (lane) {
      if (match_lane_ids.count(lane->id()) == 0) match_lane_ids[lane->id()] = 1;
      if (match_lane_ids.count(lane->id()) != 0) match_lane_ids[lane->id()]++;
    }
    debug_match_point.emplace_back(point);
  }
  if (match_lane_ids.empty()) {
    LOG(ERROR) << "GetSameLaneSequenceV2(): match lanes is empty!";
    return nullptr;
  }
  std::string de = "match_lane_ids:";
  for (const auto &it : match_lane_ids) {
    de += "(" + it.first + "-" + std::to_string(it.second) + ")";
  }
  debug.emplace_back(de);

  std::vector<std::string> best_ids;
  best_ids.emplace_back(match_lane_ids.begin()->first);
  int max_match_cnt = match_lane_ids.begin()->second;
  for (const auto &match_lane : match_lane_ids) {
    if (match_lane.second > max_match_cnt) {
      best_ids.clear();
      best_ids.emplace_back(match_lane.first);
      max_match_cnt = match_lane.second;
    } else if (match_lane.second == max_match_cnt &&
               std::find(best_ids.begin(), best_ids.end(), match_lane.first) ==
                   best_ids.end()) {
      best_ids.emplace_back(match_lane.first);
    }
  }
  std::vector<LaneConstPtr> best_lanes;
  for (const auto &lane_id : best_ids) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (!lane) continue;
    best_lanes.emplace_back(lane);
  }
  LaneConstPtr best_lane = GetNearestLane(best_lanes, {x, y}, heading);
  if (!best_lane) {
    LOG(ERROR) << "GetSameLaneSequenceV2(): best lane is null!";
    return nullptr;
  }
  de = "best_ids:";
  for (const auto &it : best_ids) {
    de += "(" + it + ")";
  }
  debug.emplace_back(de);

  std::vector<LaneConstPtr> valid_lanes;
  valid_lanes.emplace_back(best_lane);

  while (valid_lanes.front()) {
    int max_match_cnt = 0;
    std::string best_id = "";
    for (const auto &lane_id : valid_lanes.front()->pre_lane_ids()) {
      if (match_lane_ids.count(lane_id) != 0 &&
          match_lane_ids[lane_id] > max_match_cnt) {
        max_match_cnt = match_lane_ids[lane_id];
        best_id = lane_id;
      }
    }
    LaneConstPtr lane = GetLaneById(best_id);
    if (!lane || best_id == valid_lanes.front()->id()) break;
    valid_lanes.insert(valid_lanes.begin(), lane);
  }

  while (valid_lanes.back()) {
    int max_match_cnt = 0;
    std::string best_id = "";
    for (const auto &lane_id : valid_lanes.back()->next_lane_ids()) {
      if (match_lane_ids.count(lane_id) != 0 &&
          match_lane_ids[lane_id] > max_match_cnt) {
        max_match_cnt = match_lane_ids[lane_id];
        best_id = lane_id;
      }
    }
    LaneConstPtr lane = GetLaneById(best_id);
    if (!lane || best_id == valid_lanes.back()->id()) break;
    valid_lanes.emplace_back(lane);
  }
  de = "valid_lanes:";
  for (const auto &it : valid_lanes) {
    de += "(" + it->id() + ")";
  }
  debug.emplace_back(de);

  return std::make_shared<LaneSequence>(valid_lanes);
}

LaneSequencePtr Map::GetSameLaneSequence(const LaneSequencePtr &lane_sequence,
                                         const double &x, const double &y) {
  std::vector<LaneConstPtr> lanes;
  if (!IsValid() || !lane_sequence || !lane_sequence->IsValid()) {
    return nullptr;
  }
  LaneConstPtr nearest_lane = lane_sequence->GetNearestLane(math::Vec2d{x, y});
  if (!nearest_lane) {
    return nullptr;
  }
  bool has_find_nearest = false;
  bool add = false;
  for (const auto &lane : lane_sequence->lanes()) {
    if (lane->id() == nearest_lane->id()) {
      has_find_nearest = true;
    }
    if (!lane->center_line().IsValid()) break;
    if (!add) {
      auto d = lane->center_line().GetDistance(x, y);
      if (d < 60.0) {
        add = true;
      }
    }
    if (add) {
      auto cur = GetSameLane(lane);
      if (cur && (cur->center_line().IsValid() || has_find_nearest)) {
        lanes.emplace_back(cur);
        const auto next_lanes = GetNextLanes(cur);
        if (next_lanes.empty() && !has_find_nearest) lanes.clear();
        if (next_lanes.empty() && has_find_nearest) break;
      } else {
        if (has_find_nearest) {
          break;
        } else {
          lanes.clear();
        }
      }
    }
  }
  if (lanes.empty()) {
    LOG(ERROR) << "GetSameLaneSequence():lanes is empty!";
    return nullptr;
  }
  return std::make_shared<LaneSequence>(lanes);
}

LaneConstPtr Map::GetOptimalNextLane(const LaneConstPtr &lane,
                                     const bool &navi_is_priority) const {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  std::vector<LaneConstPtr> next_candidates;
  bool next_is_navi = false;
  for (const auto &next_lane : next_lanes) {
    if (navi_is_priority && next_lane->is_navigation()) {
      next_candidates.emplace_back(next_lane);
      next_is_navi = true;
    }
  }
  if (next_candidates.empty()) {
    next_candidates.assign(next_lanes.begin(), next_lanes.end());
  }
  std::map<double, LaneConstPtr> next_map;
  for (const auto &next_lane : next_candidates) {
    double cost = 0.0;
    if (next_is_navi && route_) {
      cost += 5.0 * std::abs(route_->GetPriorityLaneRelation(next_lane));
    }
    if (next_lane->is_split()) {
      cost += 10.0;
    }
    if (lane->type() != next_lane->type()) {
      cost += 5.0;
    }

    if (next_is_navi) {
      cost -= std::min(1000.0, next_lane->navi_distance());
    }
    next_map[cost] = next_lane;
  }
  return next_map.begin()->second;
}

LaneConstPtr Map::GetOptimalSmoothNextLane(const LaneConstPtr &lane,
                                           const bool &navi_is_priority,
                                           std::string &debug) const {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  std::vector<LaneConstPtr> next_candidates;
  bool next_is_navi = false;
  for (const auto &next_lane : next_lanes) {
    if (navi_is_priority && next_lane->is_navigation()) {
      next_candidates.emplace_back(next_lane);
      next_is_navi = true;
    }
  }
  if (next_candidates.empty()) {
    next_candidates.assign(next_lanes.begin(), next_lanes.end());
  }
  std::map<double, LaneConstPtr> next_map;
  for (const auto &next_lane : next_candidates) {
    if (!next_lane) continue;
    debug += "(" + next_lane->id() + ":";
    double cost = 0.0;
    if (next_is_navi) {
      debug += "navi=" + std::to_string(next_lane->navi_section_cnt()) + ",";
      cost -= 100.0 * next_lane->navi_section_cnt();
    }
    if (next_lane->is_split()) {
      debug += "topo=10.0,";
      cost += 10.0;
    }

    const double ref_start_s = std::fmax(lane->curve_length() - 15.0, 0.0);
    math::Vec2d ref_vec = lane->center_line().end_point() -
                          lane->center_line().GetPointAtS(ref_start_s);

    std::vector<std::pair<double, double>> distances;
    double dis_avg = 0.0;
    for (int i = 0; i < 4; i++) {
      double next_end_s = std::fmin(next_lane->curve_length(), 15.0 - 3.0 * i);
      math::Vec2d next_vec = next_lane->center_line().GetPointAtS(next_end_s) -
                             next_lane->center_line().begin_point();
      next_vec = math::Vec2d::CreateUnitVec2d(next_vec.Angle()) * 10.0;
      double distance = std::fabs(ref_vec.CrossProd(next_vec) /
                                  std::fmax(ref_vec.Length(), Constants::ZERO));
      distances.emplace_back(distance, 0.0);
      dis_avg += distance;
    }
    for (auto &i : distances) {
      i.second = std::fabs(i.first - dis_avg * 0.25);
    }
    std::sort(distances.begin(), distances.end(),
              [](const auto &lhs, const auto &rhs) {
                return lhs.second < rhs.second;
              });
    double distance = (dis_avg - distances.back().first) / 3.0;
    if (next_lane->center_line().IsValid()) {
      cost += distance;
    } else {
      cost += 10.0;
    }
    debug += "distances=" + std::to_string(distance) + "-rm-" +
             std::to_string(distances.back().first) + ")";
    next_map[cost] = next_lane;
  }
  if (next_map.empty()) return nullptr;
  return next_map.begin()->second;
}

LaneConstPtr Map::GetSameLane(const LaneConstPtr &lane) const {
  if (lane == nullptr) {
    return nullptr;
  }
  auto it = lane_map_.find(lane->id());
  if (it != lane_map_.end()) {
    return it->second;
  }
  if (lane->center_line().points().empty()) {
    return nullptr;
  }
  auto point =
      lane->center_line().GetPointAtS(lane->center_line().length() * 0.25);
  auto point_1 = lane->center_line().GetPointAtS(
      lane->center_line().length() * 0.25 + 1.0);
  auto theta = atan2(point_1.y() - point.y(), point_1.x() - point.x());
  return GetNearestLane(Point2d(point.x(), point.y()), theta, 1.0);
}

bool Map::IsValid() const { return !lane_map_.empty(); }

void Map::GetPolygonSRange(const LaneConstPtr &lane,
                           const math::Polygon2d &polygon, double *s_min,
                           double *s_max) const {
  if (!lane || !lane->center_line().IsValid() || !s_min || !s_max) return;
  const auto &pts = lane->center_line().points();
  std::size_t idx = 1u;
  *s_min = 0.0;
  *s_max = 0.0;
  if (polygon.IsPointIn(pts.front()) && polygon.IsPointIn(pts.back())) {
    *s_max = lane->curve_length();
    return;
  }
  Point2d first_pt, second_pt;
  for (; idx < pts.size(); idx++) {
    *s_min += pts[idx - 1].DistanceTo(pts[idx]);
    *s_max += pts[idx - 1].DistanceTo(pts[idx]);
    math::LineSegment2d segment(pts[idx - 1], pts[idx]);
    if (!polygon.IsPointIn(pts[idx]) &&
        polygon.GetOverlap(segment, &first_pt, &second_pt)) {
      *s_min -= first_pt.DistanceTo(pts[idx]);
      *s_max -= second_pt.DistanceTo(pts[idx]);
      return;
    } else if (!polygon.IsPointIn(pts[idx - 1]) &&
               polygon.IsPointIn(pts[idx]) &&
               polygon.GetOverlap(segment, &first_pt, &second_pt)) {
      *s_min -= first_pt.DistanceTo(pts[idx]);
      break;
    }
  }
  if (polygon.IsPointIn(pts.back())) {
    *s_max = lane->curve_length();
    return;
  }
  idx++;
  for (; idx < pts.size(); idx++) {
    *s_max += pts[idx - 1].DistanceTo(pts[idx]);
    math::LineSegment2d segment(pts[idx - 1], pts[idx]);
    if (polygon.IsPointIn(pts[idx - 1]) && !polygon.IsPointIn(pts[idx]) &&
        polygon.GetOverlap(segment, &first_pt, &second_pt)) {
      *s_max -= second_pt.DistanceTo(pts[idx]);
      break;
    }
  }
}

void Map::ConvertMapInfo(const MapInfo &map_info) {
  is_on_highway_ = map_info.is_on_highway;
  type_ = map_info.type;
  sub_type_ = map_info.sub_type;

  v2_info_ = map_info.v2_info;
  seq_ = map_info.seq;
  timestamp_ = map_info.timestamp;
  for (const auto &boundary_info : map_info.all_lane_boundaries_vec) {
    auto boundary_ptr = std::make_shared<LaneBoundary>(boundary_info);
    lane_boundary_map_[boundary_info.id] = boundary_ptr;
  }

  for (auto &road_boundary : map_info.all_road_boundaries_vec) {
    road_boundary_map_[road_boundary.id] =
        std::make_shared<RoadBoundary>(road_boundary);
  }

  for (const auto &lane_info : map_info.all_lanes_vec) {
    LaneBoundariesPtr left_boundaries_ptr =
        BuildLaneBoundaries(lane_info.left_lane_boundary_ids);
    LaneBoundariesPtr right_boundaries_ptr =
        BuildLaneBoundaries(lane_info.right_lane_boundary_ids);
    RoadBoundariesConstPtr left_road_boundaries_ptr =
        BuildRoadBoundaries(lane_info.left_road_boundary_ids);
    RoadBoundariesConstPtr right_road_boundaries_ptr =
        BuildRoadBoundaries(lane_info.right_road_boundary_ids);
    for (const auto &left_bound_id : lane_info.left_lane_boundary_ids) {
      if (lane_boundary_map_.find(left_bound_id) != lane_boundary_map_.end()) {
        lane_boundary_map_.at(left_bound_id)->PushLeftLane(lane_info.id);
      }
    }
    for (const auto &right_bound_id : lane_info.right_lane_boundary_ids) {
      if (lane_boundary_map_.find(right_bound_id) != lane_boundary_map_.end()) {
        lane_boundary_map_.at(right_bound_id)->PushLeftLane(lane_info.id);
      }
    }

    lane_map_[lane_info.id] = std::make_shared<Lane>(
        lane_info, left_boundaries_ptr, right_boundaries_ptr,
        left_road_boundaries_ptr, right_road_boundaries_ptr);
  }

  for (const auto &clear_area : map_info.all_clear_areas_vec) {
    clear_area_map_[clear_area.id] = std::make_shared<ClearArea>(clear_area);
  }

  route_ = std::make_shared<Route>(map_info.route, this);

  double next_section_length = 0.0;
  for (auto section = map_info.route.sections.rbegin();
       section != map_info.route.sections.rend(); section++) {
    double section_length = section->length;
    for (const auto &lane_id : section->lane_ids) {
      LanePtr lane;
      if (lane_map_.find(lane_id) != lane_map_.end()) {
        lane = lane_map_[lane_id];
      }
      if (!lane) continue;

      section_length = std::max(section_length, lane->topo_length());

      double navi_distance = 0.0;
      int navi_section_cnt = 1;
      for (const auto &next_id : lane->next_lane_ids()) {
        auto next_lane = GetLaneById(next_id);
        if (next_lane && next_lane->is_navigation()) {
          navi_distance = std::max(
              navi_distance, next_section_length + next_lane->navi_distance());
          navi_section_cnt =
              std::max(navi_section_cnt, 1 + next_lane->navi_section_cnt());
        }
      }

      lane->SetIsNavigation(true, navi_distance, navi_section_cnt);
      lane->SetSectionId(section->id);

      lane->SetTrueLength(std::fmax(section->length, lane->topo_length()));
    }

    next_section_length = section_length;
  }
  for (const auto &junction_info : map_info.all_junctions_vec) {
    if (junction_info.id.empty() || junction_info.points.size() < 3u) {
      continue;
    }
    junction_map_[junction_info.id] = std::make_shared<Junction>(junction_info);
  }

  for (const auto &cross_walk_info : map_info.all_cross_walks_vec) {
    crosswalk_map_[cross_walk_info.id] =
        std::make_shared<Crosswalk>(cross_walk_info);
  }

  for (const auto &stop_line_info : map_info.all_stop_lines_vec) {
    stop_line_map_[stop_line_info.id] =
        std::make_shared<StopLine>(stop_line_info);
  }

  for (const auto &speed_bump_info : map_info.all_speed_bumps_vec) {
    speed_bump_map_[speed_bump_info.id] =
        std::make_shared<SpeedBump>(speed_bump_info);
  }

  for (const auto &traffic_light_info : map_info.all_traffic_lights_vec) {
    traffic_light_info_map_[traffic_light_info.id] =
        std::make_shared<TrafficLightInfo>(traffic_light_info);
  }
}

void Map::UpdateLane() {
  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    for (const auto &next_id : lane_ptr->next_lane_ids()) {
      if (lane_map_.find(next_id) == lane_map_.end()) {
        continue;
      }
      auto &next_lane_ptr = lane_map_.at(next_id);
      next_lane_ptr->AddPreviousLane(lane_ptr->id());
    }
    if (junction_map_.find(lane_ptr->junction_id()) != junction_map_.end()) {
      auto &junction_ptr = junction_map_.at(lane_ptr->junction_id());
      junction_ptr->PushOverlapLane(lane_ptr);
    }
  }

  for (const auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    TrafficLightStatus traffic_light_status;
    traffic_light_status.lane_id = id_lane.first;
    if (!lane_ptr->junction_id().empty()) {
      traffic_light_status.junction_id = lane_ptr->junction_id();
    }
    traffic_light_status.light_status =
        static_cast<LightStatus>(lane_ptr->light_status());
    traffic_light_status.stop_line = lane_ptr->stop_line(),
    traffic_light_status.is_left_wait_lane =
        lane_ptr->type() == LANE_LEFT_WAIT ? true : false;
    traffic_light_status_map_[traffic_light_status.lane_id] =
        traffic_light_status;
  }

  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;

    std::vector<std::string> valid_pre_lanes;
    for (const auto &lane_id : lane_ptr->pre_lane_ids()) {
      if (lane_map_.find(lane_id) != lane_map_.end() &&
          lane_map_.at(lane_id)->IsValid()) {
        valid_pre_lanes.emplace_back(lane_id);
      }
    }
    lane_ptr->SetValidPredLaneIds(std::move(valid_pre_lanes));

    std::vector<std::string> valid_next_lanes;
    std::unordered_set<TurnType> next_turn_types;
    for (const auto &lane_id : lane_ptr->next_lane_ids()) {
      if (lane_map_.find(lane_id) != lane_map_.end() &&
          lane_map_.at(lane_id)->IsValid()) {
        valid_next_lanes.emplace_back(lane_id);
        next_turn_types.insert(lane_map_.at(lane_id)->turn_type());
      }
    }
    lane_ptr->SetValidNextLaneIds(std::move(valid_next_lanes));

    lanes_.emplace_back(lane_ptr);
  }

  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    const math::LineCurve2d &center_line = lane_ptr->center_line();
    math::Vec2d process_lane_vector =
        center_line.end_point() - center_line.begin_point();

    std::vector<std::string> sorted_next_lanes;
    std::unordered_map<std::string, double> angle_of_next_lanes;

    for (const auto &lane_id : lane_ptr->next_lane_ids()) {
      if (lane_map_.find(lane_id) != lane_map_.end() &&
          lane_map_.at(lane_id)->IsValid()) {
        const auto &next_lane_ptr = lane_map_.at(lane_id);
        const math::LineCurve2d &next_center_line =
            next_lane_ptr->center_line();
        math::Vec2d next_lane_vector =
            next_center_line.end_point() - next_center_line.begin_point();
        double epsilon = 1e-10;
        double denominator =
            process_lane_vector.Length() * next_lane_vector.Length();
        if (denominator < epsilon) {
          continue;
        }
        double inner_prod = process_lane_vector.InnerProd(next_lane_vector);
        double cos_angle = inner_prod / denominator;
        double angle_rad = acos(cos_angle);
        if (process_lane_vector.CrossProd(next_lane_vector) < 0) {
          angle_rad = -angle_rad;
        }
        angle_of_next_lanes.insert(std::make_pair(lane_id, angle_rad));
        sorted_next_lanes.emplace_back(lane_id);
      }
    }
    std::sort(sorted_next_lanes.begin(), sorted_next_lanes.end(),
              [&](const std::string &poly_id_1, const std::string &poly_id_2) {
                return angle_of_next_lanes.at(poly_id_1) >
                       angle_of_next_lanes.at(poly_id_2);
              });
    lane_ptr->SetSortedNextLaneIds(std::move(sorted_next_lanes));
  }

  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    std::size_t lane_ind_in_section = 1u;
    auto cur_lane_ptr = lane_ptr;
    std::vector<std::string> processed_lane_ids;
    while (!cur_lane_ptr->left_lane_id().empty() &&
           std::find(processed_lane_ids.begin(), processed_lane_ids.end(),
                     cur_lane_ptr->id()) == processed_lane_ids.end()) {
      processed_lane_ids.emplace_back(cur_lane_ptr->id());

      auto left_lane_it = lane_map_.find(cur_lane_ptr->left_lane_id());
      if (left_lane_it == lane_map_.end()) {
        break;
      }
      lane_ind_in_section++;
      cur_lane_ptr = left_lane_it->second;
    }
    lane_ptr->SetLaneIndInSection(lane_ind_in_section);
  }
}

void Map::UpdateJunction() {
  e2e_noa::MapParallelFor(
      junction_map_.begin(), junction_map_.end(),
      [&](std::pair<std::string, JunctionPtr> id_junction) {
        this->SetEntryLanesForJunction(id_junction.second);
        this->SetExitLanesForJunction(id_junction.second);
        this->SetCrosswalkForJunction(id_junction.second);
        const auto &junction_ptr = id_junction.second;
        LINFO("junction %s entry/exit lane %lu/%lu crosswalks %lu",
              junction_ptr->id().c_str(), junction_ptr->entry_lanes().size(),
              junction_ptr->exit_lanes().size(),
              junction_ptr->crosswalks().size());
      });
  for (auto &id_junction : junction_map_) {
    junctions_.emplace_back(id_junction.second);
  }
}

void Map::UpdateSection() {
  for (auto section : route_->GetRouteInfo().sections) {
    SectionPtr section_ptr = std::make_shared<Section>();
    section_ptr->set_id(section.id);
    for (const auto &lane_id : section.lane_ids) {
      const auto &lane_it = lane_map_.find(lane_id);
      if (lane_it == lane_map_.end()) {
        LOG(ERROR) << "section_id: " << section.id << "=> lane id: " << lane_id
                   << " not found";
        continue;
      }
      auto lane = lane_it->second;
      section_ptr->add_lanes(lane->id());

      section_ptr->set_curve_length(section.length);
      section_ptr->set_topo_length(section.length);
      for (const auto &next_lane : lane->next_lane_ids()) {
        if (lane_map_.find(next_lane) != lane_map_.end()) {
          section_ptr->add_outgoing_sections(
              lane_map_[next_lane]->section_id());
        }
      }
      for (const auto &pre_lane : lane->pre_lane_ids()) {
        if (lane_map_.find(pre_lane) != lane_map_.end()) {
          section_ptr->add_outgoing_sections(lane_map_[pre_lane]->section_id());
        }
      }
    }
    section_map_[section_ptr->id()] = section_ptr;
  }
}

void Map::UpdateBoundarySection() {
  for (const auto &lane : lanes_) {
    if (lane->left_boundary()) {
      for (auto lane_boundary : lane->left_boundary()->lane_boundaries()) {
        if (lane_boundary->section_id().empty() &&
            lane_boundary_map_.find(lane_boundary->id()) !=
                lane_boundary_map_.end()) {
          auto mutable_lane_boundary =
              lane_boundary_map_.at(lane_boundary->id());
          mutable_lane_boundary->set_section_id(lane->section_id());
        }
      }
    }

    if (lane->right_boundary()) {
      for (auto lane_boundary : lane->right_boundary()->lane_boundaries()) {
        if (lane_boundary->section_id().empty() &&
            lane_boundary_map_.find(lane_boundary->id()) !=
                lane_boundary_map_.end()) {
          auto mutable_lane_boundary =
              lane_boundary_map_.at(lane_boundary->id());
          mutable_lane_boundary->set_section_id(lane->section_id());
        }
      }
    }
    if (lane->left_road_boundary()) {
      for (auto road_boundary : lane->left_road_boundary()->road_boundaries()) {
        if (road_boundary->section_id().empty() &&
            road_boundary_map_.find(road_boundary->id()) !=
                road_boundary_map_.end()) {
          auto mutable_road_boundary =
              road_boundary_map_.at(road_boundary->id());
          mutable_road_boundary->set_section_id(lane->section_id());
        }
      }
    }
    if (lane->right_road_boundary()) {
      for (auto road_boundary :
           lane->right_road_boundary()->road_boundaries()) {
        if (road_boundary->section_id().empty() &&
            road_boundary_map_.find(road_boundary->id()) !=
                road_boundary_map_.end()) {
          auto mutable_road_boundary =
              road_boundary_map_.at(road_boundary->id());
          mutable_road_boundary->set_section_id(lane->section_id());
        }
      }
    }
  }
}

void Map::SetCrosswalkForJunction(JunctionPtr &junction_ptr) {
  std::unordered_set<CrosswalkConstPtr> crosswalks;
  std::unordered_set<std::string> crosswalk_ids;
  for (const auto &overlap_lane_ptr : junction_ptr->overlap_lanes()) {
    for (const auto &crosswalk_id : overlap_lane_ptr->crosswalks()) {
      if (crosswalk_ids.find(crosswalk_id) == crosswalk_ids.end() &&
          crosswalk_map_.find(crosswalk_id) != crosswalk_map_.end() &&
          crosswalk_map_.at(crosswalk_id) != nullptr) {
        crosswalk_ids.insert(crosswalk_id);
        crosswalks.insert(crosswalk_map_.at(crosswalk_id));
      }
    }
  }
  junction_ptr->SetCrosswalks(std::move(crosswalks));
}

void Map::SetEntryLanesForJunction(JunctionPtr &junction_ptr) {
  std::vector<LaneConstPtr> entry_lanes;
  std::unordered_set<std::string> entry_lane_ids;
  for (const auto &overlap_lane_ptr : junction_ptr->overlap_lanes()) {
    for (const auto &lane_id : overlap_lane_ptr->pre_lane_ids()) {
      auto lane_ptr = GetLaneById(lane_id);
      if (lane_ptr == nullptr || lane_ptr->IsVirtual() ||
          entry_lane_ids.find(lane_id) != entry_lane_ids.end()) {
        continue;
      }
      LDEBUG("lane id %s predecessor %s nullptr %d virtual %d find %d",
             overlap_lane_ptr->id().c_str(), lane_id.c_str(),
             lane_ptr == nullptr, lane_ptr->IsVirtual(),
             entry_lane_ids.find(lane_id) != entry_lane_ids.end());
      entry_lane_ids.insert(lane_id);
      entry_lanes.emplace_back(lane_ptr);
    }
  }
  junction_ptr->SetEntryLanes(entry_lanes);
}

void Map::SetExitLanesForJunction(JunctionPtr &junction_ptr) {
  std::vector<LaneConstPtr> exit_lanes;
  std::unordered_set<std::string> exit_lane_ids;
  for (const auto &overlap_lane_ptr : junction_ptr->overlap_lanes()) {
    for (const auto &lane_id : overlap_lane_ptr->next_lane_ids()) {
      auto lane_ptr = GetLaneById(lane_id);
      if (lane_ptr == nullptr || lane_ptr->IsVirtual() ||
          exit_lane_ids.find(lane_id) != exit_lane_ids.end()) {
        continue;
      }
      LDEBUG("lane id %s successor %s nullptr %d virtual %d find %d",
             overlap_lane_ptr->id().c_str(), lane_id.c_str(),
             lane_ptr == nullptr, lane_ptr->IsVirtual(),
             exit_lane_ids.find(lane_id) != exit_lane_ids.end());
      exit_lane_ids.insert(lane_id);
      exit_lanes.emplace_back(lane_ptr);
    }
  }
  std::unordered_map<std::string, LaneConstPtr> neighbors_lane_map;
  for (const auto &lane_id : exit_lane_ids) {
    auto lane_ptr = GetLaneById(lane_id);
    if (lane_ptr == nullptr) {
      continue;
    }

    {
      const auto &left_lane_id = lane_ptr->left_lane_id();
      auto left_lane_ptr = GetLaneById(left_lane_id);
      if (left_lane_ptr != nullptr && !left_lane_ptr->IsVirtual() &&
          exit_lane_ids.find(left_lane_id) == exit_lane_ids.end() &&
          neighbors_lane_map.find(left_lane_id) == neighbors_lane_map.end()) {
        neighbors_lane_map.insert(std::make_pair(left_lane_id, left_lane_ptr));
      }
    }

    {
      const auto &right_lane_id = lane_ptr->right_lane_id();
      auto right_lane_ptr = GetLaneById(right_lane_id);
      if (right_lane_ptr != nullptr && !right_lane_ptr->IsVirtual() &&
          exit_lane_ids.find(right_lane_id) == exit_lane_ids.end() &&
          neighbors_lane_map.find(right_lane_id) == neighbors_lane_map.end()) {
        neighbors_lane_map.insert(
            std::make_pair(right_lane_id, right_lane_ptr));
      }
    }
  }
  for (const auto &neight_lane : neighbors_lane_map) {
    exit_lanes.emplace_back(neight_lane.second);
  }
  junction_ptr->SetExitLanes(exit_lanes);
}

void Map::GetLanes(const Point2d &query_pt, const double &heading,
                   const double &dist, const double &heading_limit,
                   std::vector<LaneConstPtr> *const lanes) const {
  if (lane_map_.empty()) {
    LINFO("Map::GetLanes: no lane in map");
    return;
  }
  lanes->clear();
  double search_radius =
      std::max(FLAGS_ad_e2e_planning_map_minimum_boundary_search_radius, dist);

  const auto &boundaries_segment = FindLaneBoundarySegmentsInRadius(
      query_pt.x(), query_pt.y(), search_radius);
  if (boundaries_segment.empty()) {
    return;
  }

  std::unordered_set<std::string> candidata_lane_ids;
  for (const auto &boundary_segment : boundaries_segment) {
    const auto &boundary_id = boundary_segment.element_id;
    int32_t pt_idx = boundary_segment.segment_id.value();
    if (lane_boundary_map_.find(boundary_id) == lane_boundary_map_.end() ||
        !lane_boundary_map_.at(boundary_id)->IsValid()) {
      continue;
    }
    const auto boundary_ptr = lane_boundary_map_.at(boundary_id);
    if (pt_idx < 0 ||
        static_cast<int32_t>(boundary_ptr->curve_points().size()) <= pt_idx) {
      continue;
    }

    std::unordered_set<std::string> boundary_lanes;
    double l =
        GetMinLDistByPointIndex(query_pt, boundary_ptr->curve_points(), pt_idx);
    const auto &cmp = Double::Compare(l, 0.0);
    if (cmp == Double::CompareType::EQUAL) {
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else if (cmp == Double::CompareType::GREATER) {
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else {
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
    }
    for (const auto &lane_id : boundary_lanes) {
      if (lane_map_.find(lane_id) == lane_map_.end() ||
          !lane_map_.at(lane_id)->IsValid() ||
          (std::fabs(l) > dist && !lane_map_.at(lane_id)->IsOnLane(query_pt))) {
        continue;
      }

      SLPoint sl_pt;
      double lane_heading = 0.0;
      const auto &lane_ptr = lane_map_.at(lane_id);
      lane_ptr->GetSLWithoutLimit(query_pt, &sl_pt);
      lane_ptr->GetHeadingFromS(sl_pt.s, &lane_heading);
      if (std::fabs(ad_e2e::planning::math::AngleDiff(lane_heading, heading)) >
          heading_limit) {
        continue;
      }
      candidata_lane_ids.insert(lane_id);
    }
  }
  for (const auto &lane_id : candidata_lane_ids) {
    lanes->emplace_back(lane_map_.at(lane_id));
  }
}

void Map::GetLanes(const Point2d &query_pt, const double &heading,
                   const double &dist,
                   std::vector<LaneConstPtr> *const lanes) const {
  GetLanes(query_pt, heading, dist,
           FLAGS_ad_e2e_planning_map_search_heading_limit, lanes);
}

void Map::GetLanes(const Point2d &query_pt, const double &dist,
                   std::vector<LaneConstPtr> *const lanes) const {
  if (lane_map_.empty()) {
    LINFO("Map::GetLanes: no lane in map");
    return;
  }
  lanes->clear();
  double search_radius =
      std::max(FLAGS_ad_e2e_planning_map_minimum_boundary_search_radius, dist);

  const auto &boundaries_segment = FindLaneBoundarySegmentsInRadius(
      query_pt.x(), query_pt.y(), search_radius);
  if (boundaries_segment.empty()) {
    return;
  }

  std::unordered_set<std::string> candidata_lane_ids;
  for (const auto &boundary_segment : boundaries_segment) {
    const auto &boundary_id = boundary_segment.element_id;
    int32_t pt_idx = boundary_segment.segment_id.value();
    if (lane_boundary_map_.find(boundary_id) == lane_boundary_map_.end() ||
        !lane_boundary_map_.at(boundary_id)->IsValid()) {
      continue;
    }
    const auto boundary_ptr = lane_boundary_map_.at(boundary_id);
    if (pt_idx < 0 ||
        static_cast<int32_t>(boundary_ptr->curve_points().size()) <= pt_idx) {
      continue;
    }

    std::unordered_set<std::string> boundary_lanes;
    double l =
        GetMinLDistByPointIndex(query_pt, boundary_ptr->curve_points(), pt_idx);
    const auto &cmp = Double::Compare(l, 0.0);
    if (cmp == Double::CompareType::EQUAL) {
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else if (cmp == Double::CompareType::GREATER) {
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else {
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
    }
    for (const auto &lane_id : boundary_lanes) {
      if (lane_map_.find(lane_id) == lane_map_.end() ||
          !lane_map_.at(lane_id)->IsValid()) {
        continue;
      }
      const auto &lane_ptr = lane_map_.at(lane_id);
      SLPoint sl_point;
      lane_ptr->GetSLWithoutLimit(query_pt, &sl_point);
      double dist_to_lane = 0.0;
      if (sl_point.s < 0.0) {
        dist_to_lane =
            lane_ptr->center_line().points().front().DistanceTo(query_pt);
      } else if (sl_point.s > lane_ptr->curve_length()) {
        dist_to_lane =
            lane_ptr->center_line().points().back().DistanceTo(query_pt);
      } else {
        double lw = 0.0;
        double rw = 0.0;
        lane_ptr->GetWidthFromS(sl_point.s, &lw, &rw);
        dist_to_lane = std::max(sl_point.l - lw, rw - sl_point.l);
      }
      if (math::Double::Compare(dist_to_lane, dist) !=
          math::Double::CompareType::GREATER) {
        candidata_lane_ids.insert(lane_id);
      }
    }
  }
  for (const auto &lane_id : candidata_lane_ids) {
    lanes->emplace_back(lane_map_.at(lane_id));
  }
}

void Map::OnLanes(const Point2d &query_pt, const double &heading,
                  const double &heading_limit,
                  std::vector<LaneConstPtr> *const lanes) const {
  std::vector<LaneConstPtr> candidate_lanes;
  GetLanes(query_pt, heading, 0.0, heading_limit, &candidate_lanes);
  FilterPredecessorLanes(query_pt, candidate_lanes, lanes);
}

void Map::OnLanes(const Point2d &query_pt, const double &heading,
                  std::vector<LaneConstPtr> *const lanes) const {
  std::vector<LaneConstPtr> candidate_lanes;
  GetLanes(query_pt, heading, 0.0, &candidate_lanes);
  FilterPredecessorLanes(query_pt, candidate_lanes, lanes);
}

void Map::OnLanes(const Point2d &query_pt,
                  std::vector<LaneConstPtr> *const lanes) const {
  std::vector<LaneConstPtr> candidate_lanes;
  GetLanes(query_pt, 0.0, &candidate_lanes);
  FilterPredecessorLanes(query_pt, candidate_lanes, lanes);
}

double Map::GetMinLDistByPointIndex(const Point2d &query_pt,
                                    const std::vector<Point2d> &points,
                                    const int32_t &pt_idx) const {
  double l = 0.0;
  if (pt_idx == 0) {
    Point2d bound_vec = points.at(pt_idx + 1) - points.at(pt_idx);
    Point2d query_vec = query_pt - points.at(pt_idx);
    l = bound_vec.CrossProd(query_vec) /
        std::max(FLAGS_ad_e2e_planning_zero_threshold, bound_vec.Length());
  } else if (pt_idx + 1 == static_cast<int32_t>(points.size())) {
    Point2d bound_vec = points.at(pt_idx) - points.at(pt_idx - 1);
    Point2d query_vec = query_pt - points.at(pt_idx - 1);
    l = bound_vec.CrossProd(query_vec) /
        std::max(FLAGS_ad_e2e_planning_zero_threshold, bound_vec.Length());
  } else {
    Point2d prev_vec = points.at(pt_idx) - points.at(pt_idx - 1);
    Point2d next_vec = points.at(pt_idx + 1) - points.at(pt_idx);
    Point2d query_prev_vec = query_pt - points.at(pt_idx - 1);
    Point2d query_next_vec = query_pt - points.at(pt_idx);
    double prev_l =
        prev_vec.CrossProd(query_prev_vec) /
        std::max(FLAGS_ad_e2e_planning_zero_threshold, prev_vec.Length());
    double next_l =
        next_vec.CrossProd(query_next_vec) /
        std::max(FLAGS_ad_e2e_planning_zero_threshold, next_vec.Length());
    l = std::fabs(prev_l) > std::fabs(next_l) ? prev_l : next_l;
  }
  return l;
}

LaneBoundariesPtr Map::BuildLaneBoundaries(
    const std::vector<std::string> &lane_boundary_ids) {
  std::vector<LaneBoundaryConstPtr> boundaries;
  for (const auto &boundary_id : lane_boundary_ids) {
    if (lane_boundary_map_.find(boundary_id) != lane_boundary_map_.end() &&
        lane_boundary_map_.at(boundary_id)->IsValid()) {
      boundaries.emplace_back(lane_boundary_map_.at(boundary_id));
    }
  }
  return std::make_shared<LaneBoundaries>(boundaries);
}

RoadBoundariesPtr Map::BuildRoadBoundaries(
    const std::vector<std::string> &road_boundary_ids) {
  std::vector<RoadBoundaryConstPtr> boundaries;
  for (const auto &boundary_id : road_boundary_ids) {
    if (road_boundary_map_.find(boundary_id) != road_boundary_map_.end() &&
        road_boundary_map_.at(boundary_id)->IsValid()) {
      boundaries.emplace_back(road_boundary_map_.at(boundary_id));
    }
  }
  return std::make_shared<RoadBoundaries>(boundaries);
}

void Map::FilterPredecessorLanes(
    const Point2d &query_pt, const std::vector<LaneConstPtr> &candidate_lanes,
    std::vector<LaneConstPtr> *const lanes) const {
  std::vector<std::string> candidate_lane_ids;
  std::vector<std::string> prev_lane_ids;
  for (const auto &lane_ptr : candidate_lanes) {
    if (lane_ptr->IsOnLane(query_pt)) {
      candidate_lane_ids.emplace_back(lane_ptr->id());
      prev_lane_ids.insert(prev_lane_ids.end(),
                           lane_ptr->pre_lane_ids().begin(),
                           lane_ptr->pre_lane_ids().end());
    }
  }
  std::sort(candidate_lane_ids.begin(), candidate_lane_ids.end());
  std::sort(prev_lane_ids.begin(), prev_lane_ids.end());

  std::vector<std::string> result_lanes;
  std::set_difference(candidate_lane_ids.begin(), candidate_lane_ids.end(),
                      prev_lane_ids.begin(), prev_lane_ids.end(),
                      std::back_inserter(result_lanes));
  lanes->clear();
  for (const auto &lane_id : result_lanes) {
    lanes->emplace_back(lane_map_.at(lane_id));
  }
}

JunctionConstPtr Map::GetJunctionById(const std::string &id) const {
  if (junction_map_.find(id) == junction_map_.end()) return nullptr;
  return junction_map_.at(id);
}

TrafficLightInfoPtr Map::GetTrafficLightInfoById(const std::string &id) const {
  if (traffic_light_info_map_.find(id) == traffic_light_info_map_.end()) {
    return nullptr;
  }
  return traffic_light_info_map_.at(id);
}

void Map::GetJunctions(const Point2d &pt, const double &dist,
                       std::vector<JunctionConstPtr> *const junctions) const {
  junctions->clear();
  for (const auto &id_junction : junction_map_) {
    const auto &junction_ptr = id_junction.second;
    if (Double::Compare(junction_ptr->DistanceTo(pt), dist) !=
        Double::CompareType::GREATER) {
      junctions->emplace_back(junction_ptr);
    }
  }
}

CrosswalkConstPtr Map::GetCrosswalkById(const std::string &id) const {
  if (crosswalk_map_.find(id) == crosswalk_map_.end()) return nullptr;
  return crosswalk_map_.at(id);
}

RoadBoundaryConstPtr Map::GetRoadBoundaryById(const std::string &id) const {
  if (road_boundary_map_.find(id) == road_boundary_map_.end()) return nullptr;
  if (road_boundary_map_.at(id)->type().boundary_type ==
      ad_e2e::planning::BoundaryType::VIRTUAL)
    return nullptr;
  return road_boundary_map_.at(id);
}

LaneBoundaryConstPtr Map::GetLaneBoundaryById(const std::string &id) const {
  if (lane_boundary_map_.find(id) == lane_boundary_map_.end()) return nullptr;
  return lane_boundary_map_.at(id);
}
SectionConstPtr Map::GetSectionById(const std::string &id) const {
  if (section_map_.find(id) == section_map_.end()) return nullptr;
  return section_map_.at(id);
}

std::vector<std::string> Map::FindNearRoadBoundaryIds(
    const std::string &obs_id, const Vec2d &position, const double heading,
    double range, bool check_angle, double heading_thresh,
    int *left_or_right) const {
  std::vector<std::string> near_ids;
  const auto &boundaries_segment =
      FindLaneBoundarySegmentsInRadius(position.x(), position.y(), range);
  if (boundaries_segment.empty()) {
    return near_ids;
  }
  LDEBUG("query pos(%f, %f) range %f search %lu points", position.x(),
         position.y(), range, boundaries_segment.size());
  for (const auto &pt : boundaries_segment) {
    const auto &boundary_id = pt.element_id;
    if (lane_boundary_map_.find(boundary_id) == lane_boundary_map_.end()) {
      continue;
    }
    LDEBUG("Obstacle [%s] find near lane boundary %s", obs_id.c_str(),
           boundary_id.c_str());
    const auto boundary_ptr = lane_boundary_map_.at(boundary_id);
    std::vector<std::string> lane_ids;
    lane_ids.insert(lane_ids.end(), boundary_ptr->left_lanes().begin(),
                    boundary_ptr->left_lanes().end());
    lane_ids.insert(lane_ids.end(), boundary_ptr->right_lanes().begin(),
                    boundary_ptr->right_lanes().end());
    for (const auto &lane_id : lane_ids) {
      auto lane_ptr = GetLaneById(lane_id);
      if (lane_ptr == nullptr) {
        continue;
      }
      LDEBUG("Obstacle [%s] find near lane %s", obs_id.c_str(),
             lane_id.c_str());
      if (lane_ptr->left_road_boundary() != nullptr) {
        const auto &cur_boundarys =
            lane_ptr->left_road_boundary()->road_boundaries();
        for (const auto &cur_boundary : cur_boundarys)
          near_ids.push_back(cur_boundary->id());
      }
      if (lane_ptr->right_road_boundary() != nullptr) {
        const auto &cur_boundarys =
            lane_ptr->right_road_boundary()->road_boundaries();
        for (const auto &cur_boundary : cur_boundarys)
          near_ids.push_back(cur_boundary->id());
      }
    }
  }
  return near_ids;
}
namespace {

const absl::flat_hash_map<std::type_index, ad_e2e::planning::FeatureType>
    kGeoObjectTypeToFeatureType = {
        {typeid(ad_e2e::planning::LaneBoundaryPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_LANEBOUNDARY},
        {typeid(ad_e2e::planning::RoadBoundaryPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_ROADBOUNDARY},
        {typeid(ad_e2e::planning::LanePtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_LANE},
        {typeid(ad_e2e::planning::ClearAreaPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_CLEARAREA},
        {typeid(ad_e2e::planning::JunctionPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_JUNCTION},
        {typeid(ad_e2e::planning::CrosswalkPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_CROSSWALK},
        {typeid(ad_e2e::planning::StopLinePtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_STOPLINE},
        {typeid(ad_e2e::planning::SpeedBumpPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_SPEEDBUMP},
        {typeid(ad_e2e::planning::LaneBoundaryConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_LANEBOUNDARY},
        {typeid(ad_e2e::planning::RoadBoundaryConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_ROADBOUNDARY},
        {typeid(ad_e2e::planning::LaneConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_LANE},
        {typeid(ad_e2e::planning::ClearAreaConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_CLEARAREA},
        {typeid(ad_e2e::planning::JunctionConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_JUNCTION},
        {typeid(ad_e2e::planning::CrosswalkConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_CROSSWALK},
        {typeid(ad_e2e::planning::StopLineConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_STOPLINE},
        {typeid(ad_e2e::planning::SpeedBumpConstPtr),
         ad_e2e::planning::FeatureType::FEATURETYPE_SPEEDBUMP}};

template <typename Segment, typename Object, typename Tree>
const Segment *FindNearestSegment(const Tree &feature_aabox_tree,
                                  const Vec2d &geo_point) {
  const auto it =
      feature_aabox_tree.find(kGeoObjectTypeToFeatureType.at(typeid(Object)));
  if (it == feature_aabox_tree.end() ||
      it->second.Get()->aabox_tree == nullptr) {
    return nullptr;
  }

  const auto point = geo_point;
  return it->second.Get()->aabox_tree->GetNearestObject(point);
}

template <typename Segment, typename Object, typename Tree>
std::vector<const Segment *> FindSegmentsInRadius(
    const Tree &feature_aabox_tree, const Vec2d &geo_point, double radius) {
  const auto it =
      feature_aabox_tree.find(kGeoObjectTypeToFeatureType.at(typeid(Object)));
  if (it == feature_aabox_tree.end() ||
      it->second.Get()->aabox_tree == nullptr) {
    return {};
  }

  const auto point = geo_point;
  return it->second.Get()->aabox_tree->GetObjects(point, radius);
}

template <typename Segment, typename Object, typename Tree>
Object FindNearestObject(
    const Tree &feature_aabox_tree, const Vec2d &geo_point,
    const std::function<Object(e2e_noa::mapping::ElementId)> &find_object) {
  const auto *object =
      FindNearestSegment<Segment, Object, Tree>(feature_aabox_tree, geo_point);
  if (object == nullptr) {
    return nullptr;
  }

  return find_object(object->GetElementId());
}

template <typename ObjectSegment, typename Object, typename Tree>
absl::StatusOr<e2e_noa::mapping::v2::Segment> FindNearestObjectSegment(
    const Tree &feature_aabox_tree, const Vec2d &geo_point) {
  const auto *object_segment = FindNearestSegment<ObjectSegment, Object, Tree>(
      feature_aabox_tree, geo_point);
  if (object_segment == nullptr) {
    return absl::NotFoundError("Failed to find nearest segment.");
  }
  return e2e_noa::mapping::v2::Segment{
      .element_id = object_segment->GetElementId(),
      .segment_id = object_segment->GetSegmentId()};
}

template <typename Segment, typename Object, typename Tree>
std::vector<Object> FindObjectsInRadius(
    const Tree &feature_aabox_tree, const Vec2d &geo_point, double radius,
    const std::function<Object(e2e_noa::mapping::ElementId)> &find_object) {
  absl::flat_hash_set<e2e_noa::mapping::ElementId> object_ids_in_radius;
  for (const auto &object : FindSegmentsInRadius<Segment, Object, Tree>(
           feature_aabox_tree, geo_point, radius)) {
    object_ids_in_radius.insert(object->GetElementId());
  }

  std::vector<Object> objects_in_radius;
  for (const auto &object_id : object_ids_in_radius) {
    if (const auto obj = find_object(object_id); obj != nullptr) {
      objects_in_radius.push_back(obj);
    }
  }
  return objects_in_radius;
}

template <typename ObjectSegment, typename Object, typename Tree>
std::vector<e2e_noa::mapping::v2::Segment> FindObjectSegmentsInRadius(
    const Tree &feature_aabox_tree, const Vec2d &geo_point, double radius) {
  const auto segments = FindSegmentsInRadius<ObjectSegment, Object, Tree>(
      feature_aabox_tree, geo_point, radius);
  std::vector<e2e_noa::mapping::v2::Segment> segments_in_radius;
  segments_in_radius.reserve(segments.size());
  for (const auto &seg : segments) {
    segments_in_radius.push_back(
        {.element_id = seg->GetElementId(), .segment_id = seg->GetSegmentId()});
  }

  return segments_in_radius;
}

template <typename Container, typename Node, typename Tree>
void BuildAABoxNodes(
    const Container &elements,
    const std::function<bool(const typename Container::value_type &)>
        &filter_element,
    std::vector<Node> *nodes, Tree *tree) {
  if (elements.size() == 0) {
    return;
  }
  nodes->reserve(elements.size() * 3);
  for (const auto &element : elements) {
    if (filter_element(element) || !element.second) {
      continue;
    }
    const auto &segment_points = element.second->points();
    e2e_noa::mapping::SegmentId segment_id(0);
    if (segment_points.empty()) {
      continue;
    }
    for (auto first = segment_points.begin(), second = std::next(first);
         second != segment_points.end(); ++first, ++second, ++segment_id) {
      nodes->emplace_back(element.second->id(), segment_id, *first, *second);
    }
  }
  nodes->shrink_to_fit();
  *tree = std::make_unique<e2e_noa::AABoxKDTree2d<Node>>(
      *nodes, e2e_noa::AABoxKDTreeParams{.max_leaf_size = 4});
}

void FindSegmentsByMinDistanceToPoint(
    const Vec2d &smooth_point,
    const std::vector<e2e_noa::Segment2d> &smooth_segments,
    double *min_dist_square,
    std::vector<e2e_noa::Segment2d> *segments_with_min_dist) {
  bool is_inserted = false;
  for (const auto &smooth_segment : smooth_segments) {
    const auto length_square = smooth_segment.length_sqr();
    if (const auto max_dist = *min_dist_square + length_square +
                              2 * std::max(*min_dist_square, length_square);
        smooth_point.DistanceSquareTo(smooth_segment.start()) > max_dist) {
      continue;
    }
    const auto dist_square = smooth_segment.DistanceSquareTo(smooth_point);
    if (std::fabs(dist_square - *min_dist_square) < 1E-6) {
      if (!is_inserted) {
        segments_with_min_dist->push_back(smooth_segment);
        is_inserted = true;
      }
    } else if (dist_square < *min_dist_square) {
      segments_with_min_dist->clear();
      segments_with_min_dist->push_back(smooth_segment);
      is_inserted = true;
      *min_dist_square = dist_square;
    }
  }
}

double ComputeDistanceToSegments(
    const Vec2d &point, const std::vector<e2e_noa::Segment2d> &segments) {
  CHECK_GT(segments.size(), 0);

  const double dist_to_curb = segments[0].DistanceTo(point);

  if (segments.size() == 2) {
    const auto reverse = segments[1].end() == segments[0].start();
    const auto &seg_0 = reverse ? segments[1] : segments[0];
    const auto &seg_1 = reverse ? segments[0] : segments[1];

    if (seg_0.DistanceTo(seg_1) < 1E-3 &&
        std::fabs(seg_0.unit_direction().CrossProd(seg_1.unit_direction())) <
            1E-2 &&
        seg_0.unit_direction().Dot(seg_1.unit_direction()) < 0) {
      return dist_to_curb;
    }

    if (seg_0.end() == seg_1.start()) {
      const auto counterclockwise = seg_0.ProductOntoUnit(seg_1.end()) >= 0.0;
      const auto sign = counterclockwise
                            ? (seg_0.ProductOntoUnit(point) >= 0.0 &&
                               seg_1.ProductOntoUnit(point) >= 0.0)
                            : (seg_0.ProductOntoUnit(point) > 0.0 ||
                               seg_1.ProductOntoUnit(point) > 0.0);
      return sign ? -dist_to_curb : dist_to_curb;
    }

    return seg_0.ProductOntoUnit(point) >= 0.0 ? -dist_to_curb : dist_to_curb;
  }

  return segments[0].ProductOntoUnit(point) >= 0.0 ? -dist_to_curb
                                                   : dist_to_curb;
}

}  // namespace

void Map::InitAABoxKDTree2d() {
  feature_aabox_tree_.reserve(kGeoObjectTypeToFeatureType.size() / 2);

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_LANE] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            lane_map(),
            [](const std::pair<std::string, ad_e2e::planning::LanePtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_LANEBOUNDARY] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            lane_boundary_map(),
            [](const std::pair<std::string, ad_e2e::planning::LaneBoundaryPtr>
                   &) { return false; },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_ROADBOUNDARY] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            road_boundary_map(),
            [](const std::pair<std::string, ad_e2e::planning::RoadBoundaryPtr>
                   &) { return false; },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_JUNCTION] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            junction_map(),
            [](const std::pair<std::string, ad_e2e::planning::JunctionPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_CROSSWALK] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            crosswalk_map(),
            [](const std::pair<std::string, ad_e2e::planning::CrosswalkPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_SPEEDBUMP] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            speed_bump_map(),
            [](const std::pair<std::string, ad_e2e::planning::SpeedBumpPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_STOPLINE] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            stop_line_map(),
            [](const std::pair<std::string, ad_e2e::planning::StopLinePtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_e2e::planning::FeatureType::FEATURETYPE_CLEARAREA] =
      e2e_noa::DecisionExplorationFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            clear_area_map(),
            [](const std::pair<std::string, ad_e2e::planning::ClearAreaPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });
}

absl::StatusOr<e2e_noa::mapping::v2::Segment> Map::FindNearestLaneSegment(
    double lon, double lat) const {
  return FindNearestObjectSegment<AABoxNode, ad_e2e::planning::LanePtr>(
      feature_aabox_tree_, {lon, lat});
}

absl::StatusOr<e2e_noa::mapping::v2::Segment>
Map::FindNearestRoadBoundarySegment(double lon, double lat) const {
  return FindNearestObjectSegment<AABoxNode, ad_e2e::planning::RoadBoundaryPtr>(
      feature_aabox_tree_, {lon, lat});
  ;
}

absl::StatusOr<e2e_noa::mapping::v2::Segment>
Map::FindNearestLaneBoundarySegment(double lon, double lat) const {
  return FindNearestObjectSegment<AABoxNode, ad_e2e::planning::LaneBoundaryPtr>(
      feature_aabox_tree_, {lon, lat});
}

ad_e2e::planning::LaneConstPtr Map::FindNearestLane(double lon,
                                                    double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::LaneConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetLaneById, this,
                std::placeholders::_1));
}

ad_e2e::planning::LaneBoundaryConstPtr Map::FindNearestLaneBoundary(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::LaneBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetLaneBoundaryById, this,
                std::placeholders::_1));
}

ad_e2e::planning::RoadBoundaryConstPtr Map::FindNearestRoadBoundary(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::RoadBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetRoadBoundaryById, this,
                std::placeholders::_1));
}
ad_e2e::planning::StopLineConstPtr Map::FindNearestStopLine(double lon,
                                                            double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::StopLineConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetStopLineById, this,
                std::placeholders::_1));
}
ad_e2e::planning::JunctionConstPtr Map::FindNearestJunction(double lon,
                                                            double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::JunctionConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetJunctionById, this,
                std::placeholders::_1));
}
ad_e2e::planning::CrosswalkConstPtr Map::FindNearestCrosswalk(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::CrosswalkConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetCrosswalkById, this,
                std::placeholders::_1));
}
ad_e2e::planning::SpeedBumpConstPtr Map::FindNearestSpeedBump(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::SpeedBumpConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetSpeedBumpById, this,
                std::placeholders::_1));
}
ad_e2e::planning::ClearAreaConstPtr Map::FindNearestClearArea(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_e2e::planning::ClearAreaConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_e2e::planning::Map::GetClearAreaById, this,
                std::placeholders::_1));
}

std::vector<e2e_noa::mapping::v2::Segment> Map::FindLaneSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_e2e::planning::LanePtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}

std::vector<e2e_noa::mapping::v2::Segment>
Map::FindLaneBoundarySegmentsInRadius(double lon, double lat,
                                      double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode,
                                    ad_e2e::planning::LaneBoundaryPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<e2e_noa::mapping::v2::Segment>
Map::FindRoadBoundarySegmentsInRadius(double lon, double lat,
                                      double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode,
                                    ad_e2e::planning::RoadBoundaryPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<e2e_noa::mapping::v2::Segment> Map::FindStopLineSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_e2e::planning::StopLinePtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<e2e_noa::mapping::v2::Segment> Map::FindJunctionSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_e2e::planning::JunctionPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<e2e_noa::mapping::v2::Segment> Map::FindCrosswalkSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_e2e::planning::CrosswalkPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<e2e_noa::mapping::v2::Segment> Map::FindSpeedBumpSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_e2e::planning::SpeedBumpPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<e2e_noa::mapping::v2::Segment> Map::FindClearAreaSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_e2e::planning::ClearAreaPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}

std::vector<ad_e2e::planning::LaneConstPtr> Map::FindLanesInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::LaneConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetLaneById, this,
                std::placeholders::_1));
}

std::vector<ad_e2e::planning::LaneBoundaryConstPtr>
Map::FindLaneBoundariesInRadius(double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::LaneBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetLaneBoundaryById, this,
                std::placeholders::_1));
}
std::vector<ad_e2e::planning::RoadBoundaryConstPtr>
Map::FindRoadBoundariesInRadius(double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::RoadBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetRoadBoundaryById, this,
                std::placeholders::_1));
}
std::vector<ad_e2e::planning::StopLineConstPtr> Map::FindStopLinesInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::StopLineConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetStopLineById, this,
                std::placeholders::_1));
}
std::vector<ad_e2e::planning::JunctionConstPtr> Map::FindJunctionsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::JunctionConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetJunctionById, this,
                std::placeholders::_1));
}
std::vector<ad_e2e::planning::CrosswalkConstPtr> Map::FindCrosswalksInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::CrosswalkConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetCrosswalkById, this,
                std::placeholders::_1));
}
std::vector<ad_e2e::planning::SpeedBumpConstPtr> Map::FindSpeedBumpsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::SpeedBumpConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetSpeedBumpById, this,
                std::placeholders::_1));
}
std::vector<ad_e2e::planning::ClearAreaConstPtr> Map::FindClearAreasInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_e2e::planning::ClearAreaConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_e2e::planning::Map::GetClearAreaById, this,
                std::placeholders::_1));
}

void Map::FindAllLaneSequence(
    std::vector<LaneConstPtr> &lanes,
    std::vector<std::vector<LaneConstPtr>> &lane_sequences) const {
  if (lanes.empty()) {
    return;
  }
  auto next_lanes = GetNextLanes(lanes.back());
  if (next_lanes.empty()) {
    lane_sequences.emplace_back(lanes);
  } else {
    for (auto &next : next_lanes) {
      bool flag = false;
      for (auto &exist_lane : lanes) {
        if (next->id() == exist_lane->id() ||
            (!next->section_id().empty() &&
             next->section_id() == exist_lane->section_id())) {
          flag = true;
        }
      }
      if (flag || next->id().find("nextvr") != next->id().npos) {
        lane_sequences.emplace_back(lanes);
        continue;
      }
      lanes.emplace_back(next);
      FindAllLaneSequence(lanes, lane_sequences);
      lanes.pop_back();
    }
  }
}

void Map::FindAllLaneSequence(
    std::vector<LaneConstPtr> &lanes, const double &length_limit,
    std::vector<std::vector<LaneConstPtr>> &lane_sequences) const {
  if (lanes.empty()) {
    return;
  }
  auto next_lanes_ = GetNextLanes(lanes.back());
  std::vector<LaneConstPtr> next_lanes;
  for (auto lane : next_lanes_) {
    if (lane->is_navigation()) {
      next_lanes.emplace_back(lane);
    }
  }
  if (next_lanes.empty()) {
    next_lanes = next_lanes_;
  }
  if (next_lanes.empty()) {
    lane_sequences.emplace_back(lanes);
  } else {
    for (auto &next : next_lanes) {
      double total_length = Constants::ZERO;
      bool flag = false;
      total_length += next->topo_length();
      if (total_length >= length_limit) {
        flag = true;
      }
      if (flag) {
        lanes.emplace_back(next);
        lane_sequences.emplace_back(lanes);
        lanes.pop_back();
        continue;
      }
      lanes.emplace_back(next);
      FindAllLaneSequence(lanes, length_limit - total_length, lane_sequences);
      lanes.pop_back();
    }
  }
}

void Map::GetAllLaneSequences(
    const LaneConstPtr &lane,
    std::vector<std::vector<LaneConstPtr>> &sequences) const {
  if (!lane) return;
  std::vector<LaneConstPtr> lane_ptr_vec;
  std::vector<std::vector<LaneConstPtr>> candidate_sequences;
  lane_ptr_vec.emplace_back(lane);
  FindAllLaneSequence(lane_ptr_vec, candidate_sequences);
  for (auto &candidate_sequence : candidate_sequences) {
    sequences.emplace_back(std::move(candidate_sequence));
  }
}

void Map::GetAllLaneSequences(
    const LaneConstPtr &lane, const double &length,
    std::vector<std::vector<LaneConstPtr>> &sequences) const {
  if (!lane) return;
  std::vector<LaneConstPtr> lanes;
  std::vector<std::vector<LaneConstPtr>> candidate_sequences;
  lanes.emplace_back(lane);
  FindAllLaneSequence(lanes, length, candidate_sequences);
  for (auto &candidate_sequence : candidate_sequences) {
    sequences.emplace_back(std::move(candidate_sequence));
  }
}

CompositeTurnType Map::CheckCompositeLane(const LanePtr &lane) const {
  if (!lane || lane->next_lane_ids().size() < 2) {
    return CompositeTurnType::NORMAL_TURN;
  }
  bool left_turn_check = false;
  bool right_turn_check = false;
  bool no_turn_check = false;
  for (const auto &lane_id : lane->next_lane_ids()) {
    const auto &lane = GetLaneById(lane_id);
    if (!lane || lane->junction_id().empty()) {
      continue;
    }
    switch (lane->turn_type()) {
      case LEFT_TURN:
      case U_TURN:
        left_turn_check = true;
        break;
      case RIGHT_TURN:
        right_turn_check = true;
        break;
      case NO_TURN:
        no_turn_check = true;
        break;
      default:
        break;
    }
  }
  if (left_turn_check && no_turn_check && right_turn_check) {
    return CompositeTurnType::LEFT_STRAIGHT_RIGHT;
  }
  if (left_turn_check && no_turn_check) {
    return CompositeTurnType::LEFT_STRAIGHT;
  }
  if (no_turn_check && right_turn_check) {
    return CompositeTurnType::STRAIGHT_RIGHT;
  }
  if (left_turn_check && right_turn_check) {
    return CompositeTurnType::LEFT_RIGHT;
  }
  return CompositeTurnType::NORMAL_TURN;
}

}  // namespace planning
}  // namespace ad_e2e
