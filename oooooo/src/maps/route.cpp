#include "maps/route.h"

#include <limits>

#include "maps/map.h"
#include "maps/semantic_map_util.h"

namespace ad_e2e {
namespace planning {

void Route::UpdateNaviPriorityLanes(const LaneConstPtr &ego_lane) {
  if (!ego_lane || !ego_lane->is_navigation() || route_info_.sections.empty()) {
    LOG(ERROR) << "[route] cannot update navi priority lanes!";
    return;
  }

  SectionInfo valid_back_section;
  for (auto it = route_info_.sections.rbegin();
       it != route_info_.sections.rend(); ++it) {
    const auto &section = *it;
    if (map_->GetSectionById(section.id) != nullptr) {
      for (const auto &lane_id : section.lane_ids) {
        const auto &final_lane = map_->GetLaneById(lane_id);
        if (final_lane) {
          valid_back_section = section;
          break;
        }
      }
      if (!valid_back_section.lane_ids.empty()) break;
    }
  }
  if (valid_back_section.lane_ids.empty()) {
    LOG(ERROR) << "[UpdateNaviPriorityLanes] no valid_back_section!";
    return;
  }

  std::vector<std::vector<LANE>> lane_sequences_candidates;
  for (const auto &lane_id : valid_back_section.lane_ids) {
    const auto &final_lane = map_->GetLaneById(lane_id);
    if (!final_lane) continue;
    std::vector<LANE> lane_sequence;
    lane_sequence.emplace_back(LANE(final_lane));
    FindReverseLaneSequence(lane_sequence, lane_sequences_candidates,
                            ego_lane->section_id());
  }
  if (lane_sequences_candidates.empty()) {
    LOG(ERROR) << "[UpdateNaviPriorityLanes] can not find navi priority lanes!";
    return;
  }
  SectionInfo section_info;
  GetSectionById(ego_lane->section_id(), section_info);
  const auto it_ego = std::find(section_info.lane_ids.begin(),
                                section_info.lane_ids.end(), ego_lane->id());
  for (auto &lanes_candidate : lane_sequences_candidates) {
    const auto it_last =
        std::find(section_info.lane_ids.begin(), section_info.lane_ids.end(),
                  lanes_candidate.back().lane->id());
    lanes_candidate.back().lane_change =
        std::abs(std::distance(it_ego, it_last));
  }

  std::map<double, std::vector<LANE>> cost_map;
  for (const auto &lanes_candidate : lane_sequences_candidates) {
    double cost = CalculateSequenceCost(lanes_candidate);
    cost_map[cost] = lanes_candidate;
  }

  for (const auto &lane : cost_map.begin()->second) {
    UpdateNaviPriorityLaneId(lane.lane);
  }
}

bool Route::GetSectionById(const std::string &section_id,
                           SectionInfo &section) const {
  if (route_info_.sections.empty()) {
    return false;
  }
  auto equal = [&](const SectionInfo &sec) { return sec.id == section_id; };
  auto it = std::find_if(route_info_.sections.begin(),
                         route_info_.sections.end(), equal);
  if (it == route_info_.sections.end()) {
    return false;
  } else {
    section = *it;
    return true;
  }
}

bool Route::CanDriveToRouteEnd(const LaneSequencePtr &lane_seq) const {
  if (route_info_.sections.empty()) {
    return false;
  }
  if (!lane_seq || lane_seq->lanes().empty()) {
    return false;
  }
  const auto &end_section = route_info_.sections.back();
  const auto &end_lane = std::find_if(
      lane_seq->lanes().rbegin(), lane_seq->lanes().rend(),
      [](const LaneConstPtr &lane) { return lane->is_navigation(); });
  if (end_lane == lane_seq->lanes().rend()) {
    return false;
  }
  for (const auto &lane_id : end_section.lane_ids) {
    if (lane_id == (*end_lane)->id()) {
      return true;
    }
  }
  return false;
}

LaneConstPtr Route::GetNaviPriorityLane(const std::string &section_id) const {
  SectionInfo section_info;
  if (!GetSectionById(section_id, section_info)) {
    return nullptr;
  }
  if (section_info.navi_priority_lane_id.empty()) {
    LOG(ERROR) << "[route] cannot get navi_priority_lane in section ["
               << section_id << "]!";
    return nullptr;
  }
  return map_->GetLaneById(section_info.navi_priority_lane_id);
}

int Route::GetPriorityLaneRelation(const LaneConstPtr &lane) const {
  int d = 0;
  SectionInfo section;
  if (!lane || !GetSectionById(lane->section_id(), section)) {
    return d;
  }
  auto it0 = std::find(section.lane_ids.begin(), section.lane_ids.end(),
                       section.navi_priority_lane_id);
  auto it1 =
      std::find(section.lane_ids.begin(), section.lane_ids.end(), lane->id());
  if (it0 != section.lane_ids.end() && it1 != section.lane_ids.end()) {
    d = static_cast<int>(std::distance(it0, it1));
  }
  return d;
}

double Route::GetDistanceToX(const XRoadType &x_road_type, const double &x,
                             const double &y) const {
  double dist_to_x = std::numeric_limits<double>::max();
  if (!map_ || !map_->IsValid()) return dist_to_x;
  const auto &nearest_lane = map_->GetNearestLane(
      Point2d(x, y), 0.0, Constants::MIN_HALF_LANE_WIDTH * 2.5);
  if (!nearest_lane || route_info_.sections.empty() ||
      nearest_lane->section_id().empty()) {
    return dist_to_x;
  }
  double start_lane_s = 0.0;
  math::Vec2d p(x, y);
  double s, l;
  if (nearest_lane->center_line().GetProjection(p, &s, &l)) {
    start_lane_s += s;
    start_lane_s = nearest_lane->center_line().length() - start_lane_s;
    dist_to_x = 0.0;
  } else {
    return dist_to_x;
  }
  auto equal_to = [&](const SectionInfo &sec) {
    return sec.id == nearest_lane->section_id();
  };
  auto it = std::find_if(route_info_.sections.begin(),
                         route_info_.sections.end(), equal_to);
  while (it != route_info_.sections.end()) {
    if (x_road_type == 1 && it->none_odd_type >= 1) {
      break;
    }
    dist_to_x += it->length;
    it++;
  }
  if (it == route_info_.sections.end())
    dist_to_x = std::numeric_limits<double>::max();
  return dist_to_x;
}

double Route::CalculateSequenceCost(const std::vector<LANE> &lanes) {
  double total_cost = 0.0;
  if (lanes.empty()) {
    return total_cost;
  }
  const double lane_change_weight = 10.0;
  const double topology_weight = 1.0;
  for (const auto &lane : lanes) {
    if (lane.lane->is_merge() || lane.lane->is_split()) {
      total_cost += topology_weight * 1.0;
    }

    total_cost += lane_change_weight * double(lane.lane_change);
  }
  return total_cost;
}

void Route::UpdateNaviPriorityLaneId(const LaneConstPtr &lane) {
  if (!lane || route_info_.sections.empty()) return;
  auto equal_to = [&](const SectionInfo &sec) {
    return sec.id == lane->section_id();
  };
  auto it = std::find_if(route_info_.sections.begin(),
                         route_info_.sections.end(), equal_to);
  if (it != route_info_.sections.end()) {
    it->navi_priority_lane_id = lane->id();
  }
}
void Route::FindReverseLaneSequence(
    std::vector<LANE> &lane_sequence,
    std::vector<std::vector<LANE>> &lane_sequences,
    const std::string &stop_section_id) {
  if (lane_sequence.empty() || !lane_sequence.back().lane->is_navigation()) {
    return;
  }
  auto &current = lane_sequence.back();
  if (current.lane->section_id() == stop_section_id) {
    lane_sequences.emplace_back(lane_sequence);
  } else {
    std::vector<LaneConstPtr> pre_lanes;
    current.lane_change = GetPreNaviLanes(current.lane, pre_lanes);
    if (pre_lanes.empty()) {
      return;
    }
    for (auto &pre : pre_lanes) {
      auto equal_to = [&](const LANE &lane) {
        return lane.lane->id() == pre->id();
      };
      auto it =
          std::find_if(lane_sequence.begin(), lane_sequence.end(), equal_to);
      if (it != lane_sequence.end()) return;

      lane_sequence.emplace_back(LANE(pre));
      FindReverseLaneSequence(lane_sequence, lane_sequences, stop_section_id);
      lane_sequence.pop_back();
    }
  }
}
size_t Route::GetPreNaviLanes(const LaneConstPtr &current_lane,
                              std::vector<LaneConstPtr> &pre_lanes) {
  pre_lanes.clear();
  SectionInfo section_info;
  if (!GetSectionById(current_lane->section_id(), section_info)) {
    return 0;
  }
  auto it = std::find(section_info.lane_ids.begin(),
                      section_info.lane_ids.end(), current_lane->id());
  if (it == section_info.lane_ids.end()) {
    return 0;
  }
  auto idx = std::distance(section_info.lane_ids.begin(), it);
  for (auto i = idx; i >= 0; --i) {
    auto lane = map_->GetLaneById(section_info.lane_ids[i]);
    if (!lane) continue;

    std::vector<LaneConstPtr> temp_lanes;
    map_->GetPrecedeLanes(lane, &temp_lanes, true);
    for (auto &pre_lane : temp_lanes) {
      if (LaneType::LANE_BUS_NORMAL == pre_lane->type()) continue;
      pre_lanes.emplace_back(pre_lane);
    }

    pre_lanes.insert(pre_lanes.end(),
                     pre_nextvr_lanes_[section_info.lane_ids[i]].begin(),
                     pre_nextvr_lanes_[section_info.lane_ids[i]].end());
    if (!pre_lanes.empty()) {
      return idx - i;
    }
  }
  for (auto i = idx + 1; i < static_cast<int>(section_info.lane_ids.size());
       ++i) {
    auto lane = map_->GetLaneById(section_info.lane_ids[i]);
    if (!lane) continue;

    std::vector<LaneConstPtr> temp_lanes;
    map_->GetPrecedeLanes(lane, &temp_lanes, true);
    for (auto &pre_lane : temp_lanes) {
      if (LaneType::LANE_BUS_NORMAL == pre_lane->type()) continue;
      pre_lanes.emplace_back(pre_lane);
    }

    pre_lanes.insert(pre_lanes.end(),
                     pre_nextvr_lanes_[section_info.lane_ids[i]].begin(),
                     pre_nextvr_lanes_[section_info.lane_ids[i]].end());
    if (!pre_lanes.empty()) {
      return i - idx;
    }
  }
  pre_lanes.clear();
  return 0;
}

double Route::GetDistanceToNaviEnd() const {
  double dis_to_navi_end = DBL_MAX;
  if (route_info_.navi_end.section_id.empty() ||
      route_info_.navi_start.section_id.empty()) {
    return dis_to_navi_end;
  }
  if (!map_ || !map_->IsValid()) return dis_to_navi_end;
  bool find_navi_start = false;
  bool find_navi_end = false;
  for (std::size_t index = 0; index < route_info_.sections.size(); index++) {
    if (route_info_.sections[index].id.empty()) break;
    if (route_info_.sections[index].id == route_info_.navi_start.section_id) {
      dis_to_navi_end = 0.0;
      find_navi_start = true;
    }
    if (route_info_.sections[index].id == route_info_.navi_end.section_id) {
      find_navi_end = true;
      break;
    }
    if (find_navi_start) {
      dis_to_navi_end += route_info_.sections[index].length;
    }
  }

  return find_navi_end ? dis_to_navi_end - route_info_.navi_start.s_offset +
                             route_info_.navi_end.s_offset
                       : DBL_MAX;
}
}  // namespace planning
}  // namespace ad_e2e
