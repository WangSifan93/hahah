#include "maps/lane_sequence.h"

#include <cctype>
namespace ad_e2e {
namespace planning {
LaneSequence::LaneSequence(const std::vector<LaneConstPtr> &lanes_ptr_vec) {
  for (const auto &lane : lanes_ptr_vec) {
    if (!lane) break;
    if (lanes_.empty() || lane->id() != lanes_.back()->id()) {
      lanes_.emplace_back(lane);
    }
  }
}

LaneConstPtr LaneSequence::front_lane() const {
  if (!lanes_.empty()) {
    return lanes_.front();
  }
  return nullptr;
}

bool LaneSequence::IsValid() {
  if (!lanes_.empty() && lanes_.front()) {
    return lanes_.front()->center_line().IsValid();
  }
  return false;
}

bool LaneSequence::SamplePoints(double start_s,
                                std::vector<Point2d> *const points,
                                double interval) const {
  double lane_start_s = start_s;
  for (const auto &lane : lanes_) {
    if (!lane->center_line().IsValid()) break;
    std::vector<ad_e2e::planning::math::Vec2d> sample_points;
    bool flag = lane->center_line().SamplePoints(
        lane_start_s, lane->center_line().length(), &sample_points, interval);
    if (flag) {
      lane_start_s = lane_start_s + (double)sample_points.size() * interval -
                     lane->center_line().length();
      points->insert(points->end(), sample_points.begin(), sample_points.end());
    }
  }
  return true;
}

bool LaneSequence::AddLane(const LaneConstPtr &lane) {
  if (!lane) {
    return false;
  }
  if (lanes_.empty()) {
    lanes_.emplace_back(lane);
    return true;
  } else if ((!lane->center_line().IsValid() ||
              !lanes_.back()->center_line().IsValid()) &&
             lanes_.back()->IsNext(lane->id())) {
    lanes_.emplace_back(lane);
    return true;
  } else if (std::fabs(lanes_.back()->center_line().end_point().x() -
                       lane->center_line().begin_point().x()) < 0.1 &&
             std::fabs(lanes_.back()->center_line().end_point().y() -
                       lane->center_line().begin_point().y()) < 0.1) {
    lanes_.emplace_back(lane);
    return true;
  }
  return false;
}

double LaneSequence::GetTrueLength() const {
  double len = 0;
  for (auto &lane : lanes_) {
    len += lane->topo_length();
  }
  return len;
}

double LaneSequence::GetPointsLength() const {
  double len = 0;
  for (auto &lane : lanes_) {
    len += lane->curve_length();
  }
  return len;
}

math::Vec2d LaneSequence::GetPointAtS(double s) const {
  math::Vec2d point_s;
  double accumulate_s = 0.0;
  for (const auto &lane : lanes_) {
    if (!lane->center_line().IsValid()) break;
    if (accumulate_s + lane->center_line().length() > s ||
        lane == lanes_.back()) {
      point_s = lane->center_line().GetPointAtS(s - accumulate_s);
      break;
    }
    accumulate_s += lane->center_line().length();
  }
  return point_s;
}

double LaneSequence::GetProjectionDistance(const double &x,
                                           const double &y) const {
  return GetProjectionDistance(math::Vec2d(x, y), nullptr);
}

double LaneSequence::GetProjectionDistance(const math::Vec2d &point) const {
  return GetProjectionDistance(point, nullptr);
}

double LaneSequence::GetProjectionDistance(const math::Vec2d &point,
                                           double *s_offset,
                                           double *l_offset) const {
  double project_l = std::numeric_limits<double>::max();
  const auto nearest_lane = GetNearestLane(point, nullptr);
  if (!nearest_lane) {
    return project_l;
  }
  double project_s = std::numeric_limits<double>::max();
  nearest_lane->center_line().GetProjection(point, &project_s, &project_l);
  for (const auto &lane : lanes_) {
    if (lane && lane == nearest_lane) break;
    project_s += lane->center_line().length();
  }
  if (s_offset) {
    *s_offset = project_s;
  }
  if (l_offset) {
    *l_offset = project_l;
  }
  return std::fabs(project_l);
}
bool IsNumeric(const std::string &str) {
  for (char c : str) {
    if (!std::isdigit(c)) {
      return false;
    }
  }
  return true;
}

double LaneSequence::GetDistanceToPOI(
    const PoiType &poi_type, const double &x, const double &y,
    LaneConstPtr &poi_lane,
    const ad_e2e::planning::NaviPosition *navi_start) const {
  double dist_to_poi = std::numeric_limits<double>::max();
  int start_idx = -1;
  double project_dis;
  LaneConstPtr start_lane =
      GetNearestLane(ad_e2e::planning::math::Vec2d(x, y), &project_dis);
  if (project_dis > 3.5 * Constants::MIN_HALF_LANE_WIDTH || !start_lane) {
    return dist_to_poi;
  }
  for (const auto &lane : lanes_) {
    start_idx++;
    if (lane && lane->id() == start_lane->id()) {
      start_lane = lane;
      break;
    }
  }

  if (!start_lane) {
    return dist_to_poi;
  }

  double s = 0.0, l = 0.0;
  start_lane->center_line().GetProjection({x, y}, &s, &l);

  if (poi_type == PoiType::Poi_NaviEnd) {
    if (!navi_start || navi_start->section_id.empty()) {
      return dist_to_poi;
    }
    dist_to_poi = 0.0;
    bool find_navi_start = false;
    bool find_navigation = false;
    for (const auto &lane : lanes_) {
      if (find_navigation && !lane->is_navigation()) {
        break;
      }
      if (lane->is_navigation()) {
        find_navigation = true;
        poi_lane = lane;
        dist_to_poi += lane->topo_length();
      }
      if (lane->section_id() == navi_start->section_id) {
        find_navi_start = true;
        dist_to_poi = lane->topo_length();
      }
    }
    if (!find_navigation) {
      return std::numeric_limits<double>::max();
    }
    if (find_navi_start) {
      dist_to_poi -= navi_start->s_offset;
    }
    return dist_to_poi;
  }

  if (poi_type == PoiType::Poi_Split) {
    if (lanes_[start_idx]->is_split()) {
      poi_lane = lanes_[start_idx];
      return 0.0;
    }
    dist_to_poi = start_lane->center_line().length() - s;
    ++start_idx;
    while (start_idx < static_cast<int>(lanes_.size()) &&
           !lanes_[start_idx]->is_split()) {
      dist_to_poi += lanes_[start_idx]->topo_length();
      ++start_idx;
    }
    if (start_idx == static_cast<int>(lanes_.size())) {
      dist_to_poi = std::numeric_limits<double>::max();
      poi_lane = nullptr;
    } else {
      poi_lane = lanes_[start_idx];
    }
    return dist_to_poi;
  }

  if (poi_type == PoiType::Poi_Merge) {
    dist_to_poi = start_lane->center_line().length() - s;
    if (lanes_[start_idx]->is_merge()) {
      poi_lane = lanes_[start_idx];
      return dist_to_poi;
    }
    ++start_idx;
    while (start_idx < static_cast<int>(lanes_.size()) &&
           !lanes_[start_idx]->is_merge()) {
      dist_to_poi += lanes_[start_idx]->topo_length();
      ++start_idx;
    }
    if (start_idx == static_cast<int>(lanes_.size())) {
      dist_to_poi = std::numeric_limits<double>::max();
      poi_lane = nullptr;
    } else {
      dist_to_poi += lanes_[start_idx]->topo_length();
      poi_lane = lanes_[start_idx];
    }
    return dist_to_poi;
  }

  if (poi_type == PoiType::Poi_To_Be_Merged) {
    dist_to_poi = start_lane->center_line().length() - s;
    if (lanes_[start_idx]->merge_topology() == TOPOLOGY_TO_BE_MERGED) {
      poi_lane = lanes_[start_idx];
      return dist_to_poi;
    }
    ++start_idx;
    while (start_idx < static_cast<int>(lanes_.size()) &&
           lanes_[start_idx]->merge_topology() != TOPOLOGY_TO_BE_MERGED) {
      dist_to_poi += lanes_[start_idx]->topo_length();
      ++start_idx;
    }
    if (start_idx == static_cast<int>(lanes_.size())) {
      dist_to_poi = std::numeric_limits<double>::max();
      poi_lane = nullptr;
    } else {
      dist_to_poi += lanes_[start_idx]->topo_length();
      poi_lane = lanes_[start_idx];
    }
    return dist_to_poi;
  }
  return dist_to_poi;
}

double LaneSequence::GetDistanceToPOI(const PoiType &poi_type, const double &x,
                                      const double &y) const {
  LaneConstPtr poi_lane;
  return GetDistanceToPOI(poi_type, x, y, poi_lane, nullptr);
}

LaneConstPtr LaneSequence::GetNearestLane(const math::Vec2d &point,
                                          double *dis) const {
  LaneConstPtr nearest_lane = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto &lane : lanes_) {
    if (!lane->center_line().IsValid()) continue;
    double d = lane->center_line().GetDistance(point);
    if (d < min_dist) {
      min_dist = d;
      nearest_lane = lane;
    }
  }
  if (dis) {
    *dis = min_dist;
  }
  return nearest_lane;
}

void LaneSequence::GetBoundaryTypeFromS(
    const bool &is_left, const double &s,
    std::vector<LaneBoundaryType> &boundary_type) const {
  LaneConstPtr nearest_lane = nullptr;
  double accumulate_s = 0.0;
  for (const auto &lane : lanes_) {
    if (!lane) break;
    accumulate_s += lane->curve_length();
    if (accumulate_s > s) {
      nearest_lane = lane;
      break;
    }
  }
  if (!nearest_lane) return;
  auto pt = nearest_lane->center_line().GetPointAtS(
      s + nearest_lane->curve_length() - accumulate_s);
  GetBoundaryTypeFromPoint(is_left, pt, boundary_type);
}

void LaneSequence::GetBoundaryTypeFromPoint(
    const bool &is_left, const math::Vec2d &point,
    std::vector<LaneBoundaryType> &boundary_type) const {
  auto nearest_lane = GetNearestLane(point, nullptr);
  if (!nearest_lane) return;
  nearest_lane->forward_boundary_type(is_left, point, boundary_type);
  if (boundary_type.empty()) return;
  bool find_nearest = false;
  for (const auto &lane : lanes_) {
    if (!lane) break;
    const auto &boundary_ptr =
        is_left ? lane->left_boundary() : lane->right_boundary();
    if (!boundary_ptr) break;
    const double &start_s = boundary_type.back().s;
    if (find_nearest) {
      const auto &all_boundary_type = boundary_ptr->boundary_types();
      for (const auto &boundary : all_boundary_type) {
        boundary_type.emplace_back(LaneBoundaryType{
            boundary.s + start_s, boundary.line_type, boundary.line_color});
      }
    }
    if (nearest_lane->id() == lane->id()) {
      find_nearest = true;
    }
  }
}

double LaneSequence::GetDistanceToJunction(
    LaneConstPtr &poi_lane, const ad_e2e::planning::NaviPosition &navi_start,
    bool end_check, bool special_check) const {
  double dist_to_junction = std::numeric_limits<double>::max();
  LaneConstPtr start_lane = nullptr;
  if (navi_start.section_id.empty()) {
    poi_lane = nullptr;
    return dist_to_junction;
  }
  bool find_nearest = false;
  bool find_junction = false;
  for (int index = 0; index < static_cast<int>(lanes_.size()); index++) {
    if (lanes_[index] == nullptr) break;
    if (lanes_[index]->section_id() == navi_start.section_id) {
      dist_to_junction = 0.0;
      find_nearest = true;
    }
    if (!lanes_[index]->junction_id().empty() && find_nearest) {
      if (special_check &&
          (lanes_[index]->junction_id() == "5441040265971053388" ||
           lanes_[index]->junction_id() == "4903172120450889697")) {
        dist_to_junction += lanes_[index]->topo_length();
        continue;
      }
      poi_lane = lanes_[index];
      find_junction = true;
      break;
    }
    if (find_nearest) {
      dist_to_junction += lanes_[index]->topo_length();
    }
  }
  if (end_check) {
    return std::max(dist_to_junction - navi_start.s_offset, Constants::ZERO);
  }
  return find_junction ? std::max(dist_to_junction - navi_start.s_offset, 0.0)
                       : std::numeric_limits<double>::max();
}

double LaneSequence::GetDistanceBetweenJunction(
    const std::string &begin_junction_id,
    LaneConstPtr &next_virtual_lane) const {
  double dist_to_junction = 0.0;
  LaneConstPtr begin_lane = nullptr;
  for (const auto &lane : lanes_) {
    if (lane && !begin_lane && lane->junction_id() == begin_junction_id) {
      begin_lane = lane;
    } else if (lane && begin_lane && lane->junction_id() != begin_junction_id &&
               !lane->junction_id().empty()) {
      next_virtual_lane = lane;
      return dist_to_junction;
    } else if (lane && begin_lane && lane->junction_id().empty()) {
      dist_to_junction += lane->topo_length();
    }
  }
  return dist_to_junction;
}

double LaneSequence::GetDistanceToJunction(const double &x, const double &y,
                                           LaneConstPtr &poi_lane) const {
  double dist_to_junction = std::numeric_limits<double>::max();
  int start_idx = -1;
  double project_dis;
  LaneConstPtr start_lane =
      GetNearestLane(ad_e2e::planning::math::Vec2d(x, y), &project_dis);
  if (project_dis > Constants::DEFAULT_LANE_WIDTH || !start_lane) {
    return dist_to_junction;
  }
  for (const auto &lane : lanes_) {
    start_idx++;
    if (lane && lane->id() == start_lane->id()) {
      break;
    }
  }

  if (!lanes_[start_idx]->junction_id().empty()) {
    poi_lane = lanes_[start_idx];
    return 0.0;
  }

  double s = 0.0, l = 0.0;
  start_lane->center_line().GetProjection({x, y}, &s, &l);
  dist_to_junction = start_lane->center_line().length() - s;
  ++start_idx;
  while (start_idx < static_cast<int>(lanes_.size()) &&
         (lanes_[start_idx]->junction_id().empty() ||
          !IsNumeric(lanes_[start_idx]->junction_id()))) {
    dist_to_junction += lanes_[start_idx]->topo_length();
    ++start_idx;
  }
  if (start_idx == static_cast<int>(lanes_.size())) {
    dist_to_junction = std::numeric_limits<double>::max();
    poi_lane = nullptr;
  } else {
    poi_lane = lanes_[start_idx];
  }
  return dist_to_junction;
}

double LaneSequence::GetDistanceExitJunction(
    const ad_e2e::planning::NaviPosition &navi_start) const {
  double dist_exit_junction = std::numeric_limits<double>::max();
  if (navi_start.section_id.empty()) {
    return dist_exit_junction;
  }
  bool find_nearest = false;
  bool find_junction_start = false;
  bool find_junction_end = false;
  for (int index = 0; index < static_cast<int>(lanes_.size()); index++) {
    if (lanes_[index] == nullptr) break;
    if (find_junction_end) {
      break;
    }
    if (find_junction_start) {
      if (lanes_[index]->type() != LANE_VIRTUAL_JUNCTION &&
          lanes_[index]->type() != LANE_LEFT_WAIT) {
        find_junction_end = true;
        continue;
      }
    }
    if (lanes_[index]->section_id() == navi_start.section_id) {
      dist_exit_junction = 0.0;
      find_nearest = true;
    }
    if (!lanes_[index]->junction_id().empty() && find_nearest) {
      find_junction_start = true;
    }
    if (find_nearest) {
      dist_exit_junction += lanes_[index]->topo_length();
    }
  }
  return find_junction_end
             ? std::fmax(dist_exit_junction - navi_start.s_offset, 0.0)
             : std::numeric_limits<double>::max();
}

bool LaneSequence::CanPassJunction(
    const ad_e2e::planning::NaviPosition &navi_start, double *dist_to_junction,
    bool next_junction) {
  LaneConstPtr junction_lane = nullptr;
  double dist = GetDistanceToJunction(junction_lane, navi_start, true);
  if (dist_to_junction) {
    *dist_to_junction = dist;
  }
  bool navi_virtual_lane = true;
  if (next_junction && junction_lane &&
      junction_lane->section_id() == navi_start.section_id) {
    GetDistanceBetweenJunction(junction_lane->junction_id(), junction_lane);
  }
  if (junction_lane && !junction_lane->is_navigation()) {
    navi_virtual_lane = false;
  }
  if (!junction_lane && dist < std::numeric_limits<double>::max()) {
    navi_virtual_lane = false;
  }
  return navi_virtual_lane;
}
bool LaneSequence::IsTurnLeft(const double &x, const double &y) {
  int start_idx = -1;
  double project_dis;
  LaneConstPtr start_lane =
      GetNearestLane(ad_e2e::planning::math::Vec2d(x, y), &project_dis);
  if (project_dis > Constants::DEFAULT_LANE_WIDTH || !start_lane) {
    return false;
  }
  for (const auto &lane : lanes_) {
    start_idx++;
    if (lane && lane->id() == start_lane->id()) {
      break;
    }
  }
  while (start_idx < static_cast<int>(lanes_.size()) &&
         lanes_[start_idx]->junction_id().empty()) {
    ++start_idx;
  }
  if (start_idx == -1 || start_idx >= static_cast<int>(lanes_.size())) {
    return false;
  }
  return lanes_[start_idx]->turn_type() == LEFT_TURN;
}
bool LaneSequence::IsTurnRight(const double &x, const double &y) {
  int start_idx = -1;
  double project_dis;
  LaneConstPtr start_lane =
      GetNearestLane(ad_e2e::planning::math::Vec2d(x, y), &project_dis);
  if (project_dis > Constants::DEFAULT_LANE_WIDTH || !start_lane) {
    return false;
  }
  for (const auto &lane : lanes_) {
    start_idx++;
    if (lane && lane->id() == start_lane->id()) {
      break;
    }
  }
  while (start_idx < static_cast<int>(lanes_.size()) &&
         lanes_[start_idx]->junction_id().empty()) {
    ++start_idx;
  }
  if (start_idx == -1 || start_idx >= static_cast<int>(lanes_.size())) {
    return false;
  }
  return lanes_[start_idx]->turn_type() == RIGHT_TURN;
}

bool LaneSequence::IsOnLaneSequence(const LaneConstPtr &lane) const {
  if (lanes_.empty() || !lane) {
    return false;
  }
  for (const auto &compare_lane : lanes_) {
    if (compare_lane && compare_lane->id() == lane->id()) {
      return true;
    }
  }
  return false;
}

const LanePtr LaneSequence::GetPreLaneOnLaneSequence(
    const LaneConstPtr &lane) const {
  if (lanes_.size() < 2 || !lane) {
    return nullptr;
  }
  for (int i = 0; i < lanes_.size(); i++) {
    if (lanes_[i] && lanes_[i]->id() == lane->id()) {
      return std::const_pointer_cast<ad_e2e::planning::Lane>(
          i > 0 ? lanes_[i - 1] : nullptr);
    }
  }
  return nullptr;
}

const LanePtr LaneSequence::GetNextLaneOnLaneSequence(
    const LaneConstPtr &lane) const {
  if (lanes_.empty() || lanes_.size() < 2 || !lane) {
    return nullptr;
  }
  for (int i = 0; i < lanes_.size(); i++) {
    if (lanes_[i] && lanes_[i]->id() == lane->id()) {
      return std::const_pointer_cast<ad_e2e::planning::Lane>(
          i < lanes_.size() - 1 ? lanes_[i + 1] : nullptr);
    }
  }
  return nullptr;
}

double LaneSequence::GetDistanceToTargetLaneType(
    LaneConstPtr &poi_lane, const std::unordered_set<LaneType> &lane_type_set,
    const ad_e2e::planning::NaviPosition &navi_start) const {
  double dist_to_lane = std::numeric_limits<double>::max();
  LaneConstPtr start_lane = nullptr;
  if (navi_start.section_id.empty()) {
    poi_lane = nullptr;
    return dist_to_lane;
  }
  bool find_nearest = false;
  bool find_lane = false;
  for (int index = 0; index < lanes_.size(); index++) {
    if (lanes_[index] == nullptr) break;
    if (lanes_[index]->section_id() == navi_start.section_id) {
      dist_to_lane = 0.0;
      find_nearest = true;
    }
    if (!find_nearest) {
      continue;
    }
    if (lane_type_set.find(lanes_[index]->type()) != lane_type_set.end()) {
      poi_lane = lanes_[index];
      find_lane = true;
      break;
    }
    if (find_nearest) {
      dist_to_lane += lanes_[index]->topo_length();
    }
  }
  return find_lane ? std::max(dist_to_lane - navi_start.s_offset, 0.0)
                   : std::numeric_limits<double>::max();
}

double LaneSequence::GetDistanceToCustomSpeedLimit(
    const ad_e2e::planning::NaviPosition &navi_start,
    double *speed_limit) const {
  double dist_to_custom = DBL_MAX;
  if (navi_start.section_id.empty()) {
    return dist_to_custom;
  }
  bool find_navi_start_lane = false;
  bool find_custom_lane = false;
  for (const auto &lane : lanes_) {
    if (!lane) continue;
    if (!lane->is_navigation() && find_navi_start_lane) break;
    if (!lane->is_navigation() && !find_navi_start_lane) continue;
    if (lane->section_id() == navi_start.section_id) {
      dist_to_custom = 0.0;
      find_navi_start_lane = true;
    }
    if (!find_navi_start_lane) continue;

    double custom_speed_limit = 0.0;
    if (lane->GetCustomSpeedLimit(&custom_speed_limit)) {
      find_custom_lane = true;
      *speed_limit = custom_speed_limit;
      break;
    }
    dist_to_custom += lane->topo_length();
  }
  if (!find_custom_lane) {
    dist_to_custom = DBL_MAX;
  } else {
    dist_to_custom -= navi_start.s_offset;
  }
  return dist_to_custom;
}

NavigableLaneSequence::NavigableLaneSequence(
    const std::vector<LaneConstPtr> &lanes_ptr_vec)
    : LaneSequence(lanes_ptr_vec) {
  navi_distance_ = 0.0;
}

bool NavigableLaneSequence::IsOverLap(
    const std::vector<LaneConstPtr> &other_lane_sequence) {
  const std::vector<LaneConstPtr> &cur_lane_sequence = lanes();
  if (cur_lane_sequence.empty() || other_lane_sequence.empty()) {
    return false;
  }
  std::vector<std::string> cur_lane_sequence_ids;
  std::vector<std::string> other_lane_sequence_ids;
  for (const auto &lane : cur_lane_sequence) {
    cur_lane_sequence_ids.emplace_back(lane->id());
  }
  for (const auto &lane : other_lane_sequence) {
    other_lane_sequence_ids.emplace_back(lane->id());
  }
  auto iter1 = cur_lane_sequence_ids.begin();
  auto start_iter2 = std::find(other_lane_sequence_ids.begin(),
                               other_lane_sequence_ids.end(), *iter1);
  if (start_iter2 == other_lane_sequence_ids.end()) {
    return false;
  }
  for (auto iter2 = start_iter2; iter1 != cur_lane_sequence_ids.end() &&
                                 iter2 != other_lane_sequence_ids.end();
       iter1++, iter2++) {
    if (*iter1 != *iter2) {
      return false;
    }
  }
  return true;
}

bool NavigableLaneSequence::IsTwoSplitLane(
    const std::vector<LaneConstPtr> &other_lane_sequence) {
  const std::vector<LaneConstPtr> &cur_lane_sequence = lanes();
  if (cur_lane_sequence.empty() || other_lane_sequence.empty()) {
    return false;
  }
  std::vector<std::string> cur_lane_sequence_ids;
  std::vector<std::string> other_lane_sequence_ids;
  for (const auto &lane : cur_lane_sequence) {
    cur_lane_sequence_ids.emplace_back(lane->id());
  }
  for (const auto &lane : other_lane_sequence) {
    other_lane_sequence_ids.emplace_back(lane->id());
  }
  auto iter1 = cur_lane_sequence_ids.begin();
  auto start_iter2 = std::find(other_lane_sequence_ids.begin(),
                               other_lane_sequence_ids.end(), *iter1);
  if (start_iter2 == other_lane_sequence_ids.end()) {
    return false;
  }
  for (auto iter2 = start_iter2; iter1 != cur_lane_sequence_ids.end() &&
                                 iter2 != other_lane_sequence_ids.end();
       iter1++, iter2++) {
    if (*iter1 != *iter2) {
      return true;
    }
  }
  return false;
}

std::string NavigableLaneSequence::FindSamePrevLane(
    const std::vector<LaneConstPtr> &other_lane_sequence) {
  const std::vector<LaneConstPtr> &cur_lane_sequence = lanes();
  if (cur_lane_sequence.empty() || other_lane_sequence.empty()) {
    return "";
  }
  if (cur_lane_sequence.at(0)->id() == other_lane_sequence.at(0)->id()) {
    return "";
  }
  std::vector<std::string> cur_prevs = cur_lane_sequence.at(0)->pre_lane_ids();
  std::vector<std::string> other_prevs =
      other_lane_sequence.at(0)->pre_lane_ids();
  std::vector<std::string> common;
  std::sort(cur_prevs.begin(), cur_prevs.end());
  std::sort(other_prevs.begin(), other_prevs.end());
  std::set_intersection(cur_prevs.begin(), cur_prevs.end(), other_prevs.begin(),
                        other_prevs.end(), std::back_inserter(common));

  if (!common.empty()) {
    return common.at(0);
  }
  return "";
}

void LaneSequence::GetRoadSegments(
    const bool is_left, std::vector<math::LineSegment2d> *road_segs) const {
  road_segs->clear();
  for (const auto &lane : lanes_) {
    if (!lane) continue;
    const auto &road_boundary =
        is_left ? lane->left_road_boundary() : lane->right_road_boundary();
    if (!road_boundary) continue;
    for (const auto &road : road_boundary->road_boundaries()) {
      const auto &points = road->points();
      for (int i = 1; i < points.size(); i++) {
        road_segs->emplace_back(points[i - 1], points[i]);
      }
    }
  }
}
}  // namespace planning
}  // namespace ad_e2e
