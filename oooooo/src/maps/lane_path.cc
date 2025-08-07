#include "maps/lane_path.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ostream>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "container/strong_int.h"
#include "maps/map_or_die_macros.h"
#include "math/util.h"

namespace e2e_noa::mapping {

bool LanePath::IsValid(const ad_e2e::planning::MapConstPtr& map_ptr) const {
  if (lane_ids().empty()) {
    LOG(WARNING) << "LanePath validation: lane_ids empty";
    return false;
  }

  if (start_fraction() < 0.0) {
    LOG(WARNING) << "LanePath validation: start_fraction < 0.0";
    return false;
  }
  if (start_fraction() > 1.0) {
    LOG(WARNING) << "LanePath validation: start_fraction > 1.0";
    return false;
  }
  if (end_fraction() < 0.0) {
    LOG(WARNING) << "LanePath validation: end_fraction < 0.0";
    return false;
  }
  if (end_fraction() > 1.0) {
    LOG(WARNING) << "LanePath validation: end_fraction > 1.0";
    return false;
  }

  for (int i = 0; i + 1 < lane_path_data_.size(); ++i) {
    SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, *map_ptr,
                                    lane_path_data_.lane_ids()[i], false);
    const auto& outgoing_lane_indices = lane_info.next_lane_ids();
    if (std::find_if(outgoing_lane_indices.begin(), outgoing_lane_indices.end(),
                     [this, i, map_ptr](const std::string& id) {
                       return id == lane_path_data_.lane_ids()[i + 1];
                     }) == outgoing_lane_indices.end()) {
      LOG(WARNING) << "LanePath validation: lane "
                   << lane_path_data_.lane_ids()[i] << " does not have lane "
                   << lane_path_data_.lane_ids()[i + 1]
                   << " as an outgoing lane";
      return false;
    }
  }

  return true;
}

void LanePath::Build(const ad_e2e::planning::MapConstPtr& map_ptr) {
  lane_end_s_.clear();
  lane_end_s_.reserve(lane_path_data_.size() + 1);
  lane_end_s_.push_back(0.0);
  FillLaneLengthsByIdsIfAbsent(map_ptr);

  std::vector<ad_e2e::planning::LaneConstPtr> lanes;
  for (const auto& lane_id : lane_path_data_.lane_ids()) {
    const auto& lane = map_ptr->GetLaneById(lane_id);
    if (!lane) continue;
    lanes.emplace_back(lane);
  }
  lane_seq_ =
      std::make_shared<ad_e2e::planning::LaneSequence>(std::move(lanes));

  if (lane_path_data_.size() == 1) {
    const double len = GetLaneLength(map_ptr, 0);
    lane_end_s_.push_back(len * std::abs(end_fraction() - start_fraction()));
    return;
  }
  for (int i = 0; i < lane_path_data_.size(); ++i) {
    const double len = GetLaneLength(map_ptr, i);
    double fraction = 1.0;
    if (i == 0) {
      fraction = lane_path_data_.lane_path_in_forward_direction()
                     ? (1.0 - start_fraction())
                     : start_fraction();
    } else if (i + 1 == lane_path_data_.size()) {
      fraction = lane_path_data_.lane_path_in_forward_direction()
                     ? end_fraction()
                     : (1.0 - end_fraction());
    }
    lane_end_s_.push_back(lane_end_s_.back() + len * fraction);
  }

  CHECK_EQ(lane_end_s_.size(), lane_path_data_.size() + 1);
  CHECK_GE(start_fraction(), 0.0);
  CHECK_LE(end_fraction(), 1.0);
  if (lane_path_data_.size() == 1) {
    CHECK_LE(start_fraction(), end_fraction());
  }
}

double LanePath::GetLaneLength(const ad_e2e::planning::MapConstPtr& map_ptr,
                               int index) const {
  if (index < lane_lengths_.size()) {
    return lane_lengths_[index];
  }
  const ElementId id = lane_path_data_.lane_ids()[index];
  SMM2_ASSIGN_LANE_OR_RETURN_ISSUE(lane, *map_ptr, id, 0.0);

  return lane.curve_length();
}

std::string LanePath::DebugString() const {
  return IsEmpty() ? "Empty lane path."
                   : absl::StrFormat(
                         "lane_ids: %s, start_fraction: %7.6f, end_fraction:  "
                         "%7.6f, length: %7.6f.",
                         absl::StrJoin(lane_path_data_.lane_ids(), ", "),
                         start_fraction(), end_fraction(), length());
}

bool LanePath::LaneSegment::HasOverlap(
    const LaneSegment& other,
    std::pair<double, double>* overlap_fraction) const {
  if (lane_id != other.lane_id) return false;

  const double min_fraction = std::max(start_fraction, other.start_fraction);
  const double max_fraction = std::min(end_fraction, other.end_fraction);

  if (max_fraction >= min_fraction) {
    if (overlap_fraction) {
      *overlap_fraction = std::make_pair(min_fraction, max_fraction);
    }
    return true;
  }
  return false;
}

LanePoint LanePath::ArclengthToLanePoint(double s) const {
  const auto lane_index_point = ArclengthToLaneIndexPoint(s);
  return {lane_path_data_.lane_ids()[lane_index_point.first],
          lane_index_point.second};
}

double LanePath::FirstOccurrenceOfLanePointToArclength(LanePoint point) const {
  return LaneIndexPointToArclength(
      FirstOccurrenceOfLanePointToLaneIndexPoint(point));
}

double LanePath::LastOccurrenceOfLanePointToArclength(LanePoint point) const {
  return LaneIndexPointToArclength(
      LastOccurrenceOfLanePointToLaneIndexPoint(point));
}

LanePath::LaneIndexWaypoint
LanePath::FirstOccurrenceOfLanePointToLaneIndexPoint(LanePoint point) const {
  LaneIndexWaypoint lip;
  if (ContainsLanePoint(point, &lip, true)) return lip;
  LOG(FATAL) << "Lane point " << point.DebugString() << " is not on lane path "
             << DebugString();
}

LanePath::LaneIndexWaypoint LanePath::LastOccurrenceOfLanePointToLaneIndexPoint(
    LanePoint point) const {
  LaneIndexWaypoint lip;
  if (ContainsLanePoint(point, &lip, false)) return lip;
  LOG(FATAL) << "Lane point " << point.DebugString() << " is not on lane path "
             << DebugString();
}

int LanePath::ArclengthToLaneIndex(double s) const {
  CHECK_GE(lane_end_s_.size(), 2);
  if (s >= length()) return lane_path_data_.size() - 1;
  if (s <= 0.0) return 0;
  const int index =
      std::upper_bound(lane_end_s_.begin(), lane_end_s_.end(), s) -
      lane_end_s_.begin() - 1;
  CHECK_GE(index, 0);
  CHECK_LT(index, lane_path_data_.size());
  return index;
}

double LanePath::LaneIndexToArclength(int lane_index) const {
  CHECK_LT(lane_index, lane_path_data_.size());
  return lane_end_s_[lane_index];
}

LanePath::LaneIndexWaypoint LanePath::ArclengthToLaneIndexPoint(
    double s) const {
  CHECK_GE(lane_end_s_.size(), 2);
  if (s >= length()) {
    return {lane_path_data_.size() - 1, lane_path_data_.end_fraction()};
  }

  if (s <= 0.0) return {0, lane_path_data_.start_fraction()};
  const int index = ArclengthToLaneIndex(s);
  CHECK_GE(s, lane_end_s_[index]);
  CHECK_LE(s, lane_end_s_[index + 1]);
  const double lane_fraction =
      Lerp(index == 0 ? lane_path_data_.start_fraction() : 0.0,
           index + 1 == lane_path_data_.size() ? lane_path_data_.end_fraction()
                                               : 1.0,
           LerpFactor(lane_end_s_[index], lane_end_s_[index + 1], s));
  CHECK_GE(lane_fraction, 0.0);
  CHECK_LE(lane_fraction, 1.0);
  return {index, std::clamp(lane_fraction, 0.0, 1.0)};
}

double LanePath::LaneIndexPointToArclength(int lane_index,
                                           double lane_fraction) const {
  CHECK_GE(lane_index, 0);
  CHECK_LT(lane_index, lane_path_data_.size());
  CHECK_GE(lane_fraction, 0.0);
  CHECK_LE(lane_fraction, 1.0);
  return Lerp(
      lane_end_s_[lane_index], lane_end_s_[lane_index + 1],
      LerpFactor(lane_index == 0 ? lane_path_data_.start_fraction() : 0.0,
                 lane_index + 1 == lane_path_data_.size()
                     ? lane_path_data_.end_fraction()
                     : 1.0,
                 lane_fraction));
}

double LanePath::LaneIndexPointToArclength(
    const LaneIndexWaypoint& lane_index_point) const {
  return LaneIndexPointToArclength(lane_index_point.first,
                                   lane_index_point.second);
}

bool LanePath::ContainsLanePoint(
    LanePoint point, LaneIndexWaypoint* first_encountered_lane_index_point,
    bool forward_search) const {
  const int size = lane_path_data_.size();
  if (size == 0) return false;
  const int search_start = forward_search ? 0 : size - 1;
  const int search_end = forward_search ? size : -1;
  const int search_inc = forward_search ? 1 : -1;
  for (int i = search_start; i != search_end; i += search_inc) {
    if (lane_path_data_.lane_ids()[i] != point.lane_id()) continue;
    bool found = false;
    if (size == 1) {
      if (point.fraction() > start_fraction() - kEpsilon &&
          point.fraction() < end_fraction() + kEpsilon) {
        found = true;
      }

    } else if (i == 0 && point.fraction() >= start_fraction() - kEpsilon) {
      found = true;

    } else if (i + 1 == size && point.fraction() <= end_fraction() + kEpsilon) {
      found = true;

    } else if (i != 0 && i + 1 != size) {
      found = true;
    }
    if (found) {
      if (first_encountered_lane_index_point != nullptr) {
        *first_encountered_lane_index_point = {i, point.fraction()};
      }
      return true;
    }
  }
  return false;
}

LanePath LanePath::BeforeFirstOccurrenceOfLanePoint(LanePoint point) const {
  return BeforeLaneIndexPoint(
      FirstOccurrenceOfLanePointToLaneIndexPoint(point));
}

LanePath LanePath::BeforeLastOccurrenceOfLanePoint(LanePoint point) const {
  return BeforeLaneIndexPoint(LastOccurrenceOfLanePointToLaneIndexPoint(point));
}

LanePath LanePath::AfterFirstOccurrenceOfLanePoint(LanePoint point) const {
  return AfterLaneIndexPoint(FirstOccurrenceOfLanePointToLaneIndexPoint(point));
}

LanePath LanePath::AfterLastOccurrenceOfLanePoint(LanePoint point) const {
  return AfterLaneIndexPoint(LastOccurrenceOfLanePointToLaneIndexPoint(point));
}

LanePath LanePath::BeforeArclength(double s) const {
  return BeforeLaneIndexPoint(ArclengthToLaneIndexPoint(s));
}

LanePath LanePath::AfterArclength(double s) const {
  return AfterLaneIndexPoint(ArclengthToLaneIndexPoint(s));
}

LanePath LanePath::BeforeLaneIndexPoint(int lane_index,
                                        double lane_fraction) const {
  CHECK_GE(lane_index, 0);
  CHECK_LT(lane_index, lane_path_data_.size());
  std::vector<ElementId> ids = lane_path_data_.lane_ids();
  ids.erase(ids.begin() + lane_index + 1, ids.end());

  LanePathData lp_data(start_fraction(), lane_fraction, std::move(ids),
                       lane_path_data_.lane_path_in_forward_direction());
  std::vector<double> lane_lengths(lane_lengths_.begin(),
                                   lane_lengths_.begin() + lane_index + 1);
  std::vector<double> lane_end_s(lane_end_s_.begin(),
                                 lane_end_s_.begin() + lane_index + 1);
  lane_end_s.push_back(LaneIndexPointToArclength(lane_index, lane_fraction));
  CHECK_EQ(lane_end_s.size(), lp_data.size() + 1);

  return LanePath(std::move(lp_data), std::move(lane_end_s),
                  std::move(lane_lengths));
}

LanePath LanePath::AfterLaneIndexPoint(int lane_index,
                                       double lane_fraction) const {
  CHECK_GE(lane_index, 0);
  CHECK_LT(lane_index, lane_path_data_.size());
  std::vector<ElementId> ids = lane_path_data_.lane_ids();

  ids.erase(ids.begin(), ids.begin() + lane_index);

  LanePathData lp_data(lane_fraction, end_fraction(), std::move(ids),
                       lane_path_data_.lane_path_in_forward_direction());
  std::vector<double> lane_lengths(lane_lengths_.begin() + lane_index,
                                   lane_lengths_.end());
  std::vector<double> lane_end_s;
  lane_end_s.reserve(lp_data.size() + 1);
  lane_end_s.push_back(0.0);
  const double length = LaneIndexPointToArclength(lane_index, lane_fraction);
  for (int i = lane_index + 1; i < lane_end_s_.size(); ++i) {
    lane_end_s.push_back(lane_end_s_[i] - length);
  }

  CHECK_EQ(lane_end_s.size(), lp_data.size() + 1);
  return LanePath(std::move(lp_data), std::move(lane_end_s),
                  std::move(lane_lengths));
}

LanePath LanePath::BeforeLaneIndexPoint(
    const LaneIndexWaypoint& lane_index_pt) const {
  return BeforeLaneIndexPoint(lane_index_pt.first, lane_index_pt.second);
}

LanePath LanePath::AfterLaneIndexPoint(
    const LaneIndexWaypoint& lane_index_pt) const {
  return AfterLaneIndexPoint(lane_index_pt.first, lane_index_pt.second);
}

bool LanePath::IsConnectedTo(const ad_e2e::planning::MapConstPtr& map_ptr,
                             const LanePath& subsequent_path) const {
  CHECK(!lane_path_data_.empty());
  CHECK(!subsequent_path.lane_path_data().empty());
  SMM2_ASSIGN_LANE_OR_RETURN(back_lane_info, *map_ptr, lane_ids().back(),
                             false);
  if (end_fraction() == 1.0 && subsequent_path.start_fraction() == 0.0) {
    const auto& outgoing_lanes = back_lane_info.next_lane_ids();
    const ElementId front_id = subsequent_path.lane_ids().front();
    return std::find_if(outgoing_lanes.begin(), outgoing_lanes.end(),
                        [front_id](const std::string& id) {
                          return id == front_id;
                        }) != outgoing_lanes.end();
  }
  if (lane_path_data_.lane_ids().back() != subsequent_path.lane_ids().front())
    return false;

  constexpr double kLanePathConnectedDistanceThreshold = 0.01;
  if (std::abs(lane_path_data_.end_fraction() -
               subsequent_path.start_fraction()) *
          back_lane_info.curve_length() >
      kLanePathConnectedDistanceThreshold) {
    return false;
  }
  return true;
}

LanePath LanePath::Connect(const ad_e2e::planning::MapConstPtr& map_ptr,
                           const LanePath& subsequent_path) const {
  CHECK(!lane_path_data_.empty());
  CHECK(IsConnectedTo(map_ptr, subsequent_path))
      << "this Path: " << DebugString()
      << "\nother Path: " << subsequent_path.DebugString();
  std::vector<ElementId> ids = lane_path_data_.lane_ids();
  if (lane_path_data_.lane_ids().back() == subsequent_path.lane_ids().front())
    ids.pop_back();
  ids.insert(ids.end(), subsequent_path.lane_ids().begin(),
             subsequent_path.lane_ids().end());
  return LanePath(map_ptr, std::move(ids), start_fraction(),
                  subsequent_path.end_fraction());
}

void LanePath::FillLaneLengthsByIdsIfAbsent(
    const ad_e2e::planning::MapConstPtr& map_ptr) {
  for (int i = 0; i < lane_path_data_.size(); ++i) {
    const ElementId id = lane_path_data_.lane_ids()[i];
    if (i >= lane_lengths_.size()) {
      const auto lane = map_ptr->GetLaneById(id);
      if (lane == nullptr) {
        LOG(ERROR) << "Not found map lane id: " << id;
        lane_lengths_.push_back(0.0);
      } else {
        lane_lengths_.push_back(lane->curve_length());
      }
    }
  }
}

}  // namespace e2e_noa::mapping
