#include "plan/composite_lane_path.h"

#include <cmath>
#include <limits>
#include <ostream>

#include "google/protobuf/stubs/port.h"

namespace e2e_noa {
namespace planning {

void CompositeLanePath::Build() {
  lane_path_end_s_.clear();
  lane_path_end_s_.reserve(num_lane_paths() + 1);
  lane_path_end_s_.push_back(0.0);
  cumulative_lane_path_sizes_.clear();
  cumulative_lane_path_sizes_.reserve(num_lane_paths() + 1);
  cumulative_lane_path_sizes_.push_back(0);
  for (int i = 0; i < num_lane_paths(); ++i) {
    const double s =
        lane_path_end_s_.back() +
        (lane_paths_[i].length() - GetLanePathLengthBeforeTransition(i) -
         GetLanePathLengthAfterTransition(i));
    VLOG(3) << i << " " << lane_paths_[i].length() << " "
            << GetLanePathLengthBeforeTransition(i) << " "
            << GetLanePathLengthAfterTransition(i) << " " << s;
    lane_path_end_s_.push_back(s);
    cumulative_lane_path_sizes_.push_back(cumulative_lane_path_sizes_.back() +
                                          lane_paths_[i].size());
  }
  CHECK_EQ(num_lane_paths() + 1, lane_path_end_s_.size());

  lane_path_transition_points_.clear();
  lane_path_transition_points_.reserve(num_lane_paths());
  for (int i = 0; i < num_lane_paths(); ++i) {
    LanePath& lane_path = lane_paths_[i];
    const LanePath::LaneIndexWaypoint entry =
        lane_path.ArclengthToLaneIndexPoint(
            GetLanePathLengthBeforeTransition(i));
    const LanePath::LaneIndexWaypoint exit =
        lane_path.ArclengthToLaneIndexPoint(
            lane_path.length() - GetLanePathLengthAfterTransition(i));
    lane_path_transition_points_.emplace_back(entry, exit);
  }

  lane_ids_.clear();
  lane_ids_.reserve(end() - begin());
  for (auto it = begin(); it != end(); ++it) {
    lane_ids_.push_back((*it).lane_id);
  }
  CHECK_EQ(lane_ids_.size(), end() - begin());
}

void CompositeLanePath::Clear() {
  lane_paths_.clear();
  transitions_.clear();
  lane_path_end_s_.clear();
  lane_path_transition_points_.clear();
  lane_ids_.clear();
  cumulative_lane_path_sizes_.clear();
}

CompositeLanePath::CompositeIndex
CompositeLanePath::OverallIndexToCompositeIndex(int overall_lane_index) const {
  CHECK_GE(cumulative_lane_path_sizes_.size(), 2);
  CHECK_GE(overall_lane_index, 0);
  CHECK_LT(overall_lane_index, size());
  const int lane_path_index =
      std::upper_bound(cumulative_lane_path_sizes_.begin(),
                       cumulative_lane_path_sizes_.end(), overall_lane_index) -
      cumulative_lane_path_sizes_.begin() - 1;
  CHECK_GE(lane_path_index, 0);
  CHECK_LT(lane_path_index, num_lane_paths());
  return {lane_path_index,
          overall_lane_index - cumulative_lane_path_sizes_[lane_path_index]};
}

int CompositeLanePath::CompositeIndexToOverallIndex(
    CompositeIndex composite_index) const {
  return cumulative_lane_path_sizes_[composite_index.lane_path_index] +
         composite_index.lane_index;
}

int CompositeLanePath::ArclengthToLanePathIndex(double s) const {
  CHECK_GE(lane_path_end_s_.size(), 2);
  if (s >= length()) return num_lane_paths() - 1;
  if (s <= 0.0) return 0;
  const int index =
      std::upper_bound(lane_path_end_s_.begin(), lane_path_end_s_.end(), s) -
      lane_path_end_s_.begin() - 1;
  CHECK_GE(index, 0);
  CHECK_LT(index, num_lane_paths());
  return index;
}

CompositeLanePath::LanePathPoint CompositeLanePath::ArclengthToLanePathPoint(
    double s) const {
  CHECK_GE(lane_path_end_s_.size(), 2);
  if (s >= length()) return {num_lane_paths() - 1, lane_paths_.back().length()};
  if (s <= 0.0) return {0, 0.0};
  const int lane_path_index = ArclengthToLanePathIndex(s);
  CHECK_GE(s, lane_path_end_s_[lane_path_index]);
  CHECK_LE(s, lane_path_end_s_[lane_path_index + 1]);
  return {lane_path_index,
          s - lane_path_end_s_[lane_path_index] +
              GetLanePathLengthBeforeTransition(lane_path_index)};
}

CompositeLanePath::LaneIndexWaypoint
CompositeLanePath::ArclengthToLaneIndexPoint(double s) const {
  const LanePathPoint lane_path_point = ArclengthToLanePathPoint(s);
  const LanePath::LaneIndexWaypoint lane_index_point =
      lane_paths_[lane_path_point.index()].ArclengthToLaneIndexPoint(
          lane_path_point.s());
  return {{lane_path_point.index(), lane_index_point.first},
          lane_index_point.second};
}

double CompositeLanePath::LanePathIndexToArclength(int lane_path_index) const {
  CHECK_LT(lane_path_index, num_lane_paths());
  return lane_path_end_s_[lane_path_index];
}

double CompositeLanePath::LanePathPointToArclength(
    LanePathPoint lane_path_point) const {
  const int lane_path_index = lane_path_point.index();
  const double s_in_lane_path = lane_path_point.s();
  CHECK_GE(lane_path_index, 0);
  CHECK_LT(lane_path_index, num_lane_paths());
  return lane_path_end_s_[lane_path_index] +
         (s_in_lane_path - GetLanePathLengthBeforeTransition(lane_path_index));
}

double CompositeLanePath::LaneIndexPointToArclength(
    const LaneIndexWaypoint& lane_index_point) const {
  const CompositeIndex& composite_index = lane_index_point.index();
  const double lane_fraction = lane_index_point.lane_fraction();
  CHECK_GE(composite_index.lane_path_index, 0);
  CHECK_LT(composite_index.lane_path_index, num_lane_paths());
  CHECK_GE(lane_fraction, 0.0);
  CHECK_LE(lane_fraction, 1.0);
  const double s_in_lane_path =
      lane_paths_[composite_index.lane_path_index].LaneIndexPointToArclength(
          composite_index.lane_index, lane_fraction);
  return LanePathPointToArclength(
      {composite_index.lane_path_index, s_in_lane_path});
}

mapping::LanePoint CompositeLanePath::ArclengthToLanePoint(double s) const {
  const auto lane_path_point = ArclengthToLanePathPoint(s);
  return lane_paths_[lane_path_point.index()].ArclengthToLanePoint(
      lane_path_point.s());
}

double CompositeLanePath::FirstOccurrenceOfLanePointToArclength(
    LanePoint point) const {
  return LaneIndexPointToArclength(
      FirstOccurrenceOfLanePointToLaneIndexPoint(point));
}

CompositeLanePath::LanePathPoint
CompositeLanePath::FirstOccurrenceOfLanePointToLanePathPoint(
    LanePoint point) const {
  for (int i = 0; i < num_lane_paths(); ++i) {
    const LanePath& lane_path = lane_paths_[i];
    LanePath::LaneIndexWaypoint lane_index_point;
    if (!lane_path.ContainsLanePoint(point, &lane_index_point, true)) continue;
    const double s = lane_path.LaneIndexPointToArclength(lane_index_point);
    CHECK(!std::isinf(s));
    if (s < GetLanePathLengthBeforeTransition(i)) continue;
    if (s > lane_path.length() - GetLanePathLengthAfterTransition(i)) continue;
    return {i, s};
  }

  return {-1, std::numeric_limits<double>::infinity()};
}

CompositeLanePath::LanePathPoint
CompositeLanePath::LastOccurrenceOfLanePointToLanePathPoint(
    LanePoint point) const {
  for (int i = num_lane_paths() - 1; i >= 0; --i) {
    const LanePath& lane_path = lane_paths_[i];
    LanePath::LaneIndexWaypoint lane_index_point;
    if (!lane_path.ContainsLanePoint(point, &lane_index_point, false)) continue;
    const double s = lane_path.LaneIndexPointToArclength(lane_index_point);
    CHECK(!std::isinf(s));
    if (s < GetLanePathLengthBeforeTransition(i)) continue;
    if (s > lane_path.length() - GetLanePathLengthAfterTransition(i)) continue;
    return {i, s};
  }

  return {-1, std::numeric_limits<double>::infinity()};
}

CompositeLanePath::LaneIndexWaypoint
CompositeLanePath::FirstOccurrenceOfLanePointToLaneIndexPoint(
    LanePoint point) const {
  LaneIndexWaypoint lip;
  if (ContainsLanePoint(point, &lip, true)) return lip;

  return {composite_index_end(), std::numeric_limits<double>::infinity()};
}

CompositeLanePath::LaneIndexWaypoint
CompositeLanePath::LastOccurrenceOfLanePointToLaneIndexPoint(
    LanePoint point) const {
  LaneIndexWaypoint lip;
  if (ContainsLanePoint(point, &lip, false)) return lip;

  return {composite_index_end(), std::numeric_limits<double>::infinity()};
}

bool CompositeLanePath::ContainsLanePoint(
    LanePoint point, LaneIndexWaypoint* first_encounter_of_lane_index_point,
    bool forward_search) const {
  const int size = num_lane_paths();
  if (size == 0) return false;
  const int search_start = forward_search ? 0 : size - 1;
  const int search_end = forward_search ? size : -1;
  const int search_inc = forward_search ? 1 : -1;
  for (int i = search_start; i != search_end; i += search_inc) {
    const LanePath& lane_path = lane_paths_[i];
    LanePath::LaneIndexWaypoint lip;
    if (!lane_path.ContainsLanePoint(point, &lip, forward_search)) continue;
    const double s = lane_path.LaneIndexPointToArclength(lip);
    CHECK(!std::isinf(s));
    if (s < GetLanePathLengthBeforeTransition(i)) continue;
    if (s > lane_path.length() - GetLanePathLengthAfterTransition(i)) continue;
    if (first_encounter_of_lane_index_point != nullptr) {
      *first_encounter_of_lane_index_point = {{i, lip.first}, lip.second};
    }
    return true;
  }
  return false;
}

CompositeLanePath CompositeLanePath::BeforeFirstOccurrenceOfLanePoint(
    LanePoint point) const {
  return BeforeLanePathPoint(FirstOccurrenceOfLanePointToLanePathPoint(point));
}

CompositeLanePath CompositeLanePath::BeforeLastOccurrenceOfLanePoint(
    LanePoint point) const {
  return BeforeLanePathPoint(LastOccurrenceOfLanePointToLanePathPoint(point));
}

CompositeLanePath CompositeLanePath::AfterFirstOccurrenceOfLanePoint(
    LanePoint point) const {
  return AfterLanePathPoint(FirstOccurrenceOfLanePointToLanePathPoint(point));
}

CompositeLanePath CompositeLanePath::BeforeArclength(double s) const {
  return BeforeLanePathPoint(ArclengthToLanePathPoint(s));
}

CompositeLanePath CompositeLanePath::AfterArclength(double s) const {
  return AfterLanePathPoint(ArclengthToLanePathPoint(s));
}

CompositeLanePath CompositeLanePath::BeforeLanePathPoint(
    LanePathPoint lane_path_point) const {
  const int lane_path_index = lane_path_point.index();
  CHECK_GE(lane_path_index, 0);
  CHECK_LT(lane_path_index, num_lane_paths());
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.erase(lane_paths.begin() + lane_path_index + 1, lane_paths.end());
  lane_paths.back() = lane_paths.back().BeforeArclength(lane_path_point.s());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.erase(transitions.begin() + lane_path_index, transitions.end());
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath CompositeLanePath::AfterLanePathPoint(
    LanePathPoint lane_path_point) const {
  const int lane_path_index = lane_path_point.index();
  CHECK_GE(lane_path_index, 0);
  CHECK_LT(lane_path_index, num_lane_paths());
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.erase(lane_paths.begin(), lane_paths.begin() + lane_path_index);
  lane_paths.front() = lane_paths.front().AfterArclength(lane_path_point.s());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.erase(transitions.begin(), transitions.begin() + lane_path_index);
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath CompositeLanePath::BeforeLaneIndexPoint(
    LaneIndexWaypoint lane_index_point) const {
  return BeforeLanePathPoint(
      ArclengthToLanePathPoint(LaneIndexPointToArclength(lane_index_point)));
}

bool CompositeLanePath::IsConnectedTo(
    const ad_e2e::planning::MapConstPtr& map_ptr,
    const CompositeLanePath& subsequent_path) const {
  CHECK(!subsequent_path.IsEmpty());
  return IsConnectedTo(map_ptr, subsequent_path.lane_paths().front());
}

bool CompositeLanePath::IsConnectedTo(
    const ad_e2e::planning::MapConstPtr& map_ptr,
    const LanePath& subsequent_path) const {
  CHECK(!IsEmpty());
  return lane_paths_.back().IsConnectedTo(map_ptr, subsequent_path);
}

CompositeLanePath CompositeLanePath::Connect(
    const ad_e2e::planning::MapConstPtr& map_ptr,
    const CompositeLanePath& subsequent_path) const {
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.back() =
      lane_paths.back().Connect(map_ptr, subsequent_path.lane_paths().front());
  lane_paths.insert(lane_paths.end(),
                    std::next(subsequent_path.lane_paths().begin()),
                    subsequent_path.lane_paths().end());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.insert(transitions.end(), subsequent_path.transitions().begin(),
                     subsequent_path.transitions().end());
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath CompositeLanePath::Compose(
    const CompositeLanePath& subsequent_path,
    const TransitionInfo& transition) const {
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.insert(lane_paths.end(), subsequent_path.lane_paths().begin(),
                    subsequent_path.lane_paths().end());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.push_back(transition);
  transitions.insert(transitions.end(), subsequent_path.transitions().begin(),
                     subsequent_path.transitions().end());
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath::CompositeIndex CompositeLanePath::IncrementCompositeIndex(
    CompositeIndex composite_index, int inc) const {
  if (inc < 0) return DecrementCompositeIndex(composite_index, -inc);
  int exit_index =
      LanePathExitIndexPoint(composite_index.lane_path_index).first;
  while (composite_index.lane_index + inc > exit_index) {
    inc -= exit_index - composite_index.lane_index + 1;
    ++composite_index.lane_path_index;
    if (composite_index.lane_path_index == num_lane_paths()) {
      composite_index.lane_index = 0;
      break;
    }
    composite_index.lane_index =
        LanePathEntryIndexPoint(composite_index.lane_path_index).first;
    exit_index = LanePathExitIndexPoint(composite_index.lane_path_index).first;
  }
  composite_index.lane_index += inc;
  return composite_index;
}

CompositeLanePath::CompositeIndex CompositeLanePath::DecrementCompositeIndex(
    CompositeIndex composite_index, int dec) const {
  if (dec < 0) return IncrementCompositeIndex(composite_index, -dec);
  int entry_index =
      LanePathEntryIndexPoint(composite_index.lane_path_index).first;
  while (composite_index.lane_index - dec < entry_index) {
    dec -= composite_index.lane_index - entry_index + 1;
    --composite_index.lane_path_index;
    if (composite_index.lane_path_index < 0) {
      composite_index.lane_path_index = num_lane_paths();
      composite_index.lane_index = 0;
      break;
    }
    composite_index.lane_index =
        LanePathExitIndexPoint(composite_index.lane_path_index).first;
    entry_index =
        LanePathEntryIndexPoint(composite_index.lane_path_index).first;
  }
  composite_index.lane_index -= dec;
  return composite_index;
}

int CompositeLanePath::CompositeIndexDist(CompositeIndex index0,
                                          CompositeIndex index1) const {
  if (index1 < index0) return -CompositeIndexDist(index1, index0);
  int inc = 0;
  while (index0.lane_path_index < index1.lane_path_index) {
    inc += LanePathExitIndexPoint(index0.lane_path_index).first -
           index0.lane_index + 1;
    ++index0.lane_path_index;
    index0.lane_index =
        index0.lane_path_index < num_lane_paths()
            ? LanePathEntryIndexPoint(index0.lane_path_index).first
            : 0;
  }
  inc += index1.lane_index - index0.lane_index;
  return inc;
}

mapping::LanePath::LaneSegment
CompositeLanePath::lane_segment_in_composite_lane_path(
    CompositeIndex composite_index) const {
  DCHECK_GE(composite_index.lane_path_index, 0);
  DCHECK_LT(composite_index.lane_path_index, num_lane_paths());
  const auto entry = LanePathEntryIndexPoint(composite_index.lane_path_index);
  const auto exit = LanePathExitIndexPoint(composite_index.lane_path_index);
  DCHECK_GE(composite_index.lane_index, entry.first);
  DCHECK_LE(composite_index.lane_index, exit.first);
  const auto& lane_path = lane_paths()[composite_index.lane_path_index];
  return mapping::LanePath::LaneSegment(
      CompositeIndexToOverallIndex(composite_index),
      lane_path.lane_id(composite_index.lane_index),
      composite_index.lane_index == entry.first ? entry.second : 0.0,
      composite_index.lane_index == exit.first ? exit.second : 1.0,
      lane_path_end_s_[composite_index.lane_path_index] +
          lane_path.lane_segment(composite_index.lane_index).start_s,
      lane_path_end_s_[composite_index.lane_path_index] +
          lane_path.lane_segment(composite_index.lane_index).end_s);
}

const CompositeLanePath::TransitionInfo CompositeLanePath::kDefaultTransition{
    .overlap_length = CompositeLanePath::TransitionInfo::kDefaultOverlapLength,
    .transition_point_fraction =
        CompositeLanePath::TransitionInfo::kDefaultTransitionPointFraction,
    .optimized = false,
    .lc_left = true,
    .lc_section_id = mapping::kInvalidSectionId};

}  // namespace planning
}  // namespace e2e_noa
