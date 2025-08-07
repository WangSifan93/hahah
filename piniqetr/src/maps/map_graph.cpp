#include "maps/map_graph.h"

#include "common/log.h"

namespace ad_e2e {
namespace planning {
MapGraph::MapGraph(const LaneConstPtr& lane_ptr, const double successor_range,
                   const double predecessor_range, const MapConstPtr& map_ptr)
    : successor_range_(successor_range),
      predecessor_range_(predecessor_range),
      current_lane_(lane_ptr) {
  BuildGraph(current_lane_, map_ptr);
}

void MapGraph::BuildGraph(const LaneConstPtr& lane_ptr,
                          const MapConstPtr& map_ptr) {
  if (lane_ptr == nullptr || !lane_ptr->IsValid()) {
    LWARN("Invalid lane.");
    return;
  }
  std::list<LaneConstPtr> history_lanes;
  successor_lane_sequences_.clear();
  predecessor_lane_sequences_.clear();
  int32_t uturn_count = 0;
  BuildSuccessorLaneSequence(0.0, 0.0, map_ptr, current_lane_, &history_lanes,
                             uturn_count);
  history_lanes.clear();
  BuildPredecessorLaneSequence(0.0, 0.0, map_ptr, current_lane_,
                               &history_lanes);
  LDEBUG("Success build lane sequece for lane_id: %s",
         current_lane_->id().c_str());
}

void MapGraph::BuildSuccessorLaneSequence(
    const double accumulated_lane_s, const double start_lane_s,
    const MapConstPtr& map_ptr, const LaneConstPtr& lane_ptr,
    std::list<LaneConstPtr>* const history_lane_ptrs, uint32_t uturn_count) {
  bool push_flag = false;
  if (uturn_count <= 1) {
    history_lane_ptrs->push_back(lane_ptr);
    push_flag = true;
  }
  if (accumulated_lane_s - start_lane_s >= successor_range_ ||
      lane_ptr->next_lane_ids().size() < 1 || uturn_count >= 2) {
    successor_lane_sequences_.push_back(*history_lane_ptrs);
  } else {
    for (const auto& successor_lane_id : lane_ptr->next_lane_ids()) {
      LaneConstPtr successor_lane_ptr = map_ptr->GetLaneById(successor_lane_id);
      if (successor_lane_ptr == nullptr || !successor_lane_ptr->IsValid()) {
        continue;
      }
      double successor_accumulated_s = accumulated_lane_s +
                                       successor_lane_ptr->curve_length() -
                                       start_lane_s;
      LDEBUG(
          "current_lane id:%s, successor lane  id:%s successor_accumulated_s = "
          "%lf",
          lane_ptr->id().c_str(), successor_lane_id.c_str(),
          successor_accumulated_s);
      BuildSuccessorLaneSequence(successor_accumulated_s, 0.0, map_ptr,
                                 successor_lane_ptr, history_lane_ptrs,
                                 uturn_count);
    }
  }
  if (push_flag) {
    history_lane_ptrs->pop_back();
  }
}

void MapGraph::BuildPredecessorLaneSequence(
    const double accumulated_lane_s, const double end_lane_s,
    const MapConstPtr& map_ptr, const LaneConstPtr& lane_ptr,
    std::list<LaneConstPtr>* const history_lane_ptrs) {
  history_lane_ptrs->push_back(lane_ptr);
  if (accumulated_lane_s + end_lane_s >= predecessor_range_ ||
      lane_ptr->pre_lane_ids().size() < 1) {
    predecessor_lane_sequences_.push_back(*history_lane_ptrs);
  } else {
    for (const auto& predecessor_lane_id : lane_ptr->pre_lane_ids()) {
      LaneConstPtr predecessor_lane_ptr =
          map_ptr->GetLaneById(predecessor_lane_id);
      if (predecessor_lane_ptr == nullptr || !predecessor_lane_ptr->IsValid()) {
        continue;
      }
      double predecessor_accumulated_s = accumulated_lane_s + end_lane_s;
      double length = predecessor_lane_ptr->curve_length();
      LDEBUG(
          "current_lane id:%s, predecessor lane  id:%s "
          "predecessor_accumulated_s = %lf",
          lane_ptr->id().c_str(), predecessor_lane_id.c_str(),
          predecessor_accumulated_s);
      BuildPredecessorLaneSequence(predecessor_accumulated_s, length, map_ptr,
                                   predecessor_lane_ptr, history_lane_ptrs);
    }
  }
  history_lane_ptrs->pop_back();
}

}  // namespace planning
}  // namespace ad_e2e
