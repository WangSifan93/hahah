#include "common/driving_map_topo.h"

#include <utility>

#include "absl/hash/hash.h"

namespace e2e_noa::planning {

DrivingMapTopo::DrivingMapTopo(
    std::vector<Lane> lanes, std::vector<mapping::ElementId> starting_lane_ids)
    : lanes_(std::move(lanes)),
      starting_lane_ids_(std::move(starting_lane_ids)) {
  for (size_t i = 0; i < lanes_.size(); ++i) {
    id_index_map_.emplace(lanes_[i].id, i);
  }
}

const DrivingMapTopo::Lane* DrivingMapTopo::GetLaneById(
    mapping::ElementId id) const {
  auto it = id_index_map_.find(id);

  if (it == id_index_map_.end()) {
    return nullptr;
  }

  return &lanes_[it->second];
}

}  // namespace e2e_noa::planning
