#ifndef MAP_MAP_SCENE_H
#define MAP_MAP_SCENE_H

#include <list>
#include <unordered_map>

#include "maps/map_graph.h"

namespace ad_e2e {
namespace planning {

class MapScene {
 public:
  void Insert(const std::string& lane_id, const MapGraphConstPtr& map_graph);

  const MapGraphConstPtr GetMapGraphByLane(const std::string& lane_id) const;
  const std::unordered_map<std::string, MapGraphConstPtr>& MapGraph() const;

 private:
  std::unordered_map<std::string, MapGraphConstPtr> lane_map_graph_;
};

using MapScenePtr = std::shared_ptr<MapScene>;
using MapSceneConstPtr = std::shared_ptr<const MapScene>;
}  // namespace planning
}  // namespace ad_e2e

#endif
