#ifndef MAPS_MAP_CONTAINER_H
#define MAPS_MAP_CONTAINER_H

#include <atomic>
#include <memory>
#include <vector>

#include "common/planning_error.h"
#include "common/type_def.h"
#include "maps/map_element.h"

namespace ad_e2e {
namespace planning {

class MapContainer {
 public:
  MapContainer();
  ~MapContainer() = default;

  void Init();
  void InsertMap(const MapInfo& map_info);
  ErrorCode GetMap(MapElement* const map_element);

 private:
  void GenerateMapScene();
  void GenerateVectorMap();

 private:
  std::mutex map_mutex_;
  MapElement ready_element_;
  MapElement latest_element_;
};

}  // namespace planning
}  // namespace ad_e2e

#endif
