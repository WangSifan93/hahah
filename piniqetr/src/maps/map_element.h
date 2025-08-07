#ifndef MAPS_MAP_ELEMENT_H
#define MAPS_MAP_ELEMENT_H
#include "maps/map.h"
#include "maps/map_scene.h"
#include "maps/vector_map/vector_map.h"

namespace ad_e2e {
namespace planning {

struct MapElement {
  MapPtr map_ptr = nullptr;
  MapScenePtr map_scene_ptr = nullptr;
  VectorMapPtr vector_map_ptr = nullptr;

  bool IsValid() const;
  void reset();
};
}  // namespace planning
}  // namespace ad_e2e

#endif
