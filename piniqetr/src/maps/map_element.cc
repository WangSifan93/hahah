

#include "map_element.h"

namespace ad_e2e {
namespace planning {

bool MapElement::IsValid() const {
  if (map_ptr == nullptr) {
    LOG(ERROR) << "[MapInfo] map_ptr is null!";
  } else {
    if (!map_ptr->IsValid()) {
      LOG(ERROR) << "[MapInfo] map_ptr is invalid!";
    }
    if (map_ptr->seq() == -1) {
      LOG(ERROR) << "[MapInfo] invalid map sequence!";
    }
  }
#if 0
  if (map_scene_ptr == nullptr) {
    LOG(ERROR) << "[MapInfo] map_scene_ptr is null!";
  }
  if (vector_map_ptr == nullptr) {
    LOG(ERROR) << "[MapInfo] vector_map_ptr is null";
  }
#endif
  return map_ptr != nullptr && map_scene_ptr != nullptr &&
         vector_map_ptr != nullptr && map_ptr->IsValid();
}

void MapElement::reset() {
  map_ptr = nullptr;
  map_scene_ptr = nullptr;
  vector_map_ptr = nullptr;
}

}  // namespace planning
}  // namespace ad_e2e
