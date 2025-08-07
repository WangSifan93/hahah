#include "maps/map_container.h"

#include "async/parallel_for.h"

namespace ad_e2e {
namespace planning {

MapContainer::MapContainer() {}

void MapContainer::Init() { LOG(WARNING) << "MapContainer Init"; }

void MapContainer::InsertMap(const MapInfo& map_info) {
  latest_element_.map_ptr = std::make_shared<Map>(map_info);
  GenerateMapScene();
  GenerateVectorMap();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    ready_element_ = latest_element_;
  }
}

ErrorCode MapContainer::GetMap(MapElement* const map_element) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!ready_element_.IsValid()) {
    return ErrorCode::PLANNING_MAP_FAILED;
  }
  *map_element = ready_element_;
  return ErrorCode::PLANNING_OK;
}

void MapContainer::GenerateMapScene() {
  auto map_scene_ptr = std::make_shared<MapScene>();
  const auto& map_ptr = latest_element_.map_ptr;
#if 0
  std::mutex mutex;
  e2e_noa::MapParallelFor(
      map_ptr->lanes().begin(), map_ptr->lanes().end(),
      [&](LaneConstPtr lane_ptr) {
        if (!lane_ptr->IsValid()) return;
        MapGraphConstPtr map_graph = std::make_shared<MapGraph>(
            lane_ptr, FLAGS_ad_e2e_planning_forward_lane_sequence_length,
            FLAGS_ad_e2e_planning_backward_lane_sequence_length, map_ptr);
        {
          std::lock_guard<std::mutex> lock(mutex);
          map_scene_ptr->Insert(lane_ptr->id(), map_graph);
        }
      });
#endif
  latest_element_.map_scene_ptr = map_scene_ptr;
}

void MapContainer::GenerateVectorMap() {
  VectorMapPtr vector_map = std::make_shared<VectorMap>();
  const auto& map_ptr = latest_element_.map_ptr;
  vector_map->set_timestamp(map_ptr->timestamp());
#if 0
  e2e_noa::MapParallelFor(polyline_extractors_.begin(), polyline_extractors_.end(),
                     [&](ad_e2e::planning::PolylineExtractor* extractor) {
                       extractor->ExtractPolyline(map_ptr, vector_map);
                     });
#endif
  latest_element_.vector_map_ptr = vector_map;
}

}  // namespace planning
}  // namespace ad_e2e
