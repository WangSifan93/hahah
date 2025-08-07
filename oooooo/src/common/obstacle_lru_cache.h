#ifndef AD_E2E_PLANNING_COMMON_OBSTACLE_LRU_CACHE_H
#define AD_E2E_PLANNING_COMMON_OBSTACLE_LRU_CACHE_H

#include <string>

#include "common/obstacle.h"
#include "util/lru_cache_async.h"

namespace ad_e2e {
namespace planning {

using ObstacleLRUCache = LRUCacheAsync<Obstacle, std::string>;
using ObstacleLRUCachePtr = std::shared_ptr<ObstacleLRUCache>;
using ObstacleMapPtr =
    std::shared_ptr<std::unordered_map<std::string, ObstaclePtr>>;

}  // namespace planning
}  // namespace ad_e2e

#endif
