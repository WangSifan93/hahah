#ifndef PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
#define PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "common/driving_map_topo.h"
#include "maps/lane_point.h"
#include "math/vec.h"
#include "plan/discretized_path.h"
#include "plan/planner_semantic_map_manager.h"

namespace e2e_noa {
namespace planning {

enum class LaneSemantic {
  NONE = 0,
  ROAD = 1,
  INTERSECTION_LEFT_TURN = 2,
  INTERSECTION_RIGHT_TURN = 3,
  INTERSECTION_STRAIGHT = 4,
  INTERSECTION_UTURN = 5
};

struct RouteLaneInfo {
  ad_e2e::planning::LaneConstPtr lane = nullptr;
  double start_fraction = 0.0;
  double end_fraction = 1.0;
};

struct PathPointSemantic {
  mapping::LanePoint closest_lane_point;
  Vec2d closest_lane_point_pos;
  LaneSemantic lane_semantic = LaneSemantic::NONE;

  std::vector<int> lane_path_id_history;

  double deviation_distance;
  ad_e2e::planning::LaneConstPtr lane_info = nullptr;
};

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    const DiscretizedPath& path, int max_analyze_path_index,
    const PlannerSemanticMapManager& psmm,
    const DrivingMapTopo* driving_map_topo, WorkerThreadManager* thread_pool);

}  // namespace planning
}  // namespace e2e_noa

#endif
