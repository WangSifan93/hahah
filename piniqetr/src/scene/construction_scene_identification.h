#ifndef SCENE_CONSTRUCTION_SCENE_IDENTIFICATION_H_
#define SCENE_CONSTRUCTION_SCENE_IDENTIFICATION_H_
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "behavior.pb.h"
#include "common/obstacle.h"
#include "common/path/path.h"
#include "decision_exploration/decision_exploration_input.h"
#include "object/planner_object_manager.h"
#include "plan/planner_semantic_map_manager.h"

namespace e2e_noa::planning {
using LaneConstPtr = ad_e2e::planning::LaneConstPtr;
using LaneSequencePtr = ad_e2e::planning::LaneSequencePtr;
using StationaryObstacle = ad_e2e::planning::StationaryObstacle;
using SLBoundary = ad_e2e::planning::SLBoundary;
using LaneSeqInfoPtr = ad_e2e::planning::LaneSeqInfo;
struct ConstructionSceneIdentificationInput {
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const VehicleParamsProto* vehicle_param = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;

  const std::optional<double> lcc_cruising_speed_limit = std::nullopt;
  const std::optional<Behavior_FunctionId> function_id = std::nullopt;
  const std::optional<int> tunnel_status = std::nullopt;
  const PlannerObjectController& obj_mgr;
  const LaneSequencePtr target_lane_seq = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
};
struct BlockerSceneIdentificationInput {
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const VehicleParamsProto* vehicle_param = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;

  const std::optional<double> lcc_cruising_speed_limit = std::nullopt;
  const std::optional<Behavior_FunctionId> function_id = std::nullopt;
  const std::optional<int> tunnel_status = std::nullopt;
  const PlannerObjectController& obj_mgr;
  const LaneSequencePtr target_lane_seq = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
  const LaneSequencePtr target_lane_seq_blocker = nullptr;
};

bool RunConstructionSceneIdentification(
    const ConstructionSceneIdentificationInput& input);
bool RunBlockerSceneIdentification(
    const BlockerSceneIdentificationInput& blockerinput);
bool ConstructionBlockerLcUUTrigger(
    const ConstructionSceneIdentificationInput& input);
bool BlockerLcUUTrigger(const BlockerSceneIdentificationInput& blockerinput,
                        double dist_to_junction);
bool ClassifyObstaclesByPositionAndSetSLBoundary(
    const LaneSequencePtr& lane_seq, std::vector<StationaryObstacle>& fsd_obs,
    std::vector<StationaryObstacle>& current_lane_obstacles,
    const double start_point_s);
void ConvertStObstaclesToFsdObs(
    absl::Span<const PlannerObject* const> stationary_objects,
    std::vector<StationaryObstacle>& fsd_obs);
double GetLeaderCarSOffset(const LaneSequencePtr& lane_seq,
                           const ObjectVector<PlannerObject>& obstacles,
                           const double& start_offset);
}  // namespace e2e_noa::planning

#endif
