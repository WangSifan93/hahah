#ifndef E2E_NOA_PLANNING_NODES_PLANNER_CITY_MSG_PROXY_H
#define E2E_NOA_PLANNING_NODES_PLANNER_CITY_MSG_PROXY_H

#include <memory>
#include <variant>
#include <vector>

#include "async/thread_pool.h"
#include "common/input_frame.h"
#include "common/planner_status.h"
#include "common/type_def.h"
#include "localization.pb.h"
#include "nodes/behavior_container.h"
#include "plan/planner_state.h"
#include "plan/planner_world.h"
#include "scene/scene_understanding.h"

namespace e2e_noa {
namespace planning {

using PlanningInputFrame = ad_e2e::planning::PlanningInputFrame;

struct PredictionResult {
  e2e_noa::ObjectsProto objects_proto;
  e2e_noa::ObjectsPredictionProto objects_prediction_proto;
};

std::unique_ptr<PlannerWorld> AdaptPlannerWorld(
    const PlanningInputFrame* input_frame,
    const e2e_noa::PlannerParamsProto& planner_params,
    const e2e_noa::VehicleParamsProto& vehicle_params,
    absl::Time predicted_plan_time, e2e_noa::WorkerThreadManager* thread_pool,
    PlannerState* planner_state);

namespace {
using ObstacleType = zark::sensor_fusion::Object_ClassSubType;
using IntentType = zark::prediction::proto::IntentTrajectory_DriveStatus;
using TrafficLightStatusMap = ad_e2e::planning::TrafficLightStatusMap;
using LaneConstPtr = ad_e2e::planning::LaneConstPtr;

using PoseProto = e2e_noa::PoseProto;
using ObjectType = e2e_noa::ObjectType;
using ObjectsProto = e2e_noa::ObjectsProto;
using ObjectsPredictionProto = e2e_noa::ObjectsPredictionProto;
using TrajectoryIntention = e2e_noa::TrajectoryIntention;
using PNPInfos = e2e_noa::PNPInfos;
using PlannerParamsProto = e2e_noa::PlannerParamsProto;
using VehicleParamsProto = e2e_noa::VehicleParamsProto;
using ApolloTrajectoryPointProto = e2e_noa::ApolloTrajectoryPointProto;

std::variant<PlannerStatus, std::unique_ptr<PlannerWorld>> ComputeMembers(
    const PlanningInputFrame* input_frame,
    const PlannerParamsProto& planner_params,
    const VehicleParamsProto& vehicle_params, absl::Time predicted_plan_time,
    WorkerThreadManager* thread_pool, PlannerState* planner_state);

PoseProto AdaptPoseProto(const zark::OdometryMsgType& loc_protomsg);
AutonomyStateProto AdaptAutonomyStatusProtocol(
    const zark::FctMsgType& behavior_msg);
Behavior AdaptBehavior(const zark::FctMsgType& behavior_msg,
                       const PlannerParamsProto& planner_params);
double GetFrontWheelAngle(const zark::VehicleStatusType& vehicle_status,
                          double steer_ratio);
ObjectType AdaptObjectType(ObstacleType msg_obstacle_type);
PredictionResult AdaptPredictionResult(const zark::PredictionObjs& pred_msg);

void ComputeObjectsStopTime(
    PredictionResult& prediction_result,
    absl::flat_hash_map<std::string, PlannerState::ObjectStopTimeResult>&
        object_stop_time_map);
TrajectoryIntention AdaptTrajectoryIntention(const IntentType& intention);
PNPInfos AdaptPNPInfo(const zark::PredictionObjs& prediction);
double GetLaneLength(const std::vector<Vec2d>& points);
SceneReasoningOutput RunSceneReasoningAndFillDebug(
    const PlannerSemanticMapManager& psmm,
    const TrafficLightStatusMap& tl_info_map,
    const PlannerObjectController& obj_mgr,
    const ObjectsPredictionProto& prediction,
    const ApolloTrajectoryPointProto& plan_start_point,
    WorkerThreadManager* thread_pool, ObjectHistoryController& obj_his_manager);

absl::StatusOr<std::vector<mapping::LanePath>> BuildLanePathsFormPsmm(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point);

bool CheckAlcResetCondition(ResetReasonProto::Reason reset_reason);

}  // namespace

}  // namespace planning
}  // namespace e2e_noa

#endif
