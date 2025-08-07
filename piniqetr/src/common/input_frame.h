#ifndef AD_E2E_PLANNING_COMMON_INPUT_FRAME_H
#define AD_E2E_PLANNING_COMMON_INPUT_FRAME_H
#include "common/planning_macros.h"
#include "common/type_def.h"
#include "dataflow/callback/callback.h"
#include "dataflow/module/module.h"
#include "dataflow/module/module_option.h"
#include "dataflow/module/proc.h"
#include "maps/map_element.h"
#include "message/proto/proto_serializer.hpp"
#include "messages/cheryos/vehicle/vehicle_chassis_pt.pb.h"
#include "messages/common/heartbeat.pb.h"
#include "messages/localization/localization.pb.h"
#include "messages/map_fusion/map_fusion.pb.h"
#include "messages/map_service/zdrive_map.pb.h"
#include "messages/planning/driving/ads_adptrout.pb.h"
#include "messages/planning/driving/ads_fct.pb.h"
#include "messages/planning/driving/e2e_planning_output.pb.h"
#include "messages/prediction/prediction.pb.h"

namespace zark {

namespace prediction {
namespace proto {
using PredictionObjectsMsg = hobot::message::ProtoMsg<PredictionObjects>;
using PredictionObjectsMsgSerializer =
    hobot::message::ProtobufSerializer<PredictionObjects>;
}  // namespace proto
}  // namespace prediction

namespace localization {
using LocalizationInfoMsg = hobot::message::ProtoMsg<LocalizationInfo>;
using LocalizationInfoMsgSerializer =
    hobot::message::ProtobufSerializer<LocalizationInfo>;
}  // namespace localization

namespace zdrive_map {
using ZarkMapMsg = hobot::message::ProtoMsg<ZarkMap>;
using ZarkMapMsgSerializer = hobot::message::ProtobufSerializer<ZarkMap>;
}  // namespace zdrive_map

namespace mapfusion {
using MapLocalizationInfoMsg = hobot::message::ProtoMsg<MapLocalizationInfo>;
using MapLocalizationInfoMsgSerializer =
    hobot::message::ProtobufSerializer<MapLocalizationInfo>;
}  // namespace mapfusion

namespace cheryos {
using VehiclePowerAndChassisMsgProto =
    hobot::message::ProtoMsg<VehiclePowerAndChassisMsg>;
using VehiclePowerAndChassisMsgProtoSerializer =
    hobot::message::ProtobufSerializer<VehiclePowerAndChassisMsg>;
}  // namespace cheryos

namespace ads_fct {
using FctMsgProto = hobot::message::ProtoMsg<FCT_Outputs>;
using FctMsgProtoSerializer = hobot::message::ProtobufSerializer<FCT_Outputs>;
}  // namespace ads_fct

namespace e2e_noa {
using PLN_OutputsMsg = hobot::message::ProtoMsg<PlanningResult>;
using PLN_OutputsMsgSerializer =
    hobot::message::ProtobufSerializer<PlanningResult>;
}  // namespace e2e_noa

namespace common {
using HeartBeatMsg = hobot::message::ProtoMsg<HeartBeat>;
using HeartBeatMsgSerializer = hobot::message::ProtobufSerializer<HeartBeat>;
}  // namespace common

using MapMsgType = zdrive_map::ZarkMapMsg;
using PredictionObjs = prediction::proto::PredictionObjectsMsg;
using OdometryMsgType = localization::LocalizationInfoMsg;
using MapFusionMsgType = mapfusion::MapLocalizationInfoMsg;
using VehicleStatusType = cheryos::VehiclePowerAndChassisMsgProto;
using FctMsgType = ads_fct::FctMsgProto;
}  // namespace zark

namespace ad_e2e {
namespace planning {

struct EgoInfo {
  int64_t seq_num{0};
  double stamp{0};
  Point2d pos;
  Point2d vel;
  Point2d acc;
  double heading{0};
  double steering_angle{0};
  double length{0};
  double width{0};
  double height{0};
};

struct DynamicObstacleInfo {
  int64_t seq_num{0};
  std::string id;
  ObstacleType type{OBJECT_UNKNOWN};
  double stamp{0};
  double length{0};
  double width{0};
  double height{0};
  double heading{0};
  double yaw_rate{0};
  double life_time{0};
  double confidence{0};
  Point2d pos;
  Point2d vel;
  Point2d acc;
  std::vector<Point2d> polygon;
  int8_t obstacle_state;
  int8_t fusion_type{0};
  std::vector<ObstacleLightType> obstacle_lights;

  std::vector<double> obstacle_lights_conf;
};

struct StaticObstacleInfo {
  double stamp{0};
  int64_t seq_num{0};
  std::string id;
  Point2d pos;
  Point2d vel;
  Point2d acc;
  double heading{0};
  double length{0};
  double width{0};
  double height{0};
  ObstacleType type{OBJECT_UNKNOWN};
  int8_t obstacle_state;
  int8_t fusion_type{0};
  std::vector<Point2d> polygon;
};

struct ObstacleInfo {
  int64_t seq_num{0};
  int8_t coordinate_type;

  std::vector<DynamicObstacleInfo> dynamic_obs;
  std::vector<StaticObstacleInfo> static_obs;
};

struct NavigationInfo {
  int64_t seq_num{0};
  std::vector<TurnInfo> turn_infos;
  std::vector<RoadClassInfo> road_class_infos;
  std::vector<LaneGroupInfo> lane_group_infos;
  std::vector<FormOfWayInfo> form_of_way_infos;
  bool has_navigation = false;
};

struct PlanningInputFrame {
  DECLARE_PTR(PlanningInputFrame);

  double last_odometry_timestamp{0};
  double last_msg_timestamp{0};

  std::shared_ptr<zark::OdometryMsgType> odometry = nullptr;
  std::shared_ptr<zark::MapFusionMsgType> mapfusion = nullptr;
  std::shared_ptr<zark::MapMsgType> map_input_data = nullptr;
  std::shared_ptr<zark::FctMsgType> behavior = nullptr;
  MapPtr map_ptr = nullptr;
  std::shared_ptr<zark::PredictionObjs> prediction = nullptr;
  std::shared_ptr<zark::VehicleStatusType> vehicle_status = nullptr;

  bool use_hd_map = false;
  uint64_t seq_num = 0;
};

enum Result {
  RESULT_INACTIVE = -1,
  RESULT_OK = 0,
  RESULT_FAIL = 1,
  RESULT_FAIL_GET_MAP = 2,
  RESULT_FAIL_GET_TRAFFIC = 3,
  RESULT_FAIL_GET_VEHICLE = 4,
  RESULT_FAIL_GET_LOCALIZATION = 5,
  RESULT_FAIL_UPDATE_MANAGER = 6,
  RESULT_FAIL_MANAGER_STAMP = 7,
  RESULT_FAIL_MANAGER_NO_MAP = 8,
  RESULT_FAIL_MANAGER_TARGET_LANE = 9,
  RESULT_FAIL_MANAGER_UPDATE_REFLINE = 10,
  RESULT_FAIL_MANAGER_CURRENT_LANE = 11,
  RESULT_FAIL_MANAGER_PLANNER = 12
};

struct planning_res_state {
  Result result;
  uint8_t fail_reason;
};

using planning_result_type =
    std::pair<zark::e2e_noa::PLN_OutputsMsg, planning_res_state>;
using planning_result_msg_type = zark::e2e_noa::PLN_OutputsMsg;

}  // namespace planning
}  // namespace ad_e2e

#endif
