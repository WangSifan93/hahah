#include "nodes/planner/city/msg_proxy.h"

#include <functional>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/cleanup/cleanup.h"
#include "behavior.pb.h"
#include "common/log_data.h"
#include "common/planning_macros.h"
#include "common/timer.h"
#include "decision_exploration/smooth_reference_line_builder.h"
#include "maps/local_map_builder.h"
#include "math/geometry/box2d.h"
#include "nodes/behavior_container.h"
#include "object/low_likelihood_filter.h"
#include "object/planner_object_manager_builder.h"
#include "object/predicted_spacetime_filter.h"
#include "object/reflected_object_in_proximity_filter.h"
#include "object/spacetime_state_filter.h"
#include "plan/planner_flags.h"
#include "plan/planner_main_loop_internal.h"
#include "planner_params.pb.h"
#include "planner_status.pb.h"
#include "route.pb.h"
#include "router/navi/route_navi_info.h"
#include "router/navi/route_navi_info_builder.h"
#include "router/route_sections.h"
#include "router/route_sections_info.h"
#include "router/route_util.h"
#include "turn_signal.pb.h"
#include "util/planner_status_macros.h"
#include "util/speed_util.h"
#include "util/status_macros.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

namespace {}

namespace {

void DrawPlanStartPtBox(const PlanStartPointInfo& start_point_info,
                        double length, double width) {
  auto ego_box2d =
      Box2d({start_point_info.start_point.path_point().x(),
             start_point_info.start_point.path_point().y()},
            start_point_info.start_point.path_point().theta(), length, width);
  std::vector<double> xs, ys, zs;
  xs.reserve(4);
  ys.reserve(4);
  zs.reserve(4);
  for (const auto& pt : ego_box2d.GetAllCorners()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
    zs.push_back(0.0);
  }
  Log2FG::Info3DMarkerV2("Ego Start Box", xs, ys, zs, Log2FG::kRed,
                         Log2FG::Marker3D::POLYGON);
}

}  // namespace

std::unique_ptr<PlannerWorld> AdaptPlannerWorld(
    const PlanningInputFrame* input_frame,
    const e2e_noa::PlannerParamsProto& planner_params,
    const e2e_noa::VehicleParamsProto& vehicle_params,
    absl::Time predicted_plan_time, e2e_noa::WorkerThreadManager* thread_pool,
    PlannerState* planner_state) {
  auto status_or_input =
      ComputeMembers(input_frame, planner_params, vehicle_params,
                     predicted_plan_time, thread_pool, planner_state);
  if (status_or_input.index() == 0) {
    planner_state->planner_status_code = PlannerStatusProto::INPUT_INCORRECT;
    LOG(ERROR) << "ComputeMembers failed: "
               << std::get<e2e_noa::planning::PlannerStatus>(status_or_input)
                      .ToString();
    Log2FG::LogMissionData("ComputeMembersStatus",
                           std::get<PlannerStatus>(status_or_input).ToString());
    return nullptr;
  }

  LOG(INFO) << __FUNCTION__ << "finished!";

  return std::move(std::get<1>(status_or_input));
}
namespace {
std::variant<PlannerStatus, std::unique_ptr<PlannerWorld>> ComputeMembers(
    const PlanningInputFrame* input_frame,
    const PlannerParamsProto& planner_params,
    const VehicleParamsProto& vehicle_params, absl::Time predicted_plan_time,
    WorkerThreadManager* thread_pool, PlannerState* planner_state) {
  DLOG(INFO) << "[PlannerInput] " << __FUNCTION__;
  Timer timer(__FUNCTION__);

  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();
  planner_state->seq_num = input_frame->seq_num;

  planner_state->planner_semantic_map_manager =
      std::make_shared<PlannerSemanticMapManager>(input_frame->map_ptr);

  const PoseProto pose_proto = AdaptPoseProto(*input_frame->odometry);
  Log2FG::LogVehData("Vehicle Info", pose_proto.DebugString());
  const AutonomyStateProto autonomy_state_proto =
      AdaptAutonomyStatusProtocol(*input_frame->behavior);
  if (autonomy_state_proto.autonomy_state() != AutonomyStateProto::AUTO_DRIVE &&
      autonomy_state_proto.autonomy_state() !=
          AutonomyStateProto::AUTO_STEER_ONLY) {
    planner_state->previous_trajectory.mutable_trajectory_point()->Clear();
    planner_state->prev_route_sections.Clear();
    planner_state->planner_status_code = PlannerStatusProto::OK;
  }

  const auto& previous_trajectory = planner_state->previous_trajectory;

  const bool aeb = false;
  const bool rerouted = false;
  const double front_wheel_angle =
      GetFrontWheelAngle(*input_frame->vehicle_status,
                         vehicle_params.vehicle_drive_params().steer_ratio());
  const Vec2d ego_pos = {pose_proto.pos_smooth().x(),
                         pose_proto.pos_smooth().y()};
  planner_state->input_ego_pose = {ego_pos, pose_proto.yaw()};

  planner_state->planner_status_code = PlannerStatusProto::OK;
  if (autonomy_state_proto.autonomy_state() == AutonomyStateProto::AUTO_DRIVE &&
      planner_state->planner_semantic_map_manager &&
      !planner_state->planner_semantic_map_manager->GetNearestLaneWithHeading(
          ego_pos, pose_proto.yaw(), 5.0, M_PI / 6.0)) {
    planner_state->planner_status_code = PlannerStatusProto::INPUT_INCORRECT;
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         "ego nearest lane empty.");
  }

  e2e_noa::Behavior current_behavior =
      AdaptBehavior(*input_frame->behavior, planner_params);
  Log2FG::LogMissionData("Behavior Func", current_behavior.DebugString());

  DriverAction::LaneChangeCommand lc_command_raw = DriverAction::LC_CMD_NONE;

  const auto behavior_action =
      input_frame->behavior->proto.fct_out_bus_alcinfosts()
          .fct_out_sts_drvrturnlvrcmd_edtlc();
  if (zark::ads_common::eTSLC_TurnSigLvrCmd::eTSLC_TurnLeft ==
          behavior_action ||
      zark::ads_common::eTSLC_TurnSigLvrCmd::eTSLC_LeftChange ==
          behavior_action) {
    lc_command_raw = DriverAction::LC_CMD_LEFT;
    if (ad_e2e::planning::Lane_PrepareRight == planner_state->output_lc_state ||
        ad_e2e::planning::Lane_ChangeRight == planner_state->output_lc_state) {
      lc_command_raw = DriverAction::LC_CMD_CANCEL;
    }
  } else if (zark::ads_common::eTSLC_TurnSigLvrCmd::eTSLC_TurnRight ==
                 behavior_action ||
             zark::ads_common::eTSLC_TurnSigLvrCmd::eTSLC_RightChange ==
                 behavior_action) {
    lc_command_raw = DriverAction::LC_CMD_RIGHT;
    if (ad_e2e::planning::Lane_PrepareLeft == planner_state->output_lc_state ||
        ad_e2e::planning::Lane_ChangeLeft == planner_state->output_lc_state) {
      lc_command_raw = DriverAction::LC_CMD_CANCEL;
    }
  }

  DriverAction::LaneChangeCommand new_lc_command = DriverAction::LC_CMD_NONE;
  if (planner_state->last_lc_command == DriverAction::LC_CMD_NONE &&
      lc_command_raw != DriverAction::LC_CMD_NONE) {
    new_lc_command = lc_command_raw;
    planner_state->lc_command_number = 0;

  } else if (planner_state->last_lc_command == lc_command_raw) {
    new_lc_command = lc_command_raw;
    if (lc_command_raw != DriverAction::LC_CMD_NONE) {
      planner_state->lc_command_number += 1;
    }

  } else {
    new_lc_command = DriverAction::LC_CMD_NONE;
    planner_state->lc_command_number = 0;

    if (lc_command_raw == DriverAction::LC_CMD_CANCEL) {
      new_lc_command = DriverAction::LC_CMD_CANCEL;
    }
  }

  Log2FG::LogMissionData("LaneChange",
                         DriverAction_LaneChangeCommand_Name(lc_command_raw));
  planner_state->last_lc_command = lc_command_raw;

  const auto& vehicle_status = input_frame->vehicle_status->proto;
  auto start_point_info = ComputeStPlanStartPoint(
      predicted_plan_time, previous_trajectory, pose_proto,
      autonomy_state_proto, planner_state->previous_autonomy_state,

      rerouted, aeb, front_wheel_angle,
      planner_params.spacetime_constraint_params(), vehicle_geom_params,
      vehicle_drive_params,
      (vehicle_status.tcu_and_pedal_data().pedl_override() == 0x1));
  VLOG(0) << "StartPointInfo" << start_point_info.DebugString();

  DrawPlanStartPtBox(start_point_info, vehicle_geom_params.length(),
                     vehicle_geom_params.width());
  Log2FG::LogVehData("StartPtInfo", start_point_info.start_point.DebugString());
  auto min_path_look_ahead_duration = GetStPathPlanLookAheadDuration(
      start_point_info, pose_proto, absl::Seconds(0.0), previous_trajectory);
  auto time_aligned_prev_traj = std::vector<ApolloTrajectoryPointProto>(
      CreatePreviousTrajectory(start_point_info.plan_time, previous_trajectory,
                               planner_params.spacetime_constraint_params(),
                               start_point_info.reset));

  mapping::LanePoint destination(mapping::ElementId(""), 1.0);

  const auto& psmm_maptr =
      planner_state->planner_semantic_map_manager->map_ptr();
  const double s_offset = psmm_maptr->route()->navi_start().s_offset;
  double route_start_fraction = 0.0;
  const auto& section_start =
      psmm_maptr->GetSectionById(psmm_maptr->route()->navi_start().section_id);

  if (s_offset > kPlanPassageKeepBehindLength) {
    if (std::fabs(section_start->topo_length()) < 1e-5) {
      return PlannerStatus(
          PlannerStatusProto::INPUT_INCORRECT,
          "current route start section (len == 0.0) too short! ");
    }

    route_start_fraction = (s_offset - kPlanPassageKeepBehindLength) /
                           section_start->topo_length();
  }
  RouteSections route_sections_from_current(
      route_start_fraction, 1.0, psmm_maptr->route()->GetNaviRouteIds(),
      destination);
  route_sections_from_current.set_topology_start_offset(
      psmm_maptr->route()->navi_start().s_offset);

  if (!rerouted && !route_sections_from_current.empty() &&
      !planner_state->prev_route_sections.empty()) {
    if (psmm_maptr->GetSectionById(
            planner_state->prev_route_sections.section_id(0)) == nullptr) {
      int index = -1;
      for (int i = 0; i < planner_state->prev_route_sections.size(); ++i) {
        if (psmm_maptr->GetSectionById(
                planner_state->prev_route_sections.section_id(i)) == nullptr) {
          continue;
        }
        index = i;
        break;
      }
      planner_state->prev_route_sections = RouteSections(
          index == -1 ? planner_state->prev_route_sections.start_fraction()
                      : 0.0,
          planner_state->prev_route_sections.end_fraction(),
          std::vector<mapping::SectionId>(
              planner_state->prev_route_sections.section_ids().begin() +
                  (index == -1 ? 0 : index),
              planner_state->prev_route_sections.section_ids().end()),
          planner_state->prev_route_sections.destination());
    }

    auto tailored_sections = AppendRouteSectionsToTail(
        planner_state->prev_route_sections, route_sections_from_current);

    if (!tailored_sections.ok()) {
      LOG(INFO) << "Planner prev route sections does not match route, reset";
      planner_state->prev_route_sections.Clear();
    } else {
      planner_state->prev_route_sections = std::move(tailored_sections).value();
    }
  }

  if (planner_state->prev_route_sections.empty()) {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        planner_state->prev_route_sections,
        BackwardExtendRouteSectionsFromPos(
            *planner_state->planner_semantic_map_manager,
            route_sections_from_current, ego_pos, kPlanPassageKeepBehindLength),
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);

    auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        *planner_state->planner_semantic_map_manager,
        planner_state->prev_route_sections, ego_pos);
    if (target_lane_path_or.ok()) {
      planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
    } else {
      planner_state->prev_route_sections.Clear();
      return PlannerStatus(
          PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED,
          target_lane_path_or.status().message());
    }
  } else {
    if (planner_state->prev_target_lane_path.IsEmpty()) {
      auto target_lane_path_or = FindClosestTargetLanePathOnReset(
          *planner_state->planner_semantic_map_manager,
          planner_state->prev_route_sections, ego_pos);
      if (target_lane_path_or.ok()) {
        planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
      }
    }
  }

  if (planner_state->prev_target_lane_path.IsEmpty()) {
    return PlannerStatus(PlannerStatusProto::PLANNER_STATE_INCOMPLETE,
                         "Prev target lane path empty.");
  }

  if (start_point_info.reset &&
      CheckAlcResetCondition(start_point_info.reset_reason)) {
    planner_state->preferred_lane_path = mapping::LanePath();
  }

  auto route_sections_proj_or = ProjectPointToRouteSections(
      *planner_state->planner_semantic_map_manager,
      planner_state->prev_route_sections, ego_pos,
      kMaxTravelDistanceBetweenFrames + kPlanPassageKeepBehindLength,
      kPlanPassageKeepBehindLength);
  if (!route_sections_proj_or.ok()) {
    return PlannerStatus(
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED,
        route_sections_proj_or.status().message());
  }
  auto [route_sections_from_start, route_sections_with_behind, ego_pos_proj] =
      std::move(route_sections_proj_or).value();
  planner_state->prev_route_sections = std::move(route_sections_with_behind);

  const double half_av_width =
      vehicle_params.vehicle_geometry_params().width() * 0.5;
  auto smooth_result_map_or = BuildSmoothedResultMapFromRouteSections(
      *planner_state->planner_semantic_map_manager, half_av_width,
      std::move(planner_state->smooth_result_map));
  if (smooth_result_map_or.ok()) {
    planner_state->smooth_result_map = std::move(smooth_result_map_or).value();
  }

  Log2FG::LogDataV0("lc_debug",
                    "new_lc_command: " + std::to_string(new_lc_command));
  std::optional<e2e_noa::PNPInfos> pnp_infos = std::nullopt;

  pnp_infos = AdaptPNPInfo(*input_frame->prediction);
  auto prediction_result = AdaptPredictionResult(*input_frame->prediction);
  ComputeObjectsStopTime(prediction_result,
                         planner_state->object_stop_time_map);
  auto objects_proto = ObjectsProto(std::move(prediction_result.objects_proto));
  const ObjectsPredictionProto& objects_prediction_proto =
      prediction_result.objects_prediction_proto;
  auto planner_objects = BuildPlannerObjects(
      &objects_proto, &objects_prediction_proto,
      ToUnixDoubleSeconds(start_point_info.plan_time), thread_pool);

  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold, false);
  const SpacetimeStateFilter spacetime_state_filter(pose_proto,
                                                    vehicle_geom_params);
  const PredictedSpacetimeFilter predicted_spacetime_filter(planner_objects);
  std::vector<const TrajectoryFilter*> filters = {&low_likelihood_filter,
                                                  &predicted_spacetime_filter};
  filters.push_back(&spacetime_state_filter);
  const ReflectedObjectInProximityFilter object_in_proximity_filter(
      pose_proto, vehicle_geom_params,
      FLAGS_planner_filter_reflected_object_distance);
  if (FLAGS_planner_filter_reflected_object_distance > 0.0) {
    filters.push_back(&object_in_proximity_filter);
  }
  PlannerObjectManagerBuilder obj_mgr_builder;
  obj_mgr_builder.set_planner_objects(std::move(planner_objects))
      .set_filters(filters);
  auto object_manager_or = obj_mgr_builder.Build(nullptr, thread_pool);
  if (!object_manager_or.ok()) {
    return PlannerStatus(PlannerStatusProto::OBJECT_MANAGER_FAILED,
                         object_manager_or.status().message());
  }
  auto object_manager = PlannerObjectController(std::move(*object_manager_or));
  auto st_traj_mgr =
      SpacetimeTrajectoryManager(absl::Span<const TrajectoryFilter*>{},
                                 object_manager.planner_objects(), thread_pool);

  const double cruising_speed_limit = current_behavior.cruising_speed_limit();
  std::optional<double> current_lane_speed_limit = std::nullopt;
  const auto current_lane = psmm_maptr->GetNearestLane(
      {start_point_info.start_point.path_point().x(),
       start_point_info.start_point.path_point().y()},
      10.0);
  if (current_lane != nullptr) {
    current_lane_speed_limit =
        planner_state->planner_semantic_map_manager->QueryLaneSpeedLimitById(
            current_lane->id());
  }
  constexpr double kDeltaSpeedThreshold = 0.1;
  if (autonomy_state_proto.autonomy_state() != AutonomyStateProto::AUTO_DRIVE) {
    planner_state->auto_drive_counter = 0;
  } else {
    planner_state->auto_drive_counter =
        std::min(10, planner_state->auto_drive_counter + 1);
  }
  if (planner_state->auto_drive_counter < 3) {
    planner_state->cruising_speed_limit_is_set = false;
  } else {
    if (!planner_state->cruising_speed_limit_is_set &&
        autonomy_state_proto.acc_state() == AutonomyStateProto::ACC_ACTIVE) {
      if (std::abs(cruising_speed_limit -
                   planner_state->previous_cruising_speed_limit) >
          kDeltaSpeedThreshold) {
        planner_state->cruising_speed_limit_is_set = true;
      }
    }
  }
  planner_state->previous_cruising_speed_limit = cruising_speed_limit;
  double cruising_speed_limit_set;
  if (planner_state->cruising_speed_limit_is_set ||
      !current_lane_speed_limit.has_value()) {
    cruising_speed_limit_set = cruising_speed_limit;
  } else {
    cruising_speed_limit_set =
        std::min(*current_lane_speed_limit, cruising_speed_limit);
  }

  const auto scene_reasoning_output = RunSceneReasoningAndFillDebug(
      *planner_state->planner_semantic_map_manager,
      planner_state->planner_semantic_map_manager->map_ptr()
          ->traffic_light_status_map(),
      object_manager, objects_prediction_proto, start_point_info.start_point,
      thread_pool, planner_state->object_history_manager);
  auto scene_reasoning = SceneOutputProto();
  absl::flat_hash_set<std::string> stalled_objects;
  absl::flat_hash_set<std::string> in_queue_objects;

  std::string stalled_obs_ids = "(";
  for (const auto& stalled_object :
       scene_reasoning_output.scene_output_proto.objects_annotation()) {
    stalled_objects.insert(stalled_object.object_id());
    stalled_obs_ids += stalled_object.object_id() + ",";
  }
  stalled_obs_ids += ")";
  Log2FG::LogMissionData("SceneReason-Stalled Obj", stalled_obs_ids);

  std::string in_queue_ids = "(";
  for (const auto& queue :
       scene_reasoning_output.scene_output_proto.traffic_waiting_queue()) {
    for (const auto& obj_str : queue.object_id()) {
      in_queue_objects.insert(obj_str);
      in_queue_ids += obj_str + ",";
    }
  }
  in_queue_ids += ")";
  Log2FG::LogMissionData("SceneReason-InQueue Obj", in_queue_ids);

  planner_state->ClearHMIInfo();
  for (const auto& object_pred : objects_prediction_proto.objects()) {
    const auto& object = object_pred.perception_object();

    if (ObjectType::OT_VEHICLE == object.type() ||
        ObjectType::OT_LARGE_VEHICLE == object.type()) {
      if (stalled_objects.find(object.id()) != stalled_objects.end()) {
        planner_state->stalled_cars.push_back(object.id());
      } else if (in_queue_objects.find(object.id()) != in_queue_objects.end()) {
        planner_state->in_queue_cars.push_back(object.id());
      }
    }
  }

#if 0
      
      bool btn_noa_on = vehicle_status.vehicle_body_status().button().noa_on() ==
                      Vehicle::RightKeyStatus::PRESSED;
      bool btn_ad_op_tb =
          vehicle_status.vehicle_body_status().button().ad_op_tb() ==
          Vehicle::TouchBarAdOpTbStatus::TO_LEFT_FAST_ROAD_CONFIRM;
      bool enable_tl_ok_btn = btn_noa_on || btn_ad_op_tb;
#endif
  bool override_passable =
      vehicle_status.tcu_and_pedal_data().pedl_override() == 0x1;

  planner_state->previous_autonomy_state = autonomy_state_proto;

  auto traffic_light_status_map = ad_e2e::planning::TrafficLightStatusMap(
      planner_state->planner_semantic_map_manager->map_ptr()
          ->traffic_light_status_map());
  std::unique_ptr<PlannerWorld> st_planner_input(new PlannerWorld{

      .planner_world_input = {
          .planner_params = &planner_params,
          .vehicle_params = &vehicle_params,
          .min_path_look_ahead_duration = min_path_look_ahead_duration,

          .start_point_info = std::move(start_point_info),
          .ego_pos = ego_pos,
          .prev_target_lane_path = &planner_state->prev_target_lane_path,
          .new_lc_command = new_lc_command,
          .auto_model = true,

          .route_sections_from_start = std::move(route_sections_from_start),
          .route_sections_from_current = std::move(route_sections_from_current),
          .st_traj_mgr = std::move(st_traj_mgr),
          .object_manager = std::move(object_manager),
          .stalled_objects = std::move(stalled_objects),
          .scene_reasoning =
              std::move(scene_reasoning_output.scene_output_proto),
          .time_aligned_prev_traj = std::move(time_aligned_prev_traj),
          .traffic_light_status_map = std::move(traffic_light_status_map),
          .consider_lane_change_gap = true,
          .selector_state = &planner_state->selector_state,
          .objects_proto = std::move(objects_proto),
          .pnp_infos = std::move(pnp_infos),
          .enable_tl_ok_btn = true,
          .override_passable = override_passable,
          .cruising_speed_limit = cruising_speed_limit_set,
          .behavior = current_behavior,
          .pose_proto = std::move(pose_proto)}});
  return st_planner_input;
}

PoseProto AdaptPoseProto(const zark::OdometryMsgType& loc_protomsg) {
  const auto& loc_msg = loc_protomsg.proto;
  PoseProto pose_proto;
  pose_proto.mutable_header()->set_timestamp(loc_msg.header().timestamp_ns() *
                                             1e-3);
  pose_proto.mutable_header()->set_channel_seq_number(
      loc_msg.header().sequence_num());
  pose_proto.set_timestamp(
      static_cast<double>(loc_msg.header().timestamp_ns()) * 1e-9);
  const auto& odom = loc_msg.local_pose();
  pose_proto.mutable_pos_smooth()->set_x(odom.pose().position().x());
  pose_proto.mutable_pos_smooth()->set_y(odom.pose().position().y());
  pose_proto.mutable_pos_smooth()->set_z(odom.pose().position().z());

  const Eigen::Vector3d ego_velocity = Eigen::Vector3d(
      odom.velocity().x(), odom.velocity().y(), odom.velocity().z());
  pose_proto.set_speed(ego_velocity[0]);

  Eigen::Quaterniond quat;
  quat.x() = odom.pose().quaternion().x();
  quat.y() = odom.pose().quaternion().y();
  quat.z() = odom.pose().quaternion().z();
  quat.w() = odom.pose().quaternion().w();
  quat = quat.normalized();
  const Eigen::Matrix3d R = quat.toRotationMatrix();
  const double yaw = odom.local_orientation().yaw();
  pose_proto.set_yaw(yaw);
  pose_proto.set_pitch(0.0);
  pose_proto.set_roll(0.0);

  const Eigen::Vector3d odom_velocity = R * ego_velocity;
  pose_proto.mutable_vel_smooth()->set_x(odom_velocity[0]);
  pose_proto.mutable_vel_smooth()->set_y(odom_velocity[1]);
  pose_proto.mutable_vel_smooth()->set_z(odom_velocity[2]);

  const Eigen::Vector3d ego_acc_body(odom.acceleration().x(),
                                     odom.acceleration().y(),
                                     odom.acceleration().z());
  const Eigen::Vector3d odom_acc = R * ego_acc_body;
  pose_proto.mutable_accel_smooth()->set_x(odom_acc[0]);
  pose_proto.mutable_accel_smooth()->set_y(odom_acc[1]);
  pose_proto.mutable_accel_smooth()->set_z(odom_acc[2]);
  const Eigen::Vector3d ego_angular_rate_body(0.0, 0.0, odom.yawrate());
  const Eigen::Vector3d odom_angular_rate = R * ego_angular_rate_body;
  pose_proto.mutable_ar_smooth()->set_x(odom_angular_rate[0]);
  pose_proto.mutable_ar_smooth()->set_y(odom_angular_rate[1]);
  pose_proto.mutable_ar_smooth()->set_z(odom_angular_rate[2]);

  pose_proto.mutable_vel_body()->set_x(ego_velocity[0]);
  pose_proto.mutable_vel_body()->set_y(ego_velocity[1]);
  pose_proto.mutable_vel_body()->set_z(ego_velocity[2]);
  pose_proto.mutable_accel_body()->set_x(ego_acc_body[0]);
  pose_proto.mutable_accel_body()->set_y(ego_acc_body[1]);
  pose_proto.mutable_accel_body()->set_z(ego_acc_body[2]);
  pose_proto.mutable_ar_body()->set_x(ego_angular_rate_body[0]);
  pose_proto.mutable_ar_body()->set_y(ego_angular_rate_body[1]);
  pose_proto.mutable_ar_body()->set_z(ego_angular_rate_body[2]);
  return pose_proto;
}

AutonomyStateProto AdaptAutonomyStatusProtocol(
    const zark::FctMsgType& behavior_msg) {
  const auto& behavior = behavior_msg.proto;
  AutonomyStateProto now_autonomy_state;
  now_autonomy_state.set_autonomy_state(
      AutonomyStateProto::READY_TO_AUTO_DRIVE);
  const auto& function_id =
      behavior.fct_out_bus_lineinfo().fct_out_sts_activefunction_eaf();
  if (function_id == zark::ads_common::eAF_ActiveFunction::cNOP ||
      function_id == zark::ads_common::eAF_ActiveFunction::hNOP ||
      function_id == zark::ads_common::eAF_ActiveFunction::LCC) {
    now_autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
  }
  now_autonomy_state.set_acc_state(AutonomyStateProto::ACC_NOT_ACTIVE);
  const auto& acc_state =
      behavior.fct_out_bus_accinfosts().fct_out_indx_accsyssts_u8();
  if (acc_state == zark::ads_common::eACCReqStsActive) {
    now_autonomy_state.set_acc_state(AutonomyStateProto::ACC_ACTIVE);
  }
  return now_autonomy_state;
}

Behavior AdaptBehavior(const zark::FctMsgType& behavior_msg,
                       const PlannerParamsProto& planner_params) {
  Behavior current_behavior;
  const auto& behavior = behavior_msg.proto;

  Behavior_FunctionId function_id = Behavior::NONE;
  const auto& msg_function =
      behavior.fct_out_bus_lineinfo().fct_out_sts_activefunction_eaf();
  switch (msg_function) {
    case zark::ads_common::eAF_ActiveFunction::cNOP:
      function_id = Behavior::CITY_NOA;
      break;
    case zark::ads_common::eAF_ActiveFunction::hNOP:
      function_id = Behavior::NOA;
      break;
    case zark::ads_common::eAF_ActiveFunction::LCC:
      function_id = Behavior::LKA;
      break;
    case zark::ads_common::eAF_ActiveFunction::LCCPro:
      function_id = Behavior::LKA_PLUS;
      break;
    default:
      function_id = Behavior::NONE;
      break;
  }
  current_behavior.set_function_id(function_id);
#if 0
  
  
  auto tunnel_status = behavior.chart_debug().reserve1();
  current_behavior.set_tunnel(tunnel_status);
  bool auto_navi_lc_enable_status =
      (behavior.auto_navi_lc_enable_status() ==
       behavior_idls::idls::Behavior_Constants::AUTO_NAVI_LC_ENABLE);
  bool traffic_light_func_enable = behavior.traffic_light_func_enable();
  current_behavior.set_auto_navi_lc_enable_status(auto_navi_lc_enable_status);
  current_behavior.set_traffic_light_func_enable(traffic_light_func_enable);
#endif

  bool system_break_stop = false;
  if (function_id == Behavior::CITY_NOA || function_id == Behavior::NOA) {
    system_break_stop =
        (zark::ads_common::eNOPSS_NOPSysSts::eNOPSS_SafeStop ==
         behavior.fct_out_bus_nopinfosts().fct_out_sts_hnopsyssts_enopss());
  } else if (function_id == Behavior::LKA) {
    system_break_stop =
        (zark::ads_common::eLACS_LatCtrlSts::LACS_SafeStop ==
         behavior.fct_out_bus_lccinfosts().fct_out_sts_lccsyssts_elacs());
  } else {
    system_break_stop = false;
  }
  current_behavior.set_system_break_stop(system_break_stop);

  double dynamic_headway =
      behavior.fct_out_bus_accinfosts().fct_out_t_accacttmdist_sg();
  current_behavior.set_dynamic_headway(dynamic_headway);

  double set_speed_kph_disp =
      behavior.fct_out_bus_accinfosts().fct_out_v_accactsetspd_sg();
  PiecewiseLinearFunction<double> user_set_speed_bias_plf(
      PiecewiseLinearFunctionFromProto(planner_params.speed_planning_params()
                                           .speed_limit_params()
                                           .user_set_speed_bias_plf()));
  const double cruising_speed_limit =
      VelocityCoordinateTransform(user_set_speed_bias_plf, set_speed_kph_disp);

  current_behavior.set_cruising_speed_limit(cruising_speed_limit);

  return current_behavior;
}

double GetFrontWheelAngle(const zark::VehicleStatusType& vehicle_status,
                          double steer_ratio) {
  const double front_wheel_angle =
      vehicle_status.proto.eps_data().steering_angle() / steer_ratio;
  return front_wheel_angle * ad_e2e::planning::Constants::DEG2RAD;
}

TrajectoryIntention AdaptTrajectoryIntention(const IntentType& intention) {
  TrajectoryIntention trajectory_intention =
      TrajectoryIntention::INTENTION_UNKNOWN;
  if (intention == IntentType::IntentTrajectory_DriveStatus_STATUS_LANE_KEEP) {
    trajectory_intention = TrajectoryIntention::INTENTION_PARALLEL;
  } else if (intention ==
             IntentType::IntentTrajectory_DriveStatus_STATUS_LANE_LEFT_CHANGE) {
    trajectory_intention = TrajectoryIntention::INTENTION_LC_LEFT;
  } else if (intention ==
             IntentType::
                 IntentTrajectory_DriveStatus_STATUS_LANE_RIGHT_CHANGE) {
    trajectory_intention = TrajectoryIntention::INTENTION_LC_RIGHT;
  }
  return trajectory_intention;
}

ObjectType AdaptObjectType(ObstacleType msg_obstacle_type) {
  ObjectType pnc_obstacle_type = OT_UNKNOWN_MOVABLE;
  switch (msg_obstacle_type) {
    case ObstacleType::Object_ClassSubType_ST_UNKNOWN:
    case ObstacleType::Object_ClassSubType_ST_AUTOGATE_CLOSE:
    case ObstacleType::Object_ClassSubType_ST_AUTOGATE_OPEN:
    case ObstacleType::Object_ClassSubType_ST_BIGANIMAL:
    case ObstacleType::Object_ClassSubType_ST_SMALLANIMAL:
    case ObstacleType::Object_ClassSubType_ST_PETS_CAT:
    case ObstacleType::Object_ClassSubType_ST_PETS_DOG:
      pnc_obstacle_type = OT_UNKNOWN_MOVABLE;
      break;
    case ObstacleType::Object_ClassSubType_ST_VAN:
    case ObstacleType::Object_ClassSubType_ST_CAR:
      pnc_obstacle_type = OT_VEHICLE;
      break;
    case ObstacleType::Object_ClassSubType_ST_TRUCK:
    case ObstacleType::Object_ClassSubType_ST_BUS:
    case ObstacleType::Object_ClassSubType_ST_CONSTRUCTION_VEHICLE:
      pnc_obstacle_type = OT_LARGE_VEHICLE;
      break;
    case ObstacleType::Object_ClassSubType_ST_MOTORCYCLIST:
      pnc_obstacle_type = OT_MOTORCYCLIST;
      break;
    case ObstacleType::Object_ClassSubType_ST_CYCLIST:
      pnc_obstacle_type = OT_CYCLIST;
      break;
    case ObstacleType::Object_ClassSubType_ST_PEDESTRIAN:
      pnc_obstacle_type = OT_PEDESTRIAN;
      break;
    case ObstacleType::Object_ClassSubType_ST_TRICYCLIST:
      pnc_obstacle_type = OT_TRICYCLIST;
      break;
    case ObstacleType::Object_ClassSubType_ST_TRAFFICCONE:
      pnc_obstacle_type = OT_CONE;
      break;
    case ObstacleType::Object_ClassSubType_ST_ISOLATION_BARREL:
    case ObstacleType::Object_ClassSubType_ST_BARRIER:
      pnc_obstacle_type = OT_BARRIER;
      break;
    case ObstacleType::Object_ClassSubType_ST_CONSTRUCITONSIGN:
    case ObstacleType::Object_ClassSubType_ST_WARNING_TRIANGLE:
      pnc_obstacle_type = OT_WARNING_TRIANGLE;
      break;
    case ObstacleType::Object_ClassSubType_ST_UNKNOWN_MOVABLE:
      pnc_obstacle_type = OT_UNKNOWN_MOVABLE;
      break;

    case ObstacleType::Object_ClassSubType_ST_POLE:
    case ObstacleType::Object_ClassSubType_ST_UNKNOWN_UNMOVABLE:
      pnc_obstacle_type = OT_UNKNOWN_STATIC;
      break;
  }
  return pnc_obstacle_type;
}

PredictionResult AdaptPredictionResult(const zark::PredictionObjs& pred_msg) {
  constexpr double kMinMovingSpeed = 0.45;
  PredictionResult prediction_result;
  auto& objects_proto = prediction_result.objects_proto;
  auto& objects_prediction_proto = prediction_result.objects_prediction_proto;
  const auto& prediction = pred_msg.proto;
  objects_prediction_proto.mutable_header()->set_timestamp(
      prediction.header().timestamp_ns() * 1e-3);
  objects_prediction_proto.mutable_header()->set_channel_seq_number(
      prediction.header().sequence_num());
  objects_proto.mutable_header()->set_timestamp(
      prediction.header().timestamp_ns() * 1e-3);
  objects_proto.mutable_header()->set_channel_seq_number(
      prediction.header().sequence_num());
  objects_proto.set_scope(ObjectsProto::SCOPE_REAL);
  for (const auto& obstacle : prediction.prediction_object()) {
    if ("-1" == std::to_string(obstacle.obj_id())) continue;

    const auto obj_iter = std::find_if(
        objects_proto.objects().begin(), objects_proto.objects().end(),
        [&obstacle](const ObjectProto& obj) -> bool {
          return obj.id() == std::to_string(obstacle.obj_id());
        });
    bool is_new_obstacle = false;
    const auto& fusion_obj = obstacle.fusion_object();
    if (obj_iter == objects_proto.objects().end()) {
      is_new_obstacle = true;

      auto object = objects_proto.add_objects();
      object->set_id(std::to_string(obstacle.obj_id()));
      const auto& fusion_obj = obstacle.fusion_object();
      object->set_type(AdaptObjectType(
          static_cast<ObstacleType>(fusion_obj.class_sub_type())));

      object->set_timestamp(
          static_cast<double>(prediction.header().timestamp_ns()) * 1e-9);
      object->mutable_pos()->set_x(fusion_obj.odom_center().x());
      object->mutable_pos()->set_y(fusion_obj.odom_center().y());
      object->mutable_vel()->set_x(fusion_obj.odom_abs_vel().x());
      object->mutable_vel()->set_y(fusion_obj.odom_abs_vel().y());
      object->mutable_accel()->set_x(fusion_obj.odom_abs_acc().x());
      object->mutable_accel()->set_y(fusion_obj.odom_abs_acc().y());
      object->set_yaw(fusion_obj.odom_orientation_angle());
      object->set_yaw_rate(fusion_obj.odom_orientation_angle_rate());
#if 0
          
          object->mutable_obstacle_light()->set_left_turn_lights(
              static_cast<e2e_noa::ObstacleLightType>(
                  obstacle.obstacle_light().left_turn_lights()));
          object->mutable_obstacle_light()->set_right_turn_lights(
              static_cast<e2e_noa::ObstacleLightType>(
                  obstacle.obstacle_light().right_turn_lights()));
          object->mutable_obstacle_light()->set_brake_lights(
              static_cast<e2e_noa::ObstacleLightType>(
                  obstacle.obstacle_light().brake_lights()));
          object->mutable_obstacle_light()->set_hazard_lights(
              static_cast<e2e_noa::ObstacleLightType>(
                  obstacle.obstacle_light().hazard_lights()));
          object->mutable_obstacle_light()->set_left_turn_lights_conf(
              obstacle.obstacle_light().left_turn_lights_conf());
          object->mutable_obstacle_light()->set_right_turn_lights_conf(
              obstacle.obstacle_light().right_turn_lights_conf());
          object->mutable_obstacle_light()->set_brake_lights_conf(
              obstacle.obstacle_light().brake_lights_conf());
          object->mutable_obstacle_light()->set_hazard_lights_conf(
              obstacle.obstacle_light().hazard_lights_conf());
          LOG(ERROR) << "object: " << object->id()
                    << " obstacle light : " <<
                    (int)obstacle.obstacle_light().left_turn_lights()
                    << " " << (int)obstacle.obstacle_light().right_turn_lights()
                    << " " <<  (int)obstacle.obstacle_light().brake_lights()
                    << " " << (int)obstacle.obstacle_light().hazard_lights();
#endif

      object->mutable_bounding_box()->set_x(fusion_obj.odom_center().x());
      object->mutable_bounding_box()->set_y(fusion_obj.odom_center().y());
      object->mutable_bounding_box()->set_heading(
          fusion_obj.odom_orientation_angle());
      double width = fusion_obj.size().y() < 0.01 ? 0.2 : fusion_obj.size().y();
      double length = fusion_obj.size().x();
      if (length < 0.01) {
        length = static_cast<ObstacleType>(fusion_obj.class_sub_type()) ==
                         ObstacleType::Object_ClassSubType_ST_PEDESTRIAN
                     ? 0.2
                     : 1.0;
      }
      object->mutable_bounding_box()->set_length(length);
      object->mutable_bounding_box()->set_width(width);
      object->set_parked(false);
      object->set_observation_state(OS_PARTIALLY_OBSERVED);
      object->set_ground_z(fusion_obj.size().z());
      const Vec2d obstacle_center(fusion_obj.odom_center().x(),
                                  fusion_obj.odom_center().y());
      Box2d obstacle_box(obstacle_center, fusion_obj.odom_orientation_angle(),
                         length, width);

      const std::vector<Box2d::Corner> points_index{
          Box2d::FRONT_LEFT, Box2d::REAR_LEFT, Box2d::REAR_RIGHT,
          Box2d::FRONT_RIGHT};
      std::vector<Vec2d> obstacle_corner_points;
      obstacle_corner_points.clear();
      obstacle_corner_points.emplace_back(
          fusion_obj.odom_corner_left_front().x(),
          fusion_obj.odom_corner_left_front().y());
      obstacle_corner_points.emplace_back(
          fusion_obj.odom_corner_left_rear().x(),
          fusion_obj.odom_corner_left_rear().y());
      obstacle_corner_points.emplace_back(
          fusion_obj.odom_corner_right_rear().x(),
          fusion_obj.odom_corner_right_rear().y());
      obstacle_corner_points.emplace_back(
          fusion_obj.odom_corner_right_front().x(),
          fusion_obj.odom_corner_right_front().y());
      bool use_corner_pt = false;
      bool is_convex = false;
      bool is_static = fusion_obj.odom_abs_vel().x() < 1e-6 &&
                       fusion_obj.odom_abs_vel().y() < 1e-6;
      if ((static_cast<ObstacleType>(fusion_obj.class_sub_type()) ==
               ObstacleType::Object_ClassSubType_ST_UNKNOWN_UNMOVABLE ||
           is_static) &&
          obstacle_corner_points.size() > 2u) {
        std::vector<Vec2d> points;
        for (const auto& corner_pt : obstacle_corner_points) {
          Vec2d point(corner_pt.x(), corner_pt.y());
          points.emplace_back(std::move(point));
        }
        Polygon2d corner_polygon(points, 1);
        is_convex = corner_polygon.is_convex();
        if (is_convex) {
          for (const auto& point : points) {
            auto contour = object->add_contour();
            contour->set_x(point.x());
            contour->set_y(point.y());
          }
          use_corner_pt = true;
        }
      }
      if (!use_corner_pt || !is_convex) {
        for (const Box2d::Corner point_index : points_index) {
          auto contour = object->add_contour();
          const Vec2d corner_point = obstacle_box.GetCorner(point_index);
          contour->set_x(corner_point.x());
          contour->set_y(corner_point.y());
        }
      }
    }

    auto object_prediction_proto = objects_prediction_proto.add_objects();
    object_prediction_proto->set_id(std::to_string(obstacle.obj_id()));
    ObjectProto perception_object =
        is_new_obstacle
            ? objects_proto.objects(objects_proto.objects().size() - 1)
            : *obj_iter;
    object_prediction_proto->mutable_perception_object()->CopyFrom(
        std::move(perception_object));

    const double obstacle_speed = std::sqrt(
        fusion_obj.odom_abs_vel().x() * fusion_obj.odom_abs_vel().x() +
        fusion_obj.odom_abs_vel().y() * fusion_obj.odom_abs_vel().y());

    constexpr double kMinBicycleSpeed = 1e-6;
    constexpr double kMinPedestrainMovingSpeed = 1e-6;

    double min_stationary_speed = kMinMovingSpeed;

    const auto& obstacle_type =
        static_cast<ObstacleType>(fusion_obj.class_sub_type());
    if (obstacle_type == ObstacleType::Object_ClassSubType_ST_CYCLIST) {
      min_stationary_speed = kMinBicycleSpeed;
    } else if (obstacle_type ==
               ObstacleType::Object_ClassSubType_ST_PEDESTRIAN) {
      min_stationary_speed = kMinPedestrainMovingSpeed;
    }

    bool is_stationary = obstacle_speed < min_stationary_speed ? true : false;

    for (int i = 0; i < obstacle.trajectory().size(); ++i) {
      const auto& trajectory = obstacle.trajectory().at(i);
      auto obstacle_trajectory = object_prediction_proto->add_trajectories();
      obstacle_trajectory->set_intention(
          AdaptTrajectoryIntention(trajectory.intent_type()));
      obstacle_trajectory->set_probability(trajectory.probability());

      obstacle_trajectory->set_probability(1.0);
      if (is_stationary) {
        obstacle_trajectory->set_type(PT_STATIONARY);
      } else {
        obstacle_trajectory->set_type(PT_VEHICLE_LANE_FOLLOW);
      }
      obstacle_trajectory->set_index(i);
      obstacle_trajectory->set_is_reversed(false);
      double s = 0.0;
      for (int i = 0; i < trajectory.trajectory_point().size(); ++i) {
        const auto& point = trajectory.trajectory_point()[i].path_point();
        if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
          LOG(ERROR) << "Infinite value in prediction trajectory, obstacle id: "
                     << obstacle.obj_id() << "\t point.x = " << point.x()
                     << "\t point.y = " << point.y();
          continue;
        }
        auto trajectory_point = obstacle_trajectory->add_points();
        trajectory_point->mutable_pos()->set_x(point.x());
        trajectory_point->mutable_pos()->set_y(point.y());
        if (i > 0) {
          const double dx =
              point.x() - trajectory.trajectory_point()[i - 1].path_point().x();
          const double dy =
              point.y() - trajectory.trajectory_point()[i - 1].path_point().y();
          const double dist = dx * dx + dy * dy;
          s += std::sqrt(std::max(dist, 0.0));
        }
        trajectory_point->set_s(s);
        trajectory_point->set_theta(point.theta());
        trajectory_point->set_kappa(0.0);

        trajectory_point->set_t(
            trajectory.trajectory_point()[i].relative_time() -
            trajectory.trajectory_point()[0].relative_time());
        trajectory_point->set_v(trajectory.trajectory_point()[i].v());
        trajectory_point->set_a(trajectory.trajectory_point()[i].a());
      }
    }

    if (obstacle.trajectory().empty()) {
      auto obstacle_trajectory = object_prediction_proto->add_trajectories();
      obstacle_trajectory->set_probability(1.0);
      if (is_stationary) {
        obstacle_trajectory->set_type(PT_STATIONARY);
      } else {
        obstacle_trajectory->set_type(PT_VEHICLE_LANE_FOLLOW);
      }
      obstacle_trajectory->set_intention(
          TrajectoryIntention::INTENTION_UNKNOWN);
      obstacle_trajectory->set_index(0);
      obstacle_trajectory->set_is_reversed(false);
      auto trajectory_point = obstacle_trajectory->add_points();
      trajectory_point->mutable_pos()->set_x(fusion_obj.odom_center().x());
      trajectory_point->mutable_pos()->set_y(fusion_obj.odom_center().y());
      trajectory_point->set_s(0);
      trajectory_point->set_theta(fusion_obj.odom_orientation_angle());
      trajectory_point->set_kappa(0.0);
      trajectory_point->set_t(0.0);
      trajectory_point->set_v(obstacle_speed);
      trajectory_point->set_a(0.0);
    }
  }
  return prediction_result;
}

void ComputeObjectsStopTime(
    PredictionResult& prediction_result,
    absl::flat_hash_map<std::string, PlannerState::ObjectStopTimeResult>&
        object_stop_time_map) {
  constexpr double kMinMovingSpeedThes = 0.6;
  for (const auto& object :
       prediction_result.objects_prediction_proto.objects()) {
    const auto& obj_id = object.id();
    double speed = std::sqrt(object.perception_object().vel().x() *
                                 object.perception_object().vel().x() +
                             object.perception_object().vel().y() *
                                 object.perception_object().vel().y());
    std::string speed_debug =
        "id: " + obj_id + " speed: " + std::to_string(speed);

    if (object_stop_time_map.find(object.id()) == object_stop_time_map.end()) {
      PlannerState::ObjectStopTimeResult stop_time;
      stop_time.last_time = object.perception_object().timestamp();
      if (speed < kMinMovingSpeedThes) {
        stop_time.time_duration_since_stop = 0.1;
        stop_time.previous_stop_time_duration = 0.1;
        stop_time.last_move_time_duration = 0.0;
      } else {
        stop_time.time_duration_since_stop = 0.0;
        stop_time.previous_stop_time_duration = 0.0;
        stop_time.last_move_time_duration = 0.1;
      }
      object_stop_time_map[object.id()] = stop_time;
    } else {
      if (speed < kMinMovingSpeedThes) {
        double last_stop_duration_time =
            object_stop_time_map[object.id()].time_duration_since_stop;
        double previous_stop_time_duration =
            object_stop_time_map[object.id()].previous_stop_time_duration;
        object_stop_time_map[object.id()].time_duration_since_stop =
            last_stop_duration_time +
            std::fmax(0.0, object.perception_object().timestamp() -
                               object_stop_time_map[object.id()].last_time);
        object_stop_time_map[object.id()].previous_stop_time_duration =
            previous_stop_time_duration +
            std::fmax(0.0, object.perception_object().timestamp() -
                               object_stop_time_map[object.id()].last_time);
      } else {
        double last_move_time_duration =
            object_stop_time_map[object.id()].last_move_time_duration;
        object_stop_time_map[object.id()].time_duration_since_stop = 0.0;
        object_stop_time_map[object.id()].last_move_time_duration =
            last_move_time_duration +
            std::fmax(0.0, object.perception_object().timestamp() -
                               object_stop_time_map[object.id()].last_time);
      }
      object_stop_time_map[object.id()].last_time =
          object.perception_object().timestamp();
    }
  }

  auto& prediction_proto = prediction_result.objects_prediction_proto;
  for (auto itr = object_stop_time_map.begin();
       itr != object_stop_time_map.end();) {
    auto obj_iter = prediction_proto.mutable_objects()->begin();
    for (; obj_iter != prediction_proto.mutable_objects()->end(); obj_iter++) {
      if (obj_iter->id() == itr->first) {
        break;
      }
    }
    if (obj_iter == prediction_proto.mutable_objects()->end()) {
      object_stop_time_map.erase(itr++);
    } else {
      obj_iter->mutable_stop_time()->set_time_duration_since_stop(
          itr->second.time_duration_since_stop);
      obj_iter->mutable_stop_time()->set_previous_stop_time_duration(
          itr->second.previous_stop_time_duration);
      obj_iter->mutable_stop_time()->set_last_move_time_duration(
          itr->second.last_move_time_duration);
      ++itr;
      std::string value =
          "obs-id: " + obj_iter->id() + " time_stop: " +
          std::to_string(obj_iter->stop_time().time_duration_since_stop())
              .substr(0, 4) +
          " previous_stop: " +
          std::to_string(obj_iter->stop_time().previous_stop_time_duration())
              .substr(0, 4) +
          " last_move: " +
          std::to_string(obj_iter->stop_time().last_move_time_duration())
              .substr(0, 4);
    }
  }
}

PNPInfos AdaptPNPInfo(const zark::PredictionObjs& prediction) {
  PNPInfos pnp_infos;

  return pnp_infos;
}

bool CheckAlcResetCondition(ResetReasonProto::Reason reset_reason) {
  switch (reset_reason) {
    case ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND:
    case ResetReasonProto::PREV_NOW_POINT_NOT_FOUND:
    case ResetReasonProto::REROUTED:
    case ResetReasonProto::NON_AUTONOMY:
    case ResetReasonProto::FIRST_ENGAGE:
    case ResetReasonProto::FULL_STOP:
    case ResetReasonProto::RECOVER_FROM_AEB:
    case ResetReasonProto::NEW_FREESPACE_PATH:
    case ResetReasonProto::STEER_ONLY_ENGAGE:
    case ResetReasonProto::STEER_ONLY:
    case ResetReasonProto::SPEED_ONLY:
    case ResetReasonProto::SPEED_ONLY_ENGAGE:
      return true;
    case ResetReasonProto::NONE:
    case ResetReasonProto::LON_ERROR_TOO_LARGE:
    case ResetReasonProto::LAT_ERROR_TOO_LARGE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

SceneReasoningOutput RunSceneReasoningAndFillDebug(
    const PlannerSemanticMapManager& psmm,
    const ad_e2e::planning::TrafficLightStatusMap& tl_info_map,
    const PlannerObjectController& obj_mgr,
    const ObjectsPredictionProto& prediction,
    const ApolloTrajectoryPointProto& plan_start_point,
    WorkerThreadManager* thread_pool,
    ObjectHistoryController& obj_his_manager) {
  const auto lane_paths_or = BuildLanePathsFormPsmm(psmm, plan_start_point);
  if (!lane_paths_or.ok()) {
    LOG(WARNING)
        << "RunSceneReasoningAndFillDebug failed in BuildLanePathsFormPsmm: "
        << lane_paths_or.status();
    return SceneReasoningOutput();
  }
  auto scene_reasoning_output_or = RunSceneReasoning(
      SceneReasoningInput{.psmm = &psmm,
                          .prediction = &prediction,
                          .tl_info_map = &tl_info_map,
                          .lane_paths = &(*lane_paths_or),
                          .plan_start_point = &plan_start_point},
      thread_pool, obj_his_manager);

  if (!scene_reasoning_output_or.ok()) {
    LOG(WARNING)
        << "RunSceneReasoningAndFillDebug failed in RunSceneReasoning: "
        << scene_reasoning_output_or.status();
    return SceneReasoningOutput();
  }

  return *scene_reasoning_output_or;
}

absl::StatusOr<std::vector<mapping::LanePath>> BuildLanePathsFormPsmm(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point) {
  const Vec2d ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  const auto& map = psmm.map_ptr();
  if (!map || !map->route()) {
    return absl::NotFoundError("map_ptr or route is nullptr.");
  }

  std::vector<std::string> navi_start_lanes;
  auto& navi_start = map->route()->navi_start();
  if (!navi_start.section_id.empty()) {
    const auto& navi_section = map->GetSectionById(navi_start.section_id);
    if (navi_section) {
      for (auto& id : navi_section->lanes()) {
        navi_start_lanes.push_back(id);
      }
    }
  }

  std::vector<mapping::LanePath> all_lane_paths;
  for (int i = 0; i < navi_start_lanes.size(); i++) {
    const auto& lane = map->GetLaneById(navi_start_lanes[i]);
    if (lane && lane->is_navigation() && lane->IsValid()) {
      std::vector<std::vector<LaneConstPtr>> lane_seqs;
      map->GetAllLaneSequences(lane, lane_seqs);
      double max_lane_seq_length = std::numeric_limits<double>::min();
      std::vector<LaneConstPtr> max_lane_seq = *lane_seqs.begin();
      std::vector<std::string> lane_ids;
      for (const auto& lane_seq : lane_seqs) {
        double length = 0.0;
        for (const auto& lane_ptr : lane_seq) {
          if (lane_ptr->IsValid()) length += lane_ptr->topo_length();
        }
        if (length > max_lane_seq_length) {
          max_lane_seq_length = length;
          max_lane_seq = lane_seq;
        }
      }
      for (const auto& lane_ptr : max_lane_seq) {
        if (lane_ptr->IsValid()) lane_ids.emplace_back(lane_ptr->id());
      }

      e2e_noa::mapping::LanePath lane_path(map, lane_ids, 0.0, 1.0);
      all_lane_paths.emplace_back(std::move(lane_path));
    }
  }
  if (all_lane_paths.empty()) {
    return absl::NotFoundError(" all_lane_seqs is nullptr.");
  }
  return all_lane_paths;
}

double GetLaneLength(const std::vector<Vec2d>& points) {
  if (points.empty()) return 0.0;
  double s = 0.0;
  std::vector<double> accumulated_s;
  accumulated_s.emplace_back(s);
  for (int i = 0; i < points.size(); ++i) {
    int last_index = std::max(0, i - 1);
    auto ds = std::hypot(points[i].x() - points[last_index].x(),
                         points[i].y() - points[last_index].y());
    s += ds;
    accumulated_s.emplace_back(s);
  }
  return accumulated_s.empty() ? 0.0 : accumulated_s.back();
}
}  // namespace
}  // namespace planning
}  // namespace e2e_noa
