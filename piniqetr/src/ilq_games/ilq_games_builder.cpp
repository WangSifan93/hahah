#include "ilq_games/ilq_games_builder.h"
#include "ilq_games/include/examples/muti_vehicle_game.h"
#include "maps/lane_point.h"
#include "math/frenet_common.h"
#include "util/path_util.h"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <regex>
#include <sys/stat.h>
#include <sys/types.h>

namespace e2e_noa::planning {
namespace {
constexpr double kOnComingAngleThreshold = M_PI * 7 / 8;
constexpr double kOnComingTimeThreshold = 2.0;
constexpr double kOnComingLatThreshold = 3.6;
constexpr double kOnComingLatBuffer = 0.2;
constexpr double kEpsilon = 1e-6;

inline LaneSemantic QueryLaneSemantic(const ad_e2e::planning::Lane &lane_info) {
  if (!lane_info.junction_id().empty()) {
    switch (lane_info.turn_type()) {
    case ad_e2e::planning::TurnType::NO_TURN:
      return LaneSemantic::INTERSECTION_STRAIGHT;
    case ad_e2e::planning::TurnType::LEFT_TURN:
      return LaneSemantic::INTERSECTION_LEFT_TURN;
    case ad_e2e::planning::TurnType::RIGHT_TURN:
      return LaneSemantic::INTERSECTION_RIGHT_TURN;
    case ad_e2e::planning::TurnType::U_TURN:
      return LaneSemantic::INTERSECTION_UTURN;
    default:
      throw std::runtime_error("switch case on enum unexpected");
    }
  } else {
    switch (lane_info.turn_type()) {
    case ad_e2e::planning::TurnType::NO_TURN:
      return LaneSemantic::ROAD;
    case ad_e2e::planning::TurnType::LEFT_TURN:
    case ad_e2e::planning::TurnType::RIGHT_TURN:
    case ad_e2e::planning::TurnType::U_TURN:
      return LaneSemantic::ROAD;
    default:
      throw std::runtime_error("switch case on enum unexpected");
    }
  }
}

std::optional<LaneSemantic>
GetLaneSematic(const PlannerSemanticMapManager *psmm,
               const ApolloTrajectoryPointProto *plan_start_point) {
  const Vec2d path_point_xy = ToVec2d(plan_start_point->path_point());
  std::optional<LaneSemantic> lane_semantic = std::nullopt;

  const auto current_lane = psmm->map_ptr()->GetNearestLane(
      {plan_start_point->path_point().x(), plan_start_point->path_point().y()},
      10.0);
  if (current_lane != nullptr) {
    lane_semantic = QueryLaneSemantic(*current_lane);
  }
  return lane_semantic;
}

bool CheckILQGamesEnableConditions(const ILQGAMESBuilderInput &input) {
  // do not in lane change stage
  if (input.lc_stage != LaneChangeStage::LCS_NONE) {
    return false;
  }
  // on road
  std::optional<LaneSemantic> lane_semantic =
      GetLaneSematic(input.psmm, input.plan_start_point);
  if (!lane_semantic.has_value()) {
    return false;
  }
  if (*lane_semantic != LaneSemantic::ROAD) {
    return false;
  }
  return true;
}

bool CheckThetaDiff(const double obstacle_theta, const double ego_theta) {
  const double theta_diff =
      std::fabs(NormalizeAngle(ego_theta - obstacle_theta));
  if (std::fabs(theta_diff) < kOnComingAngleThreshold) {
    return false;
  }
  return true;
}

bool CheckIsBackObject(const ApolloTrajectoryPointProto *plan_start_point,
                       const PlannerObject &obj) {
  const Vec2d ego_dir_unit =
      Vec2d::FastUnitFromAngle(plan_start_point->path_point().theta());
  bool is_back_obj = true;
  for (const auto &corner_point : obj.contour().points()) {
    if (ego_dir_unit.Dot(corner_point -
                         ToVec2d(plan_start_point->path_point())) > 0.0) {
      is_back_obj = false;
      break;
    }
  }
  return is_back_obj;
}

bool CheckInitalLatDist(const PlanPassage *passage,
                        const ApolloTrajectoryPointProto *plan_start_point,
                        const VehicleGeometryParamsProto *veh_geom,
                        const SpacetimeObjectTrajectory &obstacle,
                        const StationWaypoint &ego_s_offset,
                        const double ego_l_offset, double &ttc) {
  const double ego_front_to_center = veh_geom->front_edge_to_center();
  FrenetPolygon obj_frenet_polygon;
  auto frenet_box = passage->QueryFrenetBoxAtContour(obstacle.contour(), false);
  if (!frenet_box.ok()) {
    return false;
  }
  obj_frenet_polygon.s_max = frenet_box.value().s_max - ego_s_offset.accum_s;
  obj_frenet_polygon.s_min = frenet_box.value().s_min - ego_s_offset.accum_s;
  obj_frenet_polygon.l_max = frenet_box.value().l_max - ego_l_offset;
  obj_frenet_polygon.l_min = frenet_box.value().l_min - ego_l_offset;
  const double dl = std::min(std::fabs(obj_frenet_polygon.l_min),
                             std::fabs(obj_frenet_polygon.l_max));
  if (dl > kOnComingLatThreshold) {
    return false;
  }
  if (obj_frenet_polygon.s_min < ego_front_to_center) {
    return false;
  }
  const double ds = obj_frenet_polygon.s_min - ego_front_to_center;
  const auto ob_v_vec = obstacle.planner_object().velocity();
  const double ego_v = plan_start_point->v();
  ttc = ds / std::max(kEpsilon, ego_v + ob_v_vec.x());
  return true;
}

bool CheckLatDist(const PlanPassage *passage,
                  const VehicleGeometryParamsProto *veh_geom,
                  const SpacetimeObjectTrajectory &obstacle) {
  const auto &obj_states = obstacle.states();
  if (obj_states.empty()) {
    return false;
  }
  auto obj_states_iter = obj_states.rbegin();
  while (obj_states_iter != obj_states.rend()) {
    auto frenet_box =
        passage->QueryFrenetBoxAtContour(obj_states_iter->contour, false);

    const double min_l = std::min(std::fabs(frenet_box.value().l_min),
                                  std::fabs(frenet_box.value().l_max));
    const double ego_half_width = veh_geom->width() * 0.5;
    if (min_l < kOnComingLatBuffer + ego_half_width) {
      return true;
    }
    ++obj_states_iter;
  }
  return false;
}

bool IsValidOnComingObject(const PlanPassage *passage,
                           const ApolloTrajectoryPointProto *plan_start_point,
                           const VehicleGeometryParamsProto *veh_geom,
                           const ILQGamesParamsProto *ilq_games_params,
                           const SpacetimeObjectTrajectory &obstacle,
                           double &ttc) {
  const auto ego_s_offset =
      passage->QueryFrenetLonOffsetAt(ToVec2d(plan_start_point->path_point()));
  const auto ego_l_offset =
      passage->QueryFrenetLatOffsetAt(ToVec2d(plan_start_point->path_point()));
  if ((!ego_s_offset.ok()) || (!ego_l_offset.ok())) {
    return false;
  }
  if (obstacle.is_stationary()) {
    return false;
  }
  const auto &obj = obstacle.planner_object();
  if (!CheckThetaDiff(obj.pose().theta(),
                      plan_start_point->path_point().theta())) {
    return false;
  }
  if (CheckIsBackObject(plan_start_point, obj)) {
    return false;
  }
  if (!CheckInitalLatDist(passage, plan_start_point, veh_geom, obstacle,
                          ego_s_offset.value(), ego_l_offset.value(), ttc)) {
    return false;
  }
  if (!CheckLatDist(passage, veh_geom, obstacle)) {
    return false;
  }
  return true;
}

VehicleStaticData
UpdateStaticData(const ILQGamesParamsProto::StaticParams &static_params,
                 const double wheel_base, const double width,
                 const double length, const int id) {
  VehicleStaticData static_data;
  static_data.id = id;
  static_data.inter_axle_distance = wheel_base;
  static_data.width = width;
  static_data.length = length;
  std::string type = "car";
  //上下限参数
  static_data.max_v = static_params.max_v();
  static_data.max_a = static_params.max_a();
  static_data.min_a = static_params.min_a();
  static_data.max_phi = static_params.max_phi();
  static_data.min_proximity = static_params.min_proximity();
  // Cost权重
  static_data.state_regularization = static_params.state_regularization();
  static_data.control_regularization = static_params.control_regularization();
  static_data.lane_cost_w = static_params.lane_cost_w();
  static_data.goal_x_cost_w = static_params.goal_x_cost_w();
  static_data.goal_y_cost_w = static_params.goal_y_cost_w();
  static_data.tar_v_cost_w = static_params.tar_v_cost_w();
  static_data.omega_cost_w = static_params.omega_cost_w();
  static_data.jerk_cost_w = static_params.jerk_cost_w();
  static_data.lane_boundary_cost_w = static_params.lane_boundary_cost_w();
  static_data.vlimit_cost_w = static_params.vlimit_cost_w();
  static_data.alimit_cost_w = static_params.alimit_cost_w();
  static_data.phi_limit_cost_w = static_params.phi_limit_cost_w();
  static_data.proximity_cost_w = static_params.proximity_cost_w();
  return static_data;
}

bool UpdateEgoDynamicData(const PlanPassage *passage,
                          const ApolloTrajectoryPointProto *plan_start_point,
                          const VehicleGeometryParamsProto *veh_geom,
                          VehicleDynamicData &ego_dynamic_data) {
  constexpr double kMaxSpeed = 15.0;
  //初始状态
  ego_dynamic_data.init_x = plan_start_point->path_point().x();
  ego_dynamic_data.init_y = plan_start_point->path_point().y();
  ego_dynamic_data.init_heading = plan_start_point->path_point().theta();
  ego_dynamic_data.init_v = plan_start_point->v();
  //目标终点
  const double time_horizon = time::getTimeHorizon();
  const double time_step = time::getTimeStep();
  const auto ego_s_offset =
      passage->QueryFrenetLonOffsetAt(ToVec2d(plan_start_point->path_point()));
  if (!ego_s_offset.ok()) {
    return false;
  }
  const double s_target = std::min(ego_dynamic_data.init_v * time_horizon +
                                       ego_s_offset.value().accum_s,
                                   passage->end_s());
  const auto goal_xy = passage->QueryPointXYAtS(s_target);
  if (!goal_xy.ok()) {
    return false;
  }

  ego_dynamic_data.goal_x = goal_xy.value().x();
  ego_dynamic_data.goal_y = goal_xy.value().y();
  //期望速度
  ego_dynamic_data.tar_v = plan_start_point->v();
  //中心线参数
  ego_dynamic_data.lane.clear();
  for (double t = 0.0; t < time_horizon; t += time_step) {
    const double s =
        std::min(ego_dynamic_data.init_v * t + ego_s_offset.value().accum_s,
                 passage->end_s());
    const auto xy = passage->QueryPointXYAtS(s);
    if (!xy.ok()) {
      return false;
    }
    ego_dynamic_data.lane.push_back(Point2(xy.value().x(), xy.value().y()));
  }
  return true;
}

bool UpdateObjDynamicData(
    const SpacetimeObjectTrajectory *target_oncoming_object,
    VehicleDynamicData &obj_dynamic_data) {
  obj_dynamic_data.init_x =
      target_oncoming_object->planner_object().pose().pos().x();
  obj_dynamic_data.init_y =
      target_oncoming_object->planner_object().pose().pos().y();
  obj_dynamic_data.init_heading =
      target_oncoming_object->planner_object().pose().theta();
  obj_dynamic_data.init_v = target_oncoming_object->planner_object().pose().v();

  const auto &obj_states = target_oncoming_object->states();
  if (obj_states.empty()) {
    return false;
  }
  obj_dynamic_data.goal_x = obj_states.back().traj_point->pos().x();
  obj_dynamic_data.goal_y = obj_states.back().traj_point->pos().y();
  //期望速度
  obj_dynamic_data.tar_v = target_oncoming_object->planner_object().pose().v();
  obj_dynamic_data.lane.clear();
  for (const auto &state : obj_states) {
    obj_dynamic_data.lane.push_back(
        Point2(state.traj_point->pos().x(), state.traj_point->pos().y()));
  }
  return true;
}

bool UpdateILQGamesParams(
    const PlanPassage *passage,
    const ApolloTrajectoryPointProto *plan_start_point,
    const VehicleGeometryParamsProto *veh_geom,
    const ILQGamesParamsProto *ilq_games_params,
    const SpacetimeObjectTrajectory *target_oncoming_object,
    std::vector<VehicleStaticData> &vehicles_static_dataset,
    std::vector<VehicleDynamicData> &vehicles_dynamic_dataset) {
  vehicles_static_dataset.emplace_back(UpdateStaticData(
      ilq_games_params->ego_static_params(), veh_geom->wheel_base(),
      veh_geom->width(), veh_geom->length(), 0));
  switch (target_oncoming_object->object_type()) {
  case ObjectType::OT_PEDESTRIAN:
    vehicles_static_dataset.emplace_back(
        UpdateStaticData(ilq_games_params->ped_static_params(),
                         target_oncoming_object->bounding_box().length(),
                         target_oncoming_object->bounding_box().width(),
                         target_oncoming_object->bounding_box().length(), 1));
    break;
  case ObjectType::OT_MOTORCYCLIST:
  case ObjectType::OT_CYCLIST:
    vehicles_static_dataset.emplace_back(
        UpdateStaticData(ilq_games_params->cyc_static_params(),
                         target_oncoming_object->bounding_box().length(),
                         target_oncoming_object->bounding_box().width(),
                         target_oncoming_object->bounding_box().length(), 1));
    break;
  default:
    vehicles_static_dataset.emplace_back(
        UpdateStaticData(ilq_games_params->veh_static_params(),
                         target_oncoming_object->bounding_box().length(),
                         target_oncoming_object->bounding_box().width(),
                         target_oncoming_object->bounding_box().length(), 1));
    break;
  }

  VehicleDynamicData ego_dynamic_data;
  VehicleDynamicData obj_dynamic_data;
  if (!UpdateEgoDynamicData(passage, plan_start_point, veh_geom,
                            ego_dynamic_data)) {
    return false;
  }
  if (!UpdateObjDynamicData(target_oncoming_object, obj_dynamic_data)) {
    return false;
  }
  vehicles_dynamic_dataset.emplace_back(ego_dynamic_data);
  vehicles_dynamic_dataset.emplace_back(obj_dynamic_data);
  return true;
}

void UpdateSolverParams(const ILQGamesParamsProto *ilq_games_params,
                        SolverParams &solver_params) {
  solver_params.convergence_tolerance =
      ilq_games_params->convergence_tolerance();
  solver_params.max_solver_iters = ilq_games_params->convergence_tolerance();
  solver_params.linesearch = ilq_games_params->linesearch();
  solver_params.initial_alpha_scaling =
      ilq_games_params->initial_alpha_scaling();
  solver_params.geometric_alpha_scaling =
      ilq_games_params->geometric_alpha_scaling();
  solver_params.max_backtracking_steps =
      ilq_games_params->max_backtracking_steps();
  solver_params.expected_decrease_fraction =
      ilq_games_params->expected_decrease_fraction();
  solver_params.state_regularization = ilq_games_params->state_regularization();
  solver_params.control_regularization =
      ilq_games_params->control_regularization();
  solver_params.unconstrained_solver_max_iters =
      ilq_games_params->unconstrained_solver_max_iters();
      solver_params.enable_online_log = ilq_games_params->enable_online_log();
      solver_params.enable_offline_log = ilq_games_params->enable_offline_log();
}

SpacetimeObjectTrajectory
ModifyObjectTrajectory(const OperatingPoint &result,
                       SpacetimeObjectTrajectory *target_oncoming_object) {
  std::vector<prediction::PredictedTrajectoryPoint> modify_traj;
  PathPoint prev_point;
  prev_point.set_s(0.0);
  double dist = 0.0;
  double t = 0.0;
  if (result.xs.size() == 0) {
    return *target_oncoming_object;
  }
  const double time_step = time::getTimeStep();
  for (size_t i = 0; i < result.xs.size(); i++) {
    const auto &state = result.xs[i];
    PathPoint curr_point;
    prediction::PredictedTrajectoryPoint curr_traj_point;
    curr_point.set_x(state(0));
    curr_point.set_y(state(1));
    curr_point.set_theta(state(2));
    dist = i > 0 ? hypot(curr_point.x() - prev_point.x(),
                         curr_point.y() - prev_point.y())
                 : 0;
    double kappa = 0.0;
    if (i > 0 && i < result.xs.size()) {
      const Vec2d curr_pos(curr_point.x(), curr_point.y());
      const Vec2d pre_pos(prev_point.x(), prev_point.y());
      kappa = NormalizeAngle(curr_point.theta() - prev_point.theta()) /
              (curr_pos - pre_pos).norm();
    }
    curr_point.set_s(prev_point.s() + dist);
    curr_traj_point.set_s(curr_point.s());
    curr_traj_point.set_pos(Vec2d(state(0), state(1)));
    curr_traj_point.set_theta(state(2));
    curr_traj_point.set_kappa(kappa);
    curr_traj_point.set_t(i * time_step);
    curr_traj_point.set_v(state(4));
    curr_traj_point.set_a(state(5));
    prev_point = std::move(curr_point);

    // std::cout << "s: " << curr_traj_point.s() << " v: " <<
    // curr_traj_point.v()
    //           << " kappa: " << curr_traj_point.kappa()
    //           << " t: " << curr_traj_point.t() << " a: " <<
    //           curr_traj_point.a()
    //           << " x: " << curr_traj_point.pos().x()
    //           << " y: " << curr_traj_point.pos().y()
    //           << " heading: " << curr_traj_point.theta() << std::endl;

    modify_traj.push_back(curr_traj_point);
  }
  auto new_pred_traj = target_oncoming_object->trajectory();
  *new_pred_traj.mutable_points() = std::move(modify_traj);
  return target_oncoming_object->CreateTrajectoryMutatedInstance(
      std::move(new_pred_traj));
}

} // namespace

SpacetimeTrajectoryManager BuildILQGames(const ILQGAMESBuilderInput &input) {
  SpacetimeTrajectoryManager st_traj_manager_processed(*(input.st_traj_mgr));
  if (!CheckILQGamesEnableConditions(input)) {
    return st_traj_manager_processed;
  }
  double min_ttc = std::numeric_limits<double>::max();
  SpacetimeObjectTrajectory *target_oncoming_object = nullptr;
  for (auto &obstacle : *st_traj_manager_processed.mutable_trajectories()) {
    double ttc;
    if (!IsValidOnComingObject(input.passage, input.plan_start_point,
                               input.veh_geom, input.ilq_games_params, obstacle,
                               ttc)) {
      continue;
    }
    if (ttc < min_ttc) {
      min_ttc = ttc;
      target_oncoming_object = &obstacle;
    }
  }
  if (target_oncoming_object == nullptr) {
    return st_traj_manager_processed;
  }

  std::vector<VehicleStaticData> vehicles_static_dataset;
  std::vector<VehicleDynamicData> vehicles_dynamic_dataset;
  SolverParams solver_params;

  TimeSet time_set;
  time_set.time_horizon = 8;
  time_set.time_step = 0.2;
  time::setTimeParameter(time_set.time_step, time_set.time_horizon);

  if (!UpdateILQGamesParams(input.passage, input.plan_start_point,
                            input.veh_geom, input.ilq_games_params,
                            target_oncoming_object, vehicles_static_dataset,
                            vehicles_dynamic_dataset)) {
    return st_traj_manager_processed;
  }
  UpdateSolverParams(input.ilq_games_params, solver_params);

  std::unique_ptr<MutiVehicleGameSolve> game_solver =
      std::make_unique<MutiVehicleGameSolve>(
          solver_params, vehicles_static_dataset, vehicles_dynamic_dataset);
  std::string env_name = "ilq_games";
  game_solver->GameProblemSolve(env_name, input.seq_num);

  const OperatingPoint result = game_solver->GetFinalOperatorPoint();

  *target_oncoming_object =
      ModifyObjectTrajectory(result, target_oncoming_object);
  return st_traj_manager_processed;
}

} // namespace e2e_noa::planning
