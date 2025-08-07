#include "speed/mcts_game_decision.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "common/timer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "interactive_agent_process/interactive_agent_process.h"
#include "math/double.h"
#include "math/fast_math.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"
#include "math/intelligent_driver_model.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "plan/planner_defs.h"
#include "plan/speed_profile.h"
#include "plan/trajectory_util.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/decision/interaction_util.h"
#include "speed/decision/post_st_boundary_modifier.h"
#include "speed/decision/pre_brake_decision.h"
#include "speed/decision/pre_brake_util.h"
#include "speed/decision/st_boundary_modifier_util.h"
#include "speed/empty_road_speed.h"
#include "speed/gridded_svt_graph.h"
#include "speed/mcts_game_process/mcts_game_manager.h"
#include "speed/mcts_game_process/vehicle_status_update_model/vehicle_simulation_model.h"
#include "speed/overlap_info.h"
#include "speed/speed_point.h"
#include "speed/st_boundary.h"
#include "speed/st_graph_data.h"
#include "speed/st_point.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa::planning {
namespace {
constexpr double kVThreshold = 10.0;

void PreProcessStBoundries(
    const VehicleGeometryParamsProto &vehicle_geom,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const PathPoint &plan_start_path_point,
    absl::flat_hash_set<std::string> &non_interactive_ids) {
  for (const auto &boundary_with_decision : st_boundaries_with_decision) {
    const bool is_vru =
        StBoundaryProto::PEDESTRIAN ==
            boundary_with_decision.st_boundary()->object_type() ||
        StBoundaryProto::CYCLIST ==
            boundary_with_decision.st_boundary()->object_type();
    const bool is_vehicle = StBoundaryProto::VEHICLE ==
                            boundary_with_decision.st_boundary()->object_type();
    if (!is_vru && !is_vehicle) {
      non_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }
    if (boundary_with_decision.raw_st_boundary()->is_protective()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    if (boundary_with_decision.st_boundary()->min_t() < 1.0 && is_vehicle) {
      non_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    if (!boundary_with_decision.st_boundary()->overlap_meta().has_value()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "No overlap meta data.";
      continue;
    }
    const auto &overlap_meta =
        *boundary_with_decision.st_boundary()->overlap_meta();

    VLOG(3) << "Object type: "
            << StBoundaryProto::ObjectType_Name(
                   boundary_with_decision.st_boundary()->object_type())
            << ", overlap pattern: "
            << StOverlapMetaProto::OverlapPattern_Name(overlap_meta.pattern())
            << ", overlap source: "
            << StOverlapMetaProto::OverlapSource_Name(overlap_meta.source())
            << ", modification type: "
            << StOverlapMetaProto::ModificationType_Name(
                   overlap_meta.modification_type())
            << ", priority: "
            << StOverlapMetaProto::OverlapPriority_Name(
                   overlap_meta.priority());

    switch (overlap_meta.modification_type()) {
      case StOverlapMetaProto::NON_MODIFIABLE: {
        non_interactive_ids.insert(boundary_with_decision.id());
        VLOG(2) << "Not considered modification type.";
        continue;
      }
      case StOverlapMetaProto::LON_LAT_MODIFIABLE: {
        if (is_vehicle) {
          non_interactive_ids.insert(boundary_with_decision.id());
          VLOG(2) << "Not considered modification type.";
          continue;
        }
      }
      case StOverlapMetaProto::LON_MODIFIABLE:
        break;
    }

    if (boundary_with_decision.decision_type() != StBoundaryProto::UNKNOWN) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "Pre-decision is: "
              << StBoundaryProto::DecisionType_Name(
                     boundary_with_decision.decision_type());
      continue;
    }

    if (!boundary_with_decision.traj_id().has_value()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "Modified by others.";
      continue;
    }

    if (processed_st_objects.find(*boundary_with_decision.traj_id()) !=
        processed_st_objects.end()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    const auto *spacetime_obj =
        st_traj_mgr.FindTrajectoryById(*boundary_with_decision.traj_id());
    if (nullptr == spacetime_obj) {
      non_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    const double obs_v = spacetime_obj->planner_object().pose().v();
    if (is_vru && obs_v >= kVThreshold) {
      non_interactive_ids.insert(boundary_with_decision.id());
      continue;
    }

    if (overlap_meta.source() == StOverlapMetaProto::AV_CUTIN) {
      const auto pos = spacetime_obj->pose().pos();
      const auto heading_perp =
          Vec2d::FastUnitFromAngle(spacetime_obj->pose().theta() - M_PI * 0.5);
      const HalfPlane agent_view(pos - heading_perp, pos + heading_perp);
      const auto av_box = ComputeAvBox(
          Vec2d(plan_start_path_point.x(), plan_start_path_point.y()),
          plan_start_path_point.theta(), vehicle_geom);
      bool is_av_inside_agent_view = false;
      for (const auto &corner : av_box.GetCornersCounterClockwise()) {
        if (agent_view.IsPointInside(corner)) {
          is_av_inside_agent_view = true;
          break;
        }
      }
      if (!is_av_inside_agent_view) {
        non_interactive_ids.insert(boundary_with_decision.id());
        VLOG(2) << "Out of agent view when av cutin.";
        continue;
      }
    }
  }
}

void GetInteractiveSTBoundaries(
    const VehicleGeometryParamsProto &vehicle_geom,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    std::vector<StBoundaryWithDecision> *st_boundaries_with_decision,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const PathPoint &plan_start_path_point,
    std::vector<InteractiveInput> &interactive_input_infos) {
  absl::flat_hash_set<std::string> non_interactive_ids;
  PreProcessStBoundries(vehicle_geom, st_traj_mgr, *st_boundaries_with_decision,
                        processed_st_objects, plan_start_path_point,
                        non_interactive_ids);

  if (non_interactive_ids.size() == st_boundaries_with_decision->size()) {
    return;
  }

  for (auto &st_boundary : *st_boundaries_with_decision) {
    if (non_interactive_ids.find(st_boundary.id()) ==
        non_interactive_ids.end()) {
      auto &input_info = interactive_input_infos.emplace_back();
      input_info.boundary_with_decision = &st_boundary;
      if (!st_boundary.traj_id().has_value()) {
        continue;
      }
      const auto *spacetime_obj =
          st_traj_mgr.FindTrajectoryById(st_boundary.traj_id().value());
      if (nullptr == spacetime_obj) {
        continue;
      }

      const double obs_v = spacetime_obj->planner_object().pose().v();
      const bool is_vru =
          StBoundaryProto::PEDESTRIAN ==
              st_boundary.st_boundary()->object_type() ||
          StBoundaryProto::CYCLIST == st_boundary.st_boundary()->object_type();

      input_info.is_hrvo = is_vru && obs_v < kVThreshold;
    }
  }
}

bool IsValidMCTSResult(const MCTSInteractiveResult &interactive_result,
                       const SpacetimeTrajectoryManager &st_traj_mgr,
                       const DiscretizedPath &path) {
  auto &mcts_result = interactive_result.mcts_result;
  if (mcts_result.size() < 2 || nullptr == mcts_result.back()) {
    DLOG(INFO) << "mcts_result size = " << mcts_result.size();
    return false;
  }
  const std::string traj_id = interactive_result.traj_id;
  const auto *spacetime_obj = st_traj_mgr.FindTrajectoryById(traj_id);
  if (nullptr == spacetime_obj) {
    return false;
  }
  const double current_t = mcts_result.back()->current_time;
  double first_x = 0.0;
  double first_y = 0.0;
  double last_x = 0.0;
  double last_y = 0.0;
  double first_t = 0.0;
  double last_t = 0.0;
  for (const auto &point : spacetime_obj->trajectory().points()) {
    if (point.t() <= current_t) {
      first_x = point.pos().x();
      first_y = point.pos().y();
      first_t = point.t();
      continue;
    }
    if (point.t() >= current_t) {
      last_x = point.pos().x();
      last_y = point.pos().y();
      last_t = point.t();
      break;
    }
  }
  if (ad_e2e::planning::math::Double::Compare(last_t, first_t) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return false;
  }
  double origin_x =
      first_x + (last_x - first_x) * (current_t - first_t) / (last_t - first_t);
  double origin_y =
      first_y + (last_y - first_y) * (current_t - first_t) / (last_t - first_t);
  double mcts_x = mcts_result.back()->state.follower_status.x;
  double mcts_y = mcts_result.back()->state.follower_status.y;
  FrenetCoordinate original_frenet_point =
      path.XYToSL(Vec2d(origin_x, origin_y));
  FrenetCoordinate mcts_frenet_point = path.XYToSL(Vec2d(mcts_x, mcts_y));
  FrenetCoordinate start_frenet_point =
      path.XYToSL(Vec2d(mcts_result.front()->state.follower_status.x,
                        mcts_result.front()->state.follower_status.y));
#if VRU_DEBUG
  DLOG(INFO) << "origin_frenet_point.s = " << original_frenet_point.s
             << ", origin_frenet_point.l = " << original_frenet_point.l
             << ", mcts_frenet_point.s = " << mcts_frenet_point.s
             << ", mcts_frenet_point.l = " << mcts_frenet_point.l
             << ", start_frenet_point.s = " << start_frenet_point.s
             << " start_frenet_point.l = " << start_frenet_point.l
             << " trj id = " << traj_id << ", current_t = " << current_t;
#endif
  if (original_frenet_point.s > mcts_frenet_point.s ||
      fabs(original_frenet_point.l) < fabs(mcts_frenet_point.l) ||
      (original_frenet_point.l * start_frenet_point.l < 0.0 &&
       start_frenet_point.l * mcts_frenet_point.l > 0.0) ||
      (original_frenet_point.s * start_frenet_point.s < 0.0 &&
       start_frenet_point.s * mcts_frenet_point.s > 0.0)) {
    return true;
  }
  return false;
}

bool IsValidMCTSResult(const double remain_dis_min_threshold,
                       const std::vector<MCTSNodeConstPtr> &mcts_result) {
  if (mcts_result.size() < 2 || nullptr == mcts_result.back()) {
    return false;
  }
  const double &leader_remain_dis =
      mcts_result.back()->state.leader_status.remain_dis;
  const double &follower_remain_dis =
      mcts_result.back()->state.follower_status.remain_dis;
  if (leader_remain_dis < 0.0 && follower_remain_dis > 0.0) {
    if (std::fabs(follower_remain_dis - leader_remain_dis) >
        remain_dis_min_threshold) {
      return true;
    }
  }
  return false;
}

bool FilterInvalidMCTSInteractiveResults(
    const SpeedPlanningParamsProto &params,
    std::vector<MCTSInteractiveResult> &interactive_results,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const DiscretizedPath &path) {
  int count = 0;
  for (auto &interactive_result : interactive_results) {
    bool is_valid = false;
    if (interactive_result.is_hrvo) {
      is_valid = IsValidMCTSResult(interactive_result, st_traj_mgr, path);
    } else {
      is_valid =
          IsValidMCTSResult(params.mcts_lon_params().remain_dis_min_threshold(),
                            interactive_result.mcts_result);
    }
    interactive_result.is_valid = is_valid;
    if (is_valid) {
      count++;
    }
  }
  return count > 0;
}

std::vector<AccelPoint> CalculateMCTSAccelPointList(
    const double obj_total_time,
    const std::vector<MCTSNodeConstPtr> &mcts_result) {
  std::vector<AccelPoint> accel_point_list;
  for (int i = 1; i < mcts_result.size(); ++i) {
    if (mcts_result[i - 1]->current_time > obj_total_time) {
      return accel_point_list;
    }
    accel_point_list.emplace_back(mcts_result[i - 1]->current_time,
                                  mcts_result[i]->state.follower_status.a);
  }
  return accel_point_list;
}

inline double GetAgentTimeStep(const SpacetimeObjectTrajectory *spacetime_obj) {
  return spacetime_obj->states().size() > 1
             ? spacetime_obj->states()[1].traj_point->t() -
                   spacetime_obj->states()[0].traj_point->t()
             : prediction::kPredictionTimeStep;
}

std::vector<AccelPoint> CalculateAgentAccelPointList(
    const MCTSInteractiveResult &interactive_result) {
  const auto &mcts_result = interactive_result.mcts_result;
  const auto &spacetime_obj = interactive_result.spacetime_obj;
  const double obj_total_time = spacetime_obj->states().back().traj_point->t();
  const double t_step = GetAgentTimeStep(spacetime_obj);
  auto accel_point_list =
      CalculateMCTSAccelPointList(obj_total_time, mcts_result);
  VehicleSimulationModel model(0.0, 30.0, 0.5, 1.0);
  model.SetInitialState(mcts_result.back()->state.follower_status.s,
                        mcts_result.back()->state.follower_status.v,
                        mcts_result.back()->state.follower_status.a);
  for (double curr_t = mcts_result.back()->current_time + t_step;
       curr_t <= obj_total_time; curr_t += t_step) {
    accel_point_list.emplace_back(curr_t, model.ComputeAcceleration(curr_t));
  }
  return accel_point_list;
}

void SmoothAccelPointList(std::vector<AccelPoint> *accel_point_list) {
  constexpr int kIterationNum = 3;
  for (int it = 0; it < kIterationNum; ++it) {
    for (int i = 1; i < accel_point_list->size() - 1; ++i) {
      const double a_pre = (*accel_point_list)[i - 1].a;
      const double a_next = (*accel_point_list)[i + 1].a;

      (*accel_point_list)[i].a = (a_pre + a_next) * 0.5;
    }
  }
}

void GenerateInteractiveStBoundaryModificationInfo(
    const std::vector<MCTSInteractiveResult> &interactive_results,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>
        *modification_info) {
  for (const auto &interactive_result : interactive_results) {
    if (!interactive_result.is_valid) {
      continue;
    }
    CHECK_NOTNULL(interactive_result.st_boundary);
    CHECK_NOTNULL(interactive_result.spacetime_obj);
    auto accel_point_list = CalculateAgentAccelPointList(interactive_result);
    SmoothAccelPointList(&accel_point_list);
    CHECK(!accel_point_list.empty());
    (*modification_info)[interactive_result.traj_id] = {
        .modifier_type = StBoundaryModifierProto::MCTS,
        .decision = StBoundaryProto::UNKNOWN,
        .is_decision_changed = true,
        .accel_point_list = std::move(accel_point_list)};
  }
}

bool CheckIsExcessiveAccelOfGame(const double game_time,
                                 const DiscretizedPath &path,
                                 const SpacetimeObjectTrajectory &old_st_traj,
                                 const SpacetimeObjectTrajectory &new_st_traj) {
  const auto new_traj_p = new_st_traj.trajectory().EvaluateByTime(game_time);
  const auto old_traj_p = old_st_traj.trajectory().EvaluateByTime(game_time);
  if (!new_traj_p.has_value() || !old_traj_p.has_value()) {
    return false;
  }
  const auto new_sl_p = path.XYToSL(new_traj_p->pos());
  const auto old_sl_p = path.XYToSL(old_traj_p->pos());
  if (old_sl_p.s < new_sl_p.s + 0.5) {
    return false;
  }
  return true;
}
std::optional<StBoundaryModificationResult>
ModifyStBoundaryViaModificationInfoForMCTS(
    const StGraph &st_graph, const StBoundaryWithDecision &st_boundary_wd,
    const SpacetimeObjectTrajectory &st_traj, const DiscretizedPath &path,
    const StBoundaryModificationInfo &modification_info) {
  const bool is_mcts_modifier =
      modification_info.modifier_type == StBoundaryModifierProto::MCTS;
  const auto modifier_type = StBoundaryModifierProto::ModifierType_Name(
      modification_info.modifier_type);

  std::string decision_info;
  StBoundaryProto::DecisionReason decision_reason =
      StBoundaryProto::UNKNOWN_REASON;
  if (modification_info.is_decision_changed) {
    decision_info = absl::StrCat("decision changed by ", modifier_type);
    CHECK(is_mcts_modifier);
    decision_reason = StBoundaryProto::MCTS_DECIDER;
  } else {
    decision_info =
        absl::StrCat(st_boundary_wd.decision_info(),
                     " and keep it after modified by ", modifier_type);
    decision_reason = st_boundary_wd.decision_reason();
  }

  auto new_st_traj = CreateSpacetimeTrajectoryWithAccelPointList(
      modification_info.accel_point_list, st_traj);
  const double game_start_time = st_boundary_wd.st_boundary()->min_t();
  const double game_end_time = st_boundary_wd.st_boundary()->max_t();
  if (!CheckIsExcessiveAccelOfGame(game_start_time, path, st_traj,
                                   new_st_traj) ||
      !CheckIsExcessiveAccelOfGame(game_end_time, path, st_traj, new_st_traj)) {
    return std::nullopt;
  }

  bool generate_lane_change_gap = false;
  if (st_boundary_wd.raw_st_boundary()->is_protective() &&
      st_boundary_wd.raw_st_boundary()->protection_type() ==
          StBoundaryProto::LANE_CHANGE_GAP) {
    generate_lane_change_gap = true;
  }

  auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
      new_st_traj, generate_lane_change_gap, false, nullptr);
  auto &new_st_boundaries = st_boundary_output.st_boundaries;

  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  if ((st_boundary_wd.raw_st_boundary()->is_protective() &&
       st_boundary_wd.raw_st_boundary()->protection_type() ==
           StBoundaryProto::LANE_CHANGE_GAP) ||
      !st_boundary_wd.raw_st_boundary()->is_protective()) {
    st_boundaries_wd.reserve(new_st_boundaries.size());
    for (auto &st_boundary : new_st_boundaries) {
#if VRU_DEBUG
      DLOG(INFO) << "post new not protective: " << st_boundary->id()
                 << " proc: " << st_boundary->protection_type();
#endif
      if ((st_boundary_wd.raw_st_boundary()->is_protective() &&
           st_boundary_wd.raw_st_boundary()->protection_type() ==
               StBoundaryProto::LANE_CHANGE_GAP) ||
          !st_boundary->is_protective()) {
        st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
        double yield_time = 0.0;
        double extra_dist = 0.0;

        StBoundaryWithDecision new_st_boundary_wd(
            std::move(st_boundary), modification_info.decision, decision_reason,
            decision_info,
            st_boundary_wd.follow_standstill_distance() + extra_dist,
            st_boundary_wd.lead_standstill_distance(), 0.0, yield_time);
        st_boundaries_wd.emplace_back(std::move(new_st_boundary_wd));
      }
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modification_info.modifier_type});
}

void FillVruModifyTrajectory(
    std::vector<prediction::PredictedTrajectoryPoint> &modify_traj) {
  if (modify_traj.empty()) {
    return;
  }
  double theta = modify_traj.back().theta();
  if (modify_traj.size() > 1) {
    size_t traj_size = modify_traj.size();
    theta = atan2((modify_traj[traj_size - 1].pos().y() -
                   modify_traj[traj_size - 2].pos().y()),
                  (modify_traj[traj_size - 1].pos().x() -
                   modify_traj[traj_size - 2].pos().x()));
  }
  const double t_final = modify_traj.back().t();
  const double t_limit = 7.5;
  const double delta_t = 1.0;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  if (t_final < t_limit) {
    for (double t = t_final + delta_t; t < t_limit; t += delta_t) {
      const prediction::PredictedTrajectoryPoint &curr_traj_point =
          modify_traj.back();
      prediction::PredictedTrajectoryPoint next_traj_point;

      const double dx = curr_traj_point.v() * cos_theta * delta_t;
      const double next_x = curr_traj_point.pos().x() + dx;

      const double dy = curr_traj_point.v() * sin_theta * delta_t;
      const double next_y = curr_traj_point.pos().y() + dy;

      const double dist = hypot(dx, dy);
      next_traj_point.set_s(curr_traj_point.s() + dist);
      next_traj_point.set_pos(Vec2d(next_x, next_y));
      next_traj_point.set_theta(theta);
      next_traj_point.set_kappa(curr_traj_point.kappa());
      next_traj_point.set_t(t);
      next_traj_point.set_v(curr_traj_point.v());
      next_traj_point.set_a(0.0);
      if (ad_e2e::planning::math::Double::Compare(t, t_final + delta_t) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL) {
        next_traj_point.set_v(
            std::max(0.0, curr_traj_point.v() + curr_traj_point.a() * delta_t));
      }
#if VRU_DEBUG
      DLOG(INFO) << "s: " << next_traj_point.s()
                 << " v: " << next_traj_point.v()
                 << " kappa: " << next_traj_point.kappa()
                 << " t: " << next_traj_point.t()
                 << " a: " << next_traj_point.a()
                 << " x: " << next_traj_point.pos().x()
                 << " y: " << next_traj_point.pos().y()
                 << " heading: " << next_traj_point.theta() << " dx = " << dx
                 << " dy = " << dy;
#endif

      modify_traj.emplace_back(std::move(next_traj_point));
    }
  }
  return;
}

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryWithVRUNodeInfo(
    const MCTSInteractiveResult &interactive_result,
    const SpacetimeObjectTrajectory &st_object) {
  std::vector<PathPoint> path_points;
  std::vector<prediction::PredictedTrajectoryPoint> modify_traj;
  PathPoint prev_point;
  prev_point.set_s(0.0);
  double dist = 0;
  for (size_t i = 0; i < interactive_result.mcts_result.size(); i++) {
    const auto &node = interactive_result.mcts_result[i];
    const auto &state = node->state;
    const auto &follower_point = state.follower_status;
    PathPoint curr_point;
    prediction::PredictedTrajectoryPoint curr_traj_point;
    curr_point.set_x(follower_point.x);
    curr_point.set_y(follower_point.y);
    curr_point.set_theta(follower_point.heading);
    dist = i > 0 ? hypot(curr_point.x() - prev_point.x(),
                         curr_point.y() - prev_point.y())
                 : 0;
    double kappa = 0.0;
    if (i > 0 && i < interactive_result.mcts_result.size()) {
      const Vec2d curr_pos(curr_point.x(), curr_point.y());
      const Vec2d pre_pos(prev_point.x(), prev_point.y());
      kappa = NormalizeAngle(curr_point.theta() - prev_point.theta()) /
              (curr_pos - pre_pos).norm();
    }
    curr_point.set_s(prev_point.s() + dist);
    curr_traj_point.set_s(curr_point.s());
    curr_traj_point.set_pos(Vec2d(follower_point.x, follower_point.y));
    curr_traj_point.set_theta(follower_point.heading);
    curr_traj_point.set_kappa(kappa);
    curr_traj_point.set_t(node->current_time);
    curr_traj_point.set_v(follower_point.v);
    curr_traj_point.set_a(follower_point.a);
    prev_point = std::move(curr_point);
#if VRU_DEBUG
    DLOG(INFO) << "s: " << curr_traj_point.s() << " v: " << curr_traj_point.v()
               << " kappa: " << curr_traj_point.kappa()
               << " t: " << curr_traj_point.t() << " a: " << curr_traj_point.a()
               << " x: " << curr_traj_point.pos().x()
               << " y: " << curr_traj_point.pos().y()
               << " heading: " << curr_traj_point.theta();
#endif
    modify_traj.push_back(curr_traj_point);
  }
  FillVruModifyTrajectory(modify_traj);
  auto new_pred_traj = st_object.trajectory();
  *new_pred_traj.mutable_points() = std::move(modify_traj);
  return st_object.CreateTrajectoryMutatedInstance(std::move(new_pred_traj));
}

StBoundaryModificationResult ModifyVRUStBoundaryViaModificationInfoForMCTS(
    const StGraph &st_graph, const StBoundaryWithDecision &st_boundary_wd,
    const SpacetimeObjectTrajectory &st_traj,
    const StBoundaryModificationInfo &modification_info,
    const MCTSInteractiveResult &interactive_result) {
  const bool is_mcts_modifier =
      modification_info.modifier_type == StBoundaryModifierProto::MCTS;
  const auto modifier_type = StBoundaryModifierProto::ModifierType_Name(
      modification_info.modifier_type);

  std::string decision_info;
  StBoundaryProto::DecisionReason decision_reason =
      StBoundaryProto::UNKNOWN_REASON;
  if (modification_info.is_decision_changed) {
    decision_info = absl::StrCat("decision changed by ", modifier_type);
    CHECK(is_mcts_modifier);
    decision_reason = StBoundaryProto::MCTS_DECIDER;
  } else {
    decision_info =
        absl::StrCat(st_boundary_wd.decision_info(),
                     " and keep it after modified by ", modifier_type);
    decision_reason = st_boundary_wd.decision_reason();
  }

  auto new_st_traj =
      CreateSpacetimeTrajectoryWithVRUNodeInfo(interactive_result, st_traj);
  bool generate_lane_change_gap = false;
  if (st_boundary_wd.raw_st_boundary()->is_protective() &&
      st_boundary_wd.raw_st_boundary()->protection_type() ==
          StBoundaryProto::LANE_CHANGE_GAP) {
    generate_lane_change_gap = true;
  }

  auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
      new_st_traj, generate_lane_change_gap, false, nullptr);
  auto &new_st_boundaries = st_boundary_output.st_boundaries;

  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  if ((st_boundary_wd.raw_st_boundary()->is_protective() &&
       st_boundary_wd.raw_st_boundary()->protection_type() ==
           StBoundaryProto::LANE_CHANGE_GAP) ||
      !st_boundary_wd.raw_st_boundary()->is_protective()) {
    st_boundaries_wd.reserve(new_st_boundaries.size());
    for (auto &st_boundary : new_st_boundaries) {
      if ((st_boundary_wd.raw_st_boundary()->is_protective() &&
           st_boundary_wd.raw_st_boundary()->protection_type() ==
               StBoundaryProto::LANE_CHANGE_GAP) ||
          !st_boundary->is_protective()) {
        st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
        double yield_time = 0.0;
        double extra_dist = 0.0;

        StBoundaryWithDecision new_st_boundary_wd(
            std::move(st_boundary), modification_info.decision, decision_reason,
            decision_info,
            st_boundary_wd.follow_standstill_distance() + extra_dist,
            st_boundary_wd.lead_standstill_distance(), 0.0, yield_time);
        st_boundaries_wd.emplace_back(std::move(new_st_boundary_wd));
      }
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modification_info.modifier_type});
}

std::optional<StBoundaryModificationResult> PostModifyStBoundaryForMCTS(
    const PostStboundaryModifierInput &input,
    const StBoundaryWithDecision &st_boundary_wd) {
  if (nullptr == input.path) {
    return std::nullopt;
  }
  const auto &st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return std::nullopt;
  }
  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) {
    return std::nullopt;
  }

  if (st_boundary_wd.raw_st_boundary()->object_type() ==
          StBoundaryProto::PEDESTRIAN ||
      st_boundary_wd.raw_st_boundary()->object_type() ==
          StBoundaryProto::CYCLIST) {
    return std::nullopt;
  }

  const auto &traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());
  const auto *modification_info =
      FindOrNull(*input.modification_info_map, *traj_id);
  if (modification_info == nullptr) return std::nullopt;
  const auto *traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  return ModifyStBoundaryViaModificationInfoForMCTS(
      *input.st_graph, st_boundary_wd, *traj, *input.path, *modification_info);
}

std::optional<StBoundaryModificationResult> PostModifyVRUStBoundaryForMCTS(
    const PostStboundaryModifierInput &input,
    const MCTSInteractiveResult &interactive_result,
    const StBoundaryWithDecision &st_boundary_wd) {
  const auto &st_boundary = *st_boundary_wd.raw_st_boundary();
#if VRU_DEBUG
  DLOG(INFO) << "post modify vru st boundary by mcts "
             << int(st_boundary.source_type());
#endif
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return std::nullopt;
  }

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) {
    return std::nullopt;
  }

  if (st_boundary_wd.raw_st_boundary()->object_type() ==
      StBoundaryProto::VEHICLE) {
    return std::nullopt;
  }

  const auto &traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());
  const auto *modification_info =
      FindOrNull(*input.modification_info_map, *traj_id);
  if (modification_info == nullptr) {
    DLOG(INFO) << "modification_info is null, traj_id: " << *traj_id;
    return std::nullopt;
  }
  const auto *traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  return ModifyVRUStBoundaryViaModificationInfoForMCTS(
      *input.st_graph, st_boundary_wd, *traj, *modification_info,
      interactive_result);
}

void GenerateMCTSInteractiveResultMap(
    const std::vector<MCTSInteractiveResult> &interactive_results,
    std::unordered_map<std::string, MCTSInteractiveResult>
        &mcts_interactive_result_map) {
  for (size_t i = 0; i < interactive_results.size(); i++) {
    const auto &interactive_result = interactive_results[i];
    mcts_interactive_result_map.insert(
        std::make_pair(interactive_result.traj_id, interactive_result));
  }
  return;
}

void ModifyStBoundaries(
    const PostStboundaryModifierInput &input,
    const std::vector<MCTSInteractiveResult> &interactive_results,
    std::vector<StBoundaryWithDecision> *st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        *processed_st_objects) {
  CHECK_NOTNULL(input.st_graph);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.modification_info_map);
  const auto &modification_info_map = *input.modification_info_map;

  for (const auto &[id, _] : modification_info_map) {
    DCHECK(processed_st_objects->find(id) == processed_st_objects->end())
        << " processed_st_objects contains " << id
        << " in modification_info_map";
  }

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      interactive_processed_st_objects;
  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PostStboundaryModifierInput &, const StBoundaryWithDecision &)>(
          PostModifyStBoundaryForMCTS),
      &interactive_processed_st_objects, st_boundaries_wd);
#if VRU_DEBUG
  DLOG(INFO) << "after post mod: " << interactive_processed_st_objects.size();
#endif
  std::unordered_map<std::string, MCTSInteractiveResult>
      mcts_interactive_result_map;
  GenerateMCTSInteractiveResultMap(interactive_results,
                                   mcts_interactive_result_map);
  if (mcts_interactive_result_map.empty()) {
    DLOG(INFO) << "mcts_interactive_result_map is empty";
    return;
  }

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PostStboundaryModifierInput &, const MCTSInteractiveResult &,
          const StBoundaryWithDecision &)>(PostModifyVRUStBoundaryForMCTS),
      mcts_interactive_result_map, &interactive_processed_st_objects,
      st_boundaries_wd);
#if VRU_DEBUG
  DLOG(INFO) << "after post mod vru: "
             << interactive_processed_st_objects.size();
#endif
  // Merge newly processed trajectories with the original ones.
  for (auto &[traj_id, traj] : interactive_processed_st_objects) {
#if VRU_DEBUG
    DLOG(INFO) << "after post mod: " << traj_id;
#endif
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

}  // namespace

absl::Status MakeMCTSGameDecision(
    const SpeedPlanningParamsProto &speed_planning_params,
    const VehicleGeometryParamsProto &vehicle_geom, const StGraph &st_graph,
    const DiscretizedPath &path, const double av_velocity, const double av_acc,
    const SpacetimeTrajectoryManager &st_traj_mgr, const uint64_t seq_num,
    WorkerThreadManager *thread_pool,
    std::vector<StBoundaryWithDecision> *st_boundaries_with_decision,
    std::vector<MCTSInteractiveResult> &interactive_results) {
  if (!speed_planning_params.enable_mcts_game()) {
    return absl::OkStatus();
  }
  if (path.empty()) {
    return absl::OkStatus();
  }
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;

  std::vector<InteractiveInput> interactive_st_boundaries;
  interactive_st_boundaries.reserve(st_boundaries_with_decision->size());
  GetInteractiveSTBoundaries(vehicle_geom, st_traj_mgr,
                             st_boundaries_with_decision, processed_st_objects,
                             path.front(), interactive_st_boundaries);

  interactive_results.clear();
  ProcessMCTSGame(interactive_st_boundaries, st_traj_mgr, path, av_velocity,
                  av_acc, seq_num, &speed_planning_params, thread_pool,
                  interactive_results);

  if (!FilterInvalidMCTSInteractiveResults(
          speed_planning_params, interactive_results, st_traj_mgr, path)) {
    DLOG(INFO) << "Filter invalid interactive results failed:"
               << interactive_results.size();
    return absl::OkStatus();
  }

  absl::flat_hash_map<std::string, StBoundaryModificationInfo>
      modification_info;
  GenerateInteractiveStBoundaryModificationInfo(interactive_results,
                                                &modification_info);

  const PostStboundaryModifierInput modifier_input{
      .st_graph = &st_graph,
      .st_traj_mgr = &st_traj_mgr,
      .modification_info_map = &modification_info,
      .path = &path};
  ModifyStBoundaries(modifier_input, interactive_results,
                     st_boundaries_with_decision, &processed_st_objects);

  return absl::OkStatus();
}
}  // namespace e2e_noa::planning
