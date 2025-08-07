#include "speed/speed_optimizer_object_manager.h"

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "object/planner_object.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "speed/st_boundary.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
namespace e2e_noa::planning {
namespace {
using ClassifiedObjects = std::vector<std::optional<SpeedOptimizerObject>>;

using ClassifiedOverlapState = std::vector<std::optional<ObjectOverlapState>>;

constexpr int kObjectTypeNum = 3;

const PiecewiseLinearFunction<double, double>
    kAvSpeedFollowDistanceLowestGainPlf = {{0.0, 6.0, 10.0}, {0.9, 0.6, 0.1}};
const PiecewiseLinearFunction<double, double> KFollowTimeHeadwayGainPlf = {
    {3.0, 6.0, 10.0, 15.0, 20.0}, {0.7, 0.85, 1.0, 1.0, 1.0}};
const PiecewiseLinearFunction<double, double> KGapOVertakeTimeGainPlf = {
    {5.0, 7.0}, {0.4, 1.0}};
const double KMinHeadwayTime = 1.1;

void ModifyObjectLonBufferForSpecificScenario(
    const std::string& id,
    const std::vector<const StBoundaryWithDecision*>& st_boundaries_wd,
    std::optional<ObjectOverlapState>& overlap_state) {
  if (st_boundaries_wd.empty() || (!overlap_state.has_value())) {
    return;
  }

  const auto obj_scenario_info =
      st_boundaries_wd[0]->st_boundary()->obj_scenario_info();
  const bool is_left_turn_to_staight =
      (obj_scenario_info.interaction_zone == InteractionZone::TurnLeft) &&
      (obj_scenario_info.relationship == Relationship::Cross ||
       obj_scenario_info.relationship == Relationship::OnComing);
  if (is_left_turn_to_staight) {
    overlap_state.value().lon_buffer =
        std::max(4.0, overlap_state.value().lon_buffer);
  }
}

void FillOverlapStateLonBuffer(
    double time, const std::vector<std::optional<double>>& standstills,
    double follow_time_headway,
    const std::optional<PiecewiseLinearFunction<double>>&
        follow_rel_speed_gain_plf,
    double lead_time_headway, double av_speed,
    StBoundaryProto::ProtectionType protection_type,
    ClassifiedOverlapState* overlap_states, const std::string& id,
    const std::vector<const StBoundaryWithDecision*>& st_boundaries_wd,
    const LaneChangeStage& lc_stage) {
  CHECK_NOTNULL(overlap_states);
  for (int type = 0; type < overlap_states->size(); ++type) {
    auto& state = (*overlap_states)[type];
    if (!state.has_value()) {
      continue;
    }
    const double obj_speed = std::max(0.0, state->speed);
    double m_follow_time_headway =
        follow_time_headway * KFollowTimeHeadwayGainPlf(av_speed);
    m_follow_time_headway =
        std::clamp(m_follow_time_headway, KMinHeadwayTime, follow_time_headway);
    switch (type) {
      case SpeedOptimizerObjectType::MOVING_FOLLOW:
        CHECK(follow_rel_speed_gain_plf.has_value());
        CHECK(standstills[SpeedOptimizerObjectType::MOVING_FOLLOW].has_value());
        state->lon_buffer =
            protection_type == StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
                ? *standstills[SpeedOptimizerObjectType::MOVING_FOLLOW]
                : (obj_speed * m_follow_time_headway +
                   *standstills[SpeedOptimizerObjectType::MOVING_FOLLOW]) *
                      (*follow_rel_speed_gain_plf)(obj_speed - av_speed);
        ModifyObjectLonBufferForSpecificScenario(id, st_boundaries_wd, state);
        break;
      case SpeedOptimizerObjectType::MOVING_LEAD:
        CHECK(standstills[SpeedOptimizerObjectType::MOVING_LEAD].has_value());
        state->lon_buffer =
            protection_type == StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
                ? *standstills[SpeedOptimizerObjectType::MOVING_LEAD]
                : obj_speed * lead_time_headway +
                      *standstills[SpeedOptimizerObjectType::MOVING_LEAD];
        if (protection_type == StBoundaryProto::LANE_CHANGE_GAP ||
            lc_stage == LaneChangeStage::LCS_EXECUTING) {
          constexpr double kAcc = 1.0;
          const double v_at_time = std::max(av_speed + kAcc * time, 0.0);
          const double s_at_time =
              std::max(av_speed * time + 0.5 * kAcc * time * time, 0.0);
          state->lon_buffer =
              obj_speed * lead_time_headway +
              *standstills[SpeedOptimizerObjectType::MOVING_LEAD];
          state->lon_buffer *= KGapOVertakeTimeGainPlf(time);
          if (s_at_time < state->lon_buffer + state->bound) {
            state->lon_buffer = std::max(0.05, s_at_time - state->bound);
          }
        }
        break;
      case SpeedOptimizerObjectType::STATIONARY:
        CHECK(standstills[SpeedOptimizerObjectType::STATIONARY].has_value());
        state->lon_buffer = *standstills[SpeedOptimizerObjectType::STATIONARY];
        break;
    }
  }
}

ClassifiedOverlapState IntegrateAndClassifyOverlapState(
    const std::vector<const StBoundaryWithDecision*>& st_boundaries,
    double time, double prediction_impact_factor,
    const std::optional<std::pair<std::string, double>>& maybe_hard_brake_obj,
    const double av_speed,
    const SpeedPlanningParamsProto& speed_planning_params) {
  const auto integrate_overlap_state =
      [](const ObjectOverlapState& new_state,
         std::optional<ObjectOverlapState>* raw_state) {
        CHECK_NOTNULL(raw_state);
        if (!raw_state->has_value()) *raw_state = ObjectOverlapState{};
        (*raw_state)->bound += new_state.bound * new_state.prob;
        (*raw_state)->speed += new_state.speed * new_state.prob;
        (*raw_state)->delta_speed_factor +=
            new_state.delta_speed_factor * new_state.prob;
        (*raw_state)->prob += new_state.prob;
      };

  ClassifiedOverlapState classified_overlap_states(kObjectTypeNum);
  double min_lower_bound = std::numeric_limits<double>::max();
  double delta_speed_factor = 0.0;
  const auto& speed_opt_params = speed_planning_params.speed_optimizer_params();
  for (const StBoundaryWithDecision* stb_wd : st_boundaries) {
    const StBoundary* stb = stb_wd->st_boundary();
    const auto& raw_st = stb_wd->raw_st_boundary();
    double acc = 0.0;
    if (raw_st) {
      acc = raw_st->obj_pose_info().a();
    }
    const auto s_range = stb->GetBoundarySRange(time);
    if (!s_range.has_value()) continue;
    double bound = 0.0;
    const auto decision = stb_wd->decision_type();
    if (decision == StBoundaryProto::YIELD ||
        decision == StBoundaryProto::FOLLOW) {
      bound = s_range->second;
      min_lower_bound = std::min(min_lower_bound, s_range->second);
      if (decision == StBoundaryProto::FOLLOW) {
        const double nomal_delta_speed_factor =
            speed_opt_params.delta_speed_factor();
        const double middle_delta_speed_factor =
            speed_opt_params.middle_follow_delta_speed_factor();
        const double hard_delta_speed_factor =
            speed_opt_params.hard_follow_delta_speed_factor();
        double max_speed_factor = middle_delta_speed_factor;
        if ((!speed_planning_params.enable_hard_brake_for_every_obj()) &&
            maybe_hard_brake_obj.has_value() && stb->object_id().has_value() &&
            maybe_hard_brake_obj.value().first == stb->object_id().value()) {
          const double max_speed_base_factor =
              Lerp(middle_delta_speed_factor, 0.0, hard_delta_speed_factor,
                   -0.8, maybe_hard_brake_obj.value().second, true);

          const double delta_v = raw_st->obj_pose_info().v() - av_speed;
          max_speed_factor = Lerp(middle_delta_speed_factor, 0.0,
                                  max_speed_base_factor, -1.5, delta_v, true);

          delta_speed_factor =
              Lerp(nomal_delta_speed_factor,
                   speed_opt_params.hard_brake_start_acc(), max_speed_factor,
                   speed_opt_params.hard_brake_end_acc(), acc, true);
        } else {
          if (speed_planning_params.enable_hard_brake_for_every_obj()) {
            const double delta_v = raw_st->obj_pose_info().v() - av_speed;
            max_speed_factor =
                Lerp(middle_delta_speed_factor, 1.5, hard_delta_speed_factor,
                     0.0, delta_v, true);

            delta_speed_factor =
                Lerp(nomal_delta_speed_factor,
                     speed_opt_params.hard_brake_start_acc(), max_speed_factor,
                     speed_opt_params.hard_brake_end_acc(), acc, true);
          } else {
            delta_speed_factor = Lerp(nomal_delta_speed_factor, -2.0,
                                      max_speed_factor, -4.0, acc, true);
          }
        }
      } else {
        delta_speed_factor = speed_opt_params.delta_speed_factor();
      }

      if (raw_st) {
        if (raw_st->object_type() == StBoundaryProto::VIRTUAL &&
            raw_st->is_traffic_light()) {
          delta_speed_factor = speed_opt_params.stop_line_delta_speed_factor();
          double factor = Lerp(
              1.0, speed_opt_params.stop_line_delta_speed_enable_distance(),
              speed_opt_params.min_stop_line_decay(),
              speed_opt_params.stop_line_delta_speed_disable_distance(),
              raw_st->bottom_left_point().s(), true);
          delta_speed_factor = delta_speed_factor * factor;
        }
      }
    } else {
      bound = s_range->first;
    }
    const auto obj_speed = stb->GetStBoundarySpeedAtT(time);
    CHECK(obj_speed.has_value());

    const ObjectOverlapState tmp_overlap_state = {
        .bound = bound,
        .speed = *obj_speed,
        .prob = stb->probability(),
        .delta_speed_factor = delta_speed_factor};
    if (stb->is_stationary()) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::STATIONARY]);
    } else if (decision == StBoundaryProto::YIELD ||
               decision == StBoundaryProto::FOLLOW) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::MOVING_FOLLOW]);
    } else if (decision == StBoundaryProto::OVERTAKE) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::MOVING_LEAD]);
    }
  }

  for (int type = 0; type < classified_overlap_states.size(); ++type) {
    auto& state = classified_overlap_states[type];
    if (!state.has_value()) continue;
    const double prob = state->prob;
    CHECK_GT(prob, 0.0);
    state->bound /= prob;
    state->speed /= prob;
    if (type == SpeedOptimizerObjectType::MOVING_FOLLOW ||
        type == SpeedOptimizerObjectType::STATIONARY) {
      state->bound =
          Lerp(state->bound, min_lower_bound, prediction_impact_factor);
    }
  }
  return classified_overlap_states;
}

std::optional<std::pair<std::string, double>> MaybeHardBrakeObjInfo(
    const std::map<std::string, std::vector<const StBoundaryWithDecision*>>&
        st_boundaries_wd_map) {
  constexpr double kFrontLatThres = 0.2;
  std::vector<const StBoundaryWithDecision*> front_vehicles;
  std::string nearest_follow_obj = "";
  double min_dis = DBL_MAX;
  double nearest_follow_speed = 0.0;
  double min_stop_line_dis = DBL_MAX;

  for (const auto& [id, st_boundaries] : st_boundaries_wd_map) {
    if (st_boundaries.empty()) continue;
    const auto& stb_wd = st_boundaries.front();
    if (!stb_wd) continue;
    const auto& raw_st_boundary = stb_wd->raw_st_boundary();
    if (!raw_st_boundary) continue;
    if ((raw_st_boundary->object_type() == StBoundaryProto::VEHICLE ||
         raw_st_boundary->object_type() == StBoundaryProto::CYCLIST ||
         raw_st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) &&
        (stb_wd->decision_type() == StBoundaryProto::YIELD ||
         stb_wd->decision_type() == StBoundaryProto::FOLLOW) &&
        std::abs(stb_wd->dl()) < kFrontLatThres && stb_wd->ds() > 1e-2) {
      front_vehicles.emplace_back(stb_wd);
      if (stb_wd->decision_type() == StBoundaryProto::FOLLOW &&
          stb_wd->ds() > 0.0 && stb_wd->object_id().has_value()) {
        if (stb_wd->ds() < min_dis) {
          nearest_follow_obj = stb_wd->object_id().value();
          min_dis = stb_wd->ds();
          nearest_follow_speed = stb_wd->obj_pose_info().v();
        }
      }
    }
    if (raw_st_boundary->is_traffic_light() &&
        raw_st_boundary->bottom_left_point().s() < min_stop_line_dis) {
      min_stop_line_dis = raw_st_boundary->bottom_left_point().s();
    }
  }

  if (nearest_follow_obj.empty()) {
    return std::nullopt;
  } else if (min_dis > min_stop_line_dis) {
    return std::nullopt;
  }
  front_vehicles.erase(
      std::remove_if(front_vehicles.begin(), front_vehicles.end(),
                     [&nearest_follow_obj,
                      &min_dis](const StBoundaryWithDecision* stb_wd) {
                       return stb_wd->object_id().has_value() &&
                              (stb_wd->object_id().value() ==
                                   nearest_follow_obj ||
                               stb_wd->ds() < min_dis);
                     }),
      front_vehicles.end());

  std::sort(front_vehicles.begin(), front_vehicles.end(),
            [](const StBoundaryWithDecision* stb_wd1,
               const StBoundaryWithDecision* stb_wd2) {
              return stb_wd1->ds() < stb_wd2->ds();
            });

  double follow_brake_a_with_front = 0.0;
  double follow_brake_a_with_stopline = 0.0;
  if (!front_vehicles.empty() &&
      front_vehicles.front()->obj_pose_info().v() < Kph2Mps(2.0)) {
    follow_brake_a_with_front =
        -1.0 * Sqr(nearest_follow_speed) /
        (std::max(1e-2, 2.0 * (front_vehicles.front()->ds() - 6.0 - min_dis)));
  }
  follow_brake_a_with_stopline =
      -1.0 * Sqr(nearest_follow_speed) /
      (std::max(1e-2, 2.0 * (min_stop_line_dis - 6.0 - min_dis)));

  std::pair<std::string, double> obj_info = std::make_pair(
      nearest_follow_obj,
      std::min(follow_brake_a_with_front, follow_brake_a_with_stopline));
  return std::optional<std::pair<std::string, double>>(obj_info);
}

ClassifiedObjects GenerateIntegratedClassifiedObjects(
    const std::vector<const StBoundaryWithDecision*>& st_boundaries_wd,
    const std::optional<std::pair<std::string, double>>& maybe_hard_brake_obj,
    const SpacetimeTrajectoryManager& traj_mgr, const std::string& id,
    double delta_t, double plan_total_time, double av_speed,
    const SpeedPlanningParamsProto& speed_planning_params,
    const LaneChangeStage& lc_stage) {
  CHECK(!st_boundaries_wd.empty());

  const auto protection_type =
      st_boundaries_wd[0]->st_boundary()->protection_type();

  double min_t = std::numeric_limits<double>::max();
  double max_t = -std::numeric_limits<double>::max();
  std::vector<std::optional<double>> standstills(kObjectTypeNum);
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    const StBoundary* st_boundary = stb_wd->st_boundary();

    CHECK_EQ(st_boundary->protection_type(), protection_type);
    min_t = std::min(min_t, st_boundary->min_t());
    max_t = std::max(max_t, st_boundary->max_t());
    if (st_boundary->is_stationary()) {
      standstills[SpeedOptimizerObjectType::STATIONARY] =
          stb_wd->follow_standstill_distance();
    } else if (stb_wd->decision_type() == StBoundaryProto::YIELD ||
               stb_wd->decision_type() == StBoundaryProto::FOLLOW) {
      standstills[SpeedOptimizerObjectType::MOVING_FOLLOW] =
          stb_wd->follow_standstill_distance();
    } else if (stb_wd->decision_type() == StBoundaryProto::OVERTAKE) {
      standstills[SpeedOptimizerObjectType::MOVING_LEAD] =
          stb_wd->lead_standstill_distance();
    }
  }

  std::optional<PiecewiseLinearFunction<double>> follow_rel_speed_gain_plf;
  const double driving_style_factor =
      speed_planning_params.speed_optimizer_params().follow_lon_buffer_factor();
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    const StBoundary* st_boundary = stb_wd->st_boundary();
    if (stb_wd->decision_type() == StBoundaryProto::OVERTAKE ||
        st_boundary->is_stationary()) {
      continue;
    }
    const auto& object_id = st_boundary->object_id();
    CHECK(object_id.has_value());
    const double obj_v =
        CHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id))->pose().v();

    if (CHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id))->pose().a() >
            -0.15 &&
        lc_stage == LaneChangeStage::LCS_EXECUTING) {
      follow_rel_speed_gain_plf = PiecewiseLinearFunction<double>{
          {0.5, 0.8, 1.5},
          {1.0 * driving_style_factor, 0.7 * driving_style_factor,
           kAvSpeedFollowDistanceLowestGainPlf(obj_v) * driving_style_factor}};
    } else {
      follow_rel_speed_gain_plf = PiecewiseLinearFunction<double>{
          {2.5, 4.0},
          {1.0 * driving_style_factor,
           kAvSpeedFollowDistanceLowestGainPlf(obj_v) * driving_style_factor}};
    }
    break;
  }

  const auto& opt_params = speed_planning_params.speed_optimizer_params();
  const double lead_time_headway = speed_planning_params.lead_time_headway();
  const int knot_num = opt_params.knot_num();
  const double prediction_impact_factor = opt_params.prediction_impact_factor();

  std::vector<std::vector<std::optional<ObjectOverlapState>>>
      classified_entire_time_overlap_states(kObjectTypeNum);
  max_t = std::min(max_t, plan_total_time);
  const int start_idx = FloorToInt(min_t / delta_t);
  const int end_idx = FloorToInt(max_t / delta_t);
  const double follow_time_headway =
      st_boundaries_wd[0]->st_boundary()->is_large_vehicle()
          ? speed_planning_params.large_vehicle_follow_time_headway()
          : speed_planning_params.follow_time_headway();
  for (int idx = start_idx; idx <= end_idx; ++idx) {
    auto overlap_states_at_t = IntegrateAndClassifyOverlapState(
        st_boundaries_wd, idx * delta_t, prediction_impact_factor,
        maybe_hard_brake_obj, av_speed, speed_planning_params);

    FillOverlapStateLonBuffer(idx * delta_t, standstills, follow_time_headway,
                              follow_rel_speed_gain_plf, lead_time_headway,
                              av_speed, protection_type, &overlap_states_at_t,
                              id, st_boundaries_wd, lc_stage);
    CHECK_LT(idx, knot_num);
    for (int type = 0; type < overlap_states_at_t.size(); ++type) {
      auto state = overlap_states_at_t[type];
      if (state.has_value()) {
        auto& entire_time_states = classified_entire_time_overlap_states[type];
        if (entire_time_states.empty()) entire_time_states.resize(knot_num);
        entire_time_states[idx] = state;
      }
    }
  }

  const auto* stb = st_boundaries_wd.front()->st_boundary();
  const auto object_type = stb->object_type();
  const auto source_type = stb->source_type();
  ClassifiedObjects classified_objects(kObjectTypeNum);
  for (int type = 0; type < classified_objects.size(); ++type) {
    auto& states = classified_entire_time_overlap_states[type];
    if (!states.empty()) {
      const auto standstill = standstills[type];
      CHECK(standstill.has_value());
      classified_objects[type] =
          SpeedOptimizerObject(std::move(states), delta_t, id, *standstill,
                               object_type, source_type, protection_type);
    }
  }
  return classified_objects;
}

}  // namespace

SpeedOptimizerObjectManager::SpeedOptimizerObjectManager(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr, double av_speed,
    double plan_total_time, double plan_time_interval,
    const SpeedPlanningParamsProto& speed_planning_params,
    const LaneChangeStage& lc_stage) {
  std::map<std::string, std::vector<const StBoundaryWithDecision*>>
      st_boundary_map;
  for (const StBoundaryWithDecision& stb_wd : st_boundaries_with_decision) {
    if (stb_wd.decision_type() == StBoundaryProto::UNKNOWN ||
        stb_wd.decision_type() == StBoundaryProto::IGNORE) {
      continue;
    }
    const auto id = GetStBoundaryIntegrationId(*stb_wd.st_boundary());
    st_boundary_map[id].push_back(&stb_wd);
  }

  objects_.resize(kObjectTypeNum);
  const auto& maybe_hard_brake_obj_info =
      MaybeHardBrakeObjInfo(st_boundary_map);
  if (maybe_hard_brake_obj_info.has_value()) {
    if (maybe_hard_brake_obj_info.value().second < -0.5) {
      attention_obj_id_ = maybe_hard_brake_obj_info.value().first;
    }
  }
  for (const auto& [id, st_boundaries] : st_boundary_map) {
    auto classified_objects = GenerateIntegratedClassifiedObjects(
        st_boundaries, maybe_hard_brake_obj_info, traj_mgr, id,
        plan_time_interval, plan_total_time, av_speed, speed_planning_params,
        lc_stage);
    for (int type = 0; type < classified_objects.size(); ++type) {
      auto object = classified_objects[type];
      if (object.has_value()) {
        objects_[type].push_back(std::move(*object));
      }
    }
  }
}

}  // namespace e2e_noa::planning
