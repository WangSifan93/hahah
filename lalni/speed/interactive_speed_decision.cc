#include "speed/interactive_speed_decision.h"

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
#include "speed/overlap_info.h"
#include "speed/speed_point.h"
#include "speed/st_boundary.h"
#include "speed/st_graph_data.h"
#include "speed/st_point.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

DECLARE_bool(enable_interactive_speed_decision_draw_st_traj);

namespace e2e_noa::planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kBillion = 1e9;
constexpr double kEps = 1e-6;
constexpr double kMaxDecelMargin = -0.4;
constexpr double kMaxAccMargin = 0.2;
#define DEBUG_GAME_THEORY (0)

const PiecewiseLinearFunction<double> kLowPrioDecelDiscomfortPlf(
    {-3.5 - kEps, -3.5, -2.0, -1.0, -0.0}, {kInf, 40.0, 15.0, 5.0, 0.0});
const PiecewiseLinearFunction<double> kEqualPrioDecelDiscomfortPlf(
    {-2.2 - kEps, -2.2, -2.0, -1.0, -0.0}, {kInf, 35.0, 15.0, 5.0, 0.0});
const PiecewiseLinearFunction<double> kHighPrioDecelDiscomfortPlf(
    {-1.5 - kEps, -1.5, -1.0, -0.5, -0.0}, {kInf, 30.0, 10.0, 5.0, 0.0});
const PiecewiseLinearFunction<double> kLargeVehicleDecelDiscomfortPlf(
    {-0.5 - kEps, -0.5, -0.0}, {kInf, 40.0, 0.0});

const PiecewiseLinearFunction<double> kAgentWaitingDiscomfortPlf(
    {0.0, 0.8, 0.8 + kEps}, {0.0, 25.0, kInf});
const PiecewiseLinearFunction<double> kAgentAggresiveWaitingDiscomfortPlf(
    {0.0, 4.0, 4.0 + kEps}, {0.0, 25.0, kInf});

const PiecewiseLinearFunction<double> kAvLowPrioDecelDiscomfortPlf(
    {-3.5 - kEps, -3.5, -2.0, -1.0, -0.0}, {kBillion, 40.0, 15.0, 5.0, 0.0});
const PiecewiseLinearFunction<double> kAvEqualPrioDecelDiscomfortPlf(
    {-2.2 - kEps, -2.2, -2.0, -1.0, -0.0}, {kBillion, 35.0, 15.0, 5.0, 0.0});

const PiecewiseLinearFunction<double> kAvWaitingDiscomfortPlf(
    {0.0, 0.8, 0.8 + kEps}, {0.0, 25.0, kBillion});
const PiecewiseLinearFunction<double> kAvAggresiveWaitingDiscomfortPlf(
    {0.0, 4.0, 4.0 + kEps}, {0.0, 25.0, kBillion});

struct YieldingResult {
  bool already_overtake = false;
  bool already_yield = false;
  bool has_yield_intention = false;
  bool is_parallel_front_merging = false;
  double start_yield_time = 0.0;
  double arrive_yielding_point_time = 0.0;
  double arrive_yielding_point_decel = 0.0;
  double v_at_yielding_point = 0.0;
  double s_at_yielding_point = 0.0;
  double waiting_duration = 0.0;
  double finish_yield_time = 0.0;
};

struct InteractiveResult {
  double cost = 0.0;
  std::optional<YieldingResult> yielding_result = std::nullopt;

  std::string traj_id;
  StBoundaryProto::DecisionType decision = StBoundaryProto::UNKNOWN;

  const SpacetimeObjectTrajectory* spacetime_obj = nullptr;
  const StBoundary* st_boundary = nullptr;
};

inline idm::Parameters GetIDMParams(double v_desire) {
  return idm::Parameters{.v_desire = v_desire,
                         .s_min = 4.0,
                         .t_desire = 2.5,
                         .acc_max = 2.0,
                         .comfortable_brake = 2.0,
                         .brake_max = 4.0,
                         .delta = 4.0};
}

double CalculateConsistencyCost(
    absl::Span<const StBoundaryWithDecision* const> st_boundaries,
    const SpeedVector& speed_profile) {
  double cost = 0.0;
  return cost;
}

std::vector<AccelPoint> GenerateFreeDrivingAccelPointList(double start_time,
                                                          double total_time) {
  constexpr double kFreeDriveingAccelDuration = 1.5;
  constexpr double kObjectFreeAccel = 0.8;
  std::vector<AccelPoint> accel_point_list;
  accel_point_list.emplace_back(start_time, kObjectFreeAccel);
  if (total_time - start_time > kFreeDriveingAccelDuration) {
    accel_point_list.emplace_back(start_time + kFreeDriveingAccelDuration, 0.0);
  }
  return accel_point_list;
}

absl::StatusOr<BruteForceFrenetCoordinate> BuildAgentFrenetFrame(
    const SpacetimeObjectTrajectory& spacetime_obj) {
  const auto& st_traj_points = spacetime_obj.trajectory().points();
  std::vector<Vec2d> agent_traj_points_2d;
  agent_traj_points_2d.reserve(st_traj_points.size());
  for (const auto& point : st_traj_points) {
    agent_traj_points_2d.push_back(point.pos());
  }
  return BuildBruteForceFrenetFrame(agent_traj_points_2d, true);
}

idm::Parameters CalculateAgentIdmParams(
    const SpacetimeObjectTrajectory& spacetime_obj) {
  double agent_predicted_max_v = 0.0;
  for (const auto& state : spacetime_obj.states()) {
    agent_predicted_max_v =
        std::max(agent_predicted_max_v, state.traj_point->v());
  }
  return GetIDMParams(agent_predicted_max_v);
}

inline double GetAgentTimeStep(const SpacetimeObjectTrajectory& spacetime_obj) {
  return spacetime_obj.states().size() > 1
             ? spacetime_obj.states()[1].traj_point->t() -
                   spacetime_obj.states()[0].traj_point->t()
             : prediction::kPredictionTimeStep;
}

struct StartFollowState {
  double t = 0.0;
  double v = 0.0;
  double a = 0.0;
  double s = 0.0;
};

StartFollowState CalculateAgentStartFollowState(
    const SpacetimeObjectTrajectory& spacetime_obj,
    const YieldingResult& yielding_result) {
  if (yielding_result.already_yield) {
    return StartFollowState{.t = 0.0,
                            .v = spacetime_obj.planner_object().pose().v(),
                            .a = spacetime_obj.planner_object().pose().a(),
                            .s = 0.0};
  }
  const double obj_total_time = spacetime_obj.states().back().traj_point->t();

  constexpr double kAgentSlidingDuration = 0.5;
  return StartFollowState{
      .t = std::min(yielding_result.finish_yield_time + kAgentSlidingDuration,
                    obj_total_time),
      .v = yielding_result.v_at_yielding_point,
      .a = 0.0,
      .s = yielding_result.s_at_yielding_point +
           yielding_result.v_at_yielding_point * kAgentSlidingDuration};
}

std::vector<AccelPoint> CalculateAccelPointListBeforeFollow(
    const SpacetimeObjectTrajectory& spacetime_obj,
    const YieldingResult& yielding_result) {
  if (yielding_result.already_yield) {
    return {AccelPoint(0.0, spacetime_obj.planner_object().pose().a())};
  }

  std::vector<AccelPoint> accel_point_list;
  accel_point_list.reserve(3);

  accel_point_list.emplace_back(yielding_result.start_yield_time,
                                yielding_result.arrive_yielding_point_decel);

  if (yielding_result.waiting_duration > 0.0) {
    accel_point_list.emplace_back(
        yielding_result.arrive_yielding_point_time,
        0.5 * yielding_result.arrive_yielding_point_decel);
  }
  const double obj_total_time = spacetime_obj.states().back().traj_point->t();
  if (yielding_result.finish_yield_time > obj_total_time) {
    return accel_point_list;
  }

  accel_point_list.emplace_back(yielding_result.finish_yield_time, 0.0);
  return accel_point_list;
}

inline bool IsAgentFreeDriving(const SpacetimeObjectTrajectory& spacetime_obj,
                               const DiscretizedPath& path,
                               const OverlapInfo& end_overlap, double agent_s,
                               double av_s) {
  const double agent_end_overlap_s =
      spacetime_obj.states()[end_overlap.obj_idx].traj_point->s();
  const double av_end_overlap_s = path[end_overlap.av_end_idx].s();
  const bool is_agent_traj_stay_on_av_path =
      static_cast<int>(spacetime_obj.states().size()) - 1 ==
      end_overlap.obj_idx;

  return agent_s > agent_end_overlap_s ||
         (av_s > av_end_overlap_s && !is_agent_traj_stay_on_av_path);
}

struct AvState {
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double theta = 0.0;
  Vec2d pos;
};

inline AvState CreateAvState(const DiscretizedPath& path,
                             const SpeedVector& speed_profile, double curr_t) {
  CHECK_LE(curr_t, speed_profile.back().t());

  const auto av_speed_point = speed_profile.EvaluateByTime(curr_t);
  CHECK(av_speed_point.has_value());
  const auto av_path_point = path.Evaluate(av_speed_point->s());
  return AvState{.s = av_speed_point->s(),
                 .v = av_speed_point->v(),
                 .a = av_speed_point->a(),
                 .theta = av_path_point.theta(),
                 .pos = Vec2d(av_path_point.x(), av_path_point.y())};
}

std::vector<AccelPoint> CalculateAgentAccelPointList(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeObjectTrajectory& spacetime_obj,
    const OverlapInfo& end_overlap, const DiscretizedPath& path,
    const SpeedVector& speed_profile, const YieldingResult& yielding_result) {
  CHECK_GE(speed_profile.size(), 2);

  const StartFollowState start_follow_state =
      CalculateAgentStartFollowState(spacetime_obj, yielding_result);
  auto accel_point_list =
      CalculateAccelPointListBeforeFollow(spacetime_obj, yielding_result);
  const double t_step = GetAgentTimeStep(spacetime_obj);
  const double obj_total_time = spacetime_obj.states().back().traj_point->t();
  accel_point_list.reserve(
      3 + FloorToInt((obj_total_time - start_follow_state.t) / t_step));
  const auto agent_ff = BuildAgentFrenetFrame(spacetime_obj);
  if (!agent_ff.ok()) return accel_point_list;
  const auto agent_idm_params = CalculateAgentIdmParams(spacetime_obj);

  double prev_v = start_follow_state.v;
  double prev_s = start_follow_state.s;
  double prev_a = start_follow_state.a;
  double begin_follow_time = start_follow_state.t;

  for (double curr_t = begin_follow_time + t_step; curr_t <= obj_total_time;
       curr_t += t_step) {
    AvState av_state;
    bool is_curr_t_exceed_av_traj = false;
    if (curr_t <= speed_profile.back().t()) {
      av_state = CreateAvState(path, speed_profile, curr_t);
    } else {
      is_curr_t_exceed_av_traj = true;
    }
    const double curr_s = prev_s + prev_v * t_step + 0.5 * prev_a * Sqr(t_step);

    if (IsAgentFreeDriving(spacetime_obj, path, end_overlap, curr_s,
                           av_state.s) ||
        is_curr_t_exceed_av_traj) {
      auto free_driving_a_list =
          GenerateFreeDrivingAccelPointList(curr_t, obj_total_time);
      std::move(free_driving_a_list.begin(), free_driving_a_list.end(),
                std::back_inserter(accel_point_list));
      break;
    }

    const auto av_back_edge_center_pos =
        av_state.pos - vehicle_geom.back_edge_to_center() *
                           Vec2d::FastUnitFromAngle(av_state.theta);
    const auto sl = agent_ff->XYToSL(av_back_edge_center_pos);
    const double ds =
        sl.s - curr_s - spacetime_obj.bounding_box().length() * 0.5;
    const double curr_v = prev_v + prev_a * t_step;

    double agent_theta = 0.0;
    const auto& st_traj_points = spacetime_obj.trajectory().points();
    if (curr_s > st_traj_points.back().s()) {
      const auto& last_p = st_traj_points.back();
      const auto& second_to_last_p = st_traj_points[st_traj_points.size() - 2];
      agent_theta =
          LerpAngle(second_to_last_p.theta(), last_p.theta(),
                    LerpFactor(second_to_last_p.s(), last_p.s(), curr_s));
    } else {
      const auto traj_point = QueryTrajectoryPointByS(st_traj_points, curr_s);
      agent_theta = traj_point.theta();
    }
    const double av_v_eff = yielding_result.has_yield_intention ? 0.8 : 1.0;
    const double agent_acc_v =
        std::max(0.0, av_v_eff * av_state.v *
                          fast_math::Cos(av_state.theta - agent_theta));
    const double dv = agent_acc_v - curr_v;
    double curr_a = ComputeIDMAcceleration(curr_v, ds, dv, agent_idm_params);
    if (yielding_result.has_yield_intention) {
      curr_a = std::min(std::min(curr_a, 0.0), start_follow_state.a);
    }

    if (av_state.a > -kEps) {
      constexpr double kKeepDistanceDecel = -0.5;
      curr_a = std::max(kKeepDistanceDecel, curr_a);
    }

    if (curr_v + curr_a * t_step < 0.0) {
      curr_a = -curr_v / t_step;
    }
    accel_point_list.emplace_back(curr_t, curr_a);

    prev_v = curr_v;
    prev_a = curr_a;
  }

  return accel_point_list;
}

double CalculateAgentReactionTime(
    const StOverlapMetaProto& overlap_meta,
    const SpacetimeObjectTrajectory& spacetime_obj) {
  if ((overlap_meta.has_is_making_u_turn() &&
       overlap_meta.is_making_u_turn()) ||
      (overlap_meta.has_is_crossing_straight_lane() &&
       overlap_meta.is_crossing_straight_lane())) {
    return 0.0;
  }

  PiecewiseLinearFunction<double> agent_reaction_time_plf;

  if (spacetime_obj.planner_object().is_large_vehicle() &&
      overlap_meta.priority() != StOverlapMetaProto::HIGH) {
    PiecewiseLinearFunction<double> large_vehicle_reaction_time_plf(
        {-1.0, -0.1, 0.0, 0.3}, {0.3, 0.5, 0.5, 1.2});
    agent_reaction_time_plf = std::move(large_vehicle_reaction_time_plf);
  } else {
    switch (overlap_meta.priority()) {
      case StOverlapMetaProto::HIGH: {
        PiecewiseLinearFunction<double> low_prio_reaction_time_plf(
            {-1.0, -0.1, 0.0, 0.3}, {0.0, 0.1, 0.1, 0.5});
        agent_reaction_time_plf = std::move(low_prio_reaction_time_plf);
        break;
      }
      case StOverlapMetaProto::UNKNOWN_PRIORITY:
      case StOverlapMetaProto::EQUAL: {
        PiecewiseLinearFunction<double> equal_prio_reaction_time_plf(
            {-1.0, -0.1, 0.0, 0.3}, {0.0, 0.1, 0.3, 0.7});
        agent_reaction_time_plf = std::move(equal_prio_reaction_time_plf);
        break;
      }
      case StOverlapMetaProto::LOW: {
        PiecewiseLinearFunction<double> high_prio_reaction_time_plf(
            {-1.0, -0.1, 0.0, 0.3}, {0.0, 0.3, 0.5, 1.2});
        agent_reaction_time_plf = std::move(high_prio_reaction_time_plf);
        break;
      }
    }
  }

  if (overlap_meta.has_time_to_lc_complete()) {
    CHECK_EQ(overlap_meta.source(), StOverlapMetaProto::AV_CUTIN);
    const PiecewiseLinearFunction<double> lc_time_discount_plf({0.0, 2.0},
                                                               {0.0, 1.0});
    return agent_reaction_time_plf(spacetime_obj.planner_object().pose().a()) *
           lc_time_discount_plf(overlap_meta.time_to_lc_complete());
  }

  return agent_reaction_time_plf(spacetime_obj.planner_object().pose().a());
}

double CalculateAgentFollowDistance(
    const StOverlapMetaProto& overlap_meta,
    StBoundaryProto::ObjectType object_type, double overlap_av_theta,
    double overlap_agent_theta, double overlap_av_v,
    bool is_parallel_front_merging,
    bool geometry_theory_use_particular_follow_distance,
    double particular_agent_follow_distance) {
  constexpr double kAggresiveSafetyDistanceLv1 = 0.5;
  constexpr double kAggresiveSafetyDistanceLv2 = 1.5;
  constexpr double kAggresiveSafetyDistanceLv3 = 3.0;

  if (geometry_theory_use_particular_follow_distance) {
    return particular_agent_follow_distance;
  }

  if (overlap_meta.has_is_making_u_turn() && overlap_meta.is_making_u_turn()) {
    VLOG(2) << "Agent is crossed by agent making U-Turn, use aggressive safety "
               "distance lv1.";
    return kAggresiveSafetyDistanceLv1;
  }

  if (overlap_meta.has_is_crossing_straight_lane() &&
      overlap_meta.is_crossing_straight_lane()) {
    VLOG(2) << "AV is crossed by agent while going straight, use aggressive "
               "safety distance lv1.";
    return kAggresiveSafetyDistanceLv1;
  }

  if (is_parallel_front_merging) {
    VLOG(2) << "Agent is parallel merging in front, use aggressive safety "
               "distance lv2.";
    return kAggresiveSafetyDistanceLv2;
  }

  if (overlap_meta.has_is_merging_straight_lane() &&
      overlap_meta.is_merging_straight_lane()) {
    VLOG(2) << "AV is merging by agent while going straight, use aggressive "
               "safety distance lv2.";
    return kAggresiveSafetyDistanceLv2;
  }

  if ((overlap_meta.source() == StOverlapMetaProto::LANE_CROSS ||
       overlap_meta.source() == StOverlapMetaProto::LANE_MERGE) &&
      (overlap_meta.priority() == StOverlapMetaProto::HIGH ||
       overlap_meta.priority() == StOverlapMetaProto::EQUAL)) {
    VLOG(2) << "AV is crossing/merging by agent with high/equal priority, use "
               "aggressive safety distance lv3.";
    return kAggresiveSafetyDistanceLv3;
  }

  double agent_standoff_distance = 0.0;

  PiecewiseLinearFunction<double> agent_standoff_time_plf;
  if (object_type == StBoundaryProto::CYCLIST) {
    constexpr double kCyclistStandoffDistance = 2.0;
    const PiecewiseLinearFunction<double> cyclist_standoff_time_plf(
        {M_PI / 4.0, M_PI / 2.0, M_PI * 2.0 / 3.0, M_PI * 5.0 / 6.0, M_PI},
        {0.2, 0.4, 0.6, 0.8, 1.3});
    agent_standoff_distance = kCyclistStandoffDistance;
    agent_standoff_time_plf = cyclist_standoff_time_plf;
  } else {
    constexpr double kVehicleStandoffDistance = 3.0;
    const PiecewiseLinearFunction<double> vehicle_standoff_time_plf(
        {M_PI / 4.0, M_PI / 2.0, M_PI * 2.0 / 3.0, M_PI * 5.0 / 6.0, M_PI},
        {0.2, 0.5, 0.7, 1.0, 1.5});
    agent_standoff_distance = kVehicleStandoffDistance;
    agent_standoff_time_plf = vehicle_standoff_time_plf;
  }
  const double heading_diff =
      NormalizeAngle(overlap_av_theta - overlap_agent_theta);
  const double overlap_agent_signed_acc_v =
      overlap_av_v * fast_math::Cos(overlap_av_theta - overlap_agent_theta);
  constexpr double kMaxTemporalFollowDistance = 5.0;
  const double temporal_follow_distance =
      std::min(std::abs(overlap_agent_signed_acc_v) *
                   agent_standoff_time_plf(std::abs(heading_diff)),
               kMaxTemporalFollowDistance);

  return agent_standoff_distance + temporal_follow_distance;
}

std::optional<YieldingResult> CalculateYieldingResultIfAgentYieldAV(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeObjectTrajectory& spacetime_obj,
    const std::vector<OverlapInfo>& overlap_infos, const DiscretizedPath& path,
    const SpeedVector& speed_profile, const StOverlapMetaProto& overlap_meta,
    StBoundaryProto::ObjectType object_type, double agent_reaction_time,
    bool geometry_theory_use_particular_follow_distance,
    double particular_agent_follow_distance,
    double follow_distance_buffer = 0.0, bool enable_vlog = true) {
  const auto& overlap_info = overlap_infos.front();
  YieldingResult yielding_result;

  const int overlap_av_idx =
      (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
  CHECK_GT(path.size(), overlap_av_idx);
  const auto& overlap_av_path_point = path[overlap_av_idx];
  if (overlap_av_path_point.s() > speed_profile.back().s()) {
    yielding_result.already_overtake = true;
    VLOG_IF(2, enable_vlog)
        << "AV cannot reach overlap point with this speed profile.";
    return yielding_result;
  }
  const auto av_speed_point =
      speed_profile.EvaluateByS(overlap_av_path_point.s());
  CHECK(av_speed_point.has_value());
  const double overlap_av_v = av_speed_point->v();

  const int overlap_agent_idx = overlap_info.obj_idx;
  CHECK_GT(spacetime_obj.states().size(), overlap_agent_idx);
  const auto* overlap_agent_traj_point =
      spacetime_obj.states()[overlap_agent_idx].traj_point;

  double penetration_distance = 0.0;
  const auto& obj_contour = spacetime_obj.states()[overlap_agent_idx].contour;
  const auto& direction =
      -Vec2d::FastUnitFromAngle(overlap_agent_traj_point->theta());
  for (int i :
       {overlap_info.av_start_idx, overlap_av_idx, overlap_info.av_end_idx}) {
    const auto& av_point = path[i];
    const Box2d av_box = ComputeAvBoxWithBuffer(
        Vec2d(av_point.x(), av_point.y()), av_point.theta(), vehicle_geom,
        spacetime_obj.required_lateral_gap(),
        spacetime_obj.required_lateral_gap());
    double dist = 0.0;
    Polygon2d(av_box).GetPenetrationDistanceAlongDir(obj_contour, direction,
                                                     &dist);
    penetration_distance = std::max(penetration_distance, dist);
  }

  const double agent_overlap_s =
      overlap_agent_traj_point->s() - penetration_distance;
  const double overlap_agent_v = overlap_agent_traj_point->v();
  const double agent_v = spacetime_obj.planner_object().pose().v();
  const double agent_a = spacetime_obj.planner_object().pose().a();
  const double overlap_agent_acc_v = std::max(
      0.0, overlap_av_v * fast_math::Cos(overlap_av_path_point.theta() -
                                         overlap_agent_traj_point->theta()));

  if (agent_v < kEps) {
    VLOG_IF(2, enable_vlog) << "Agent pose is stationary.";
    return std::nullopt;
  }

  const bool is_parallel_merging = IsParallelMerging(overlap_meta);
  const bool is_parallel_front_merging =
      is_parallel_merging &&
      HasYieldingIntentionToFrontAv(
          path.front(), spacetime_obj.planner_object(), vehicle_geom,
          overlap_meta, speed_profile,
          spacetime_obj.states()[overlap_agent_idx].traj_point->t());
  yielding_result.is_parallel_front_merging = is_parallel_front_merging;

  const double agent_follow_distance = CalculateAgentFollowDistance(
      overlap_meta, object_type, overlap_av_path_point.theta(),
      overlap_agent_traj_point->theta(), overlap_av_v,
      is_parallel_front_merging, geometry_theory_use_particular_follow_distance,
      particular_agent_follow_distance);

  const auto it = std::lower_bound(
      spacetime_obj.states().begin(), spacetime_obj.states().end(),
      av_speed_point->t(),
      [](const auto& state, double t) { return state.traj_point->t() < t; });
  if (it != spacetime_obj.states().end()) {
    const double agent_pred_s_at_yielding_point = it->traj_point->s();
    const double agent_pred_v_at_yielding_point = it->traj_point->v();
    const double agent_yield_s =
        agent_overlap_s - agent_follow_distance - follow_distance_buffer;
    if (agent_pred_s_at_yielding_point <= agent_yield_s &&
        agent_pred_v_at_yielding_point <= overlap_agent_acc_v) {
      yielding_result.already_yield = true;
      VLOG_IF(2, enable_vlog)
          << "Current st traj already yield this speed profile, no need to "
             "modify.";
      return yielding_result;
    }
  }

  const double agent_start_yield_v =
      std::max(agent_v + agent_a * agent_reaction_time, 0.0);
  const double agent_reaction_s =
      0.5 * (agent_start_yield_v + agent_v) * agent_reaction_time;

  const double agent_decel_dist =
      agent_overlap_s - agent_reaction_s - agent_follow_distance;

  if (agent_decel_dist <= 0.0) {
    VLOG_IF(2, enable_vlog)
        << "agent_v: " << agent_v << ", agent_a: " << agent_a
        << ", overlap_av_v:" << overlap_av_v
        << ", overlap_agent_acc_v: " << overlap_agent_acc_v
        << ", overlap_av_s:" << overlap_av_path_point.s()
        << ", agent_overlap_s: " << agent_overlap_s
        << ", agent_reaction_s: " << agent_reaction_s
        << ", agent_follow_distance:" << agent_follow_distance
        << ", agent_decel_dist: " << agent_decel_dist;
    VLOG_IF(2, enable_vlog) << "Agent has no enough distance to brake.";
    return std::nullopt;
  }

  const bool is_lane_cross =
      (overlap_meta.source() == StOverlapMetaProto::LANE_CROSS);
  const bool no_need_decel =
      is_lane_cross || overlap_agent_acc_v >= agent_start_yield_v;
  const double agent_acc_decel =
      no_need_decel ? 0.0
                    : (Sqr(overlap_agent_acc_v) - Sqr(agent_start_yield_v)) /
                          agent_decel_dist * 0.5;

  const double agent_braking_to_stop_decel =
      -Sqr(agent_start_yield_v) / agent_decel_dist * 0.5;

  const double t_stop =
      -agent_start_yield_v / std::min(agent_braking_to_stop_decel, -kEps);

  const double t_decel_to_acc =
      no_need_decel
          ? 0.0
          : (overlap_agent_acc_v - agent_start_yield_v) / agent_acc_decel;

  const double object_yielding_time =
      CalcObjectYieldingTime(overlap_infos, overlap_meta, path, speed_profile);
  const double t_yield = object_yielding_time - agent_reaction_time;

  double agent_arrive_yielding_point_decel = 0.0;
  double agent_finish_yield_decel = 0.0;
  double agent_v_at_yielding_point = 0.0;
  double agent_arrive_yielding_point_time = 0.0;
  double agent_waiting_duration = 0.0;
  if (t_yield <= t_decel_to_acc) {
    agent_arrive_yielding_point_decel = agent_acc_decel;
    agent_finish_yield_decel = agent_acc_decel;
    agent_arrive_yielding_point_time = agent_reaction_time + t_decel_to_acc;
    agent_v_at_yielding_point = overlap_agent_acc_v;
    VLOG_IF(2, enable_vlog) << "[acc]:";

  } else if (t_yield > t_decel_to_acc && t_yield < t_stop) {
    agent_arrive_yielding_point_decel = std::min(
        2.0 * (agent_decel_dist - agent_start_yield_v * t_yield) / Sqr(t_yield),
        0.0);
    agent_finish_yield_decel = agent_arrive_yielding_point_decel;
    agent_arrive_yielding_point_time = agent_reaction_time + t_yield;
    agent_v_at_yielding_point =
        agent_start_yield_v + agent_arrive_yielding_point_decel * t_yield;
    VLOG_IF(2, enable_vlog) << "[slower acc]:";

  } else if (t_yield > t_decel_to_acc && t_yield < t_stop) {
  } else {
    agent_arrive_yielding_point_decel = agent_braking_to_stop_decel;
    agent_finish_yield_decel = 0.0;
    agent_v_at_yielding_point = 0.0;
    agent_arrive_yielding_point_time = agent_reaction_time + t_stop;
    agent_waiting_duration = t_yield - t_stop;
    VLOG_IF(2, enable_vlog) << "[stop to waiting]:";
  }
  const double agent_s_at_yielding_point =
      agent_overlap_s - agent_follow_distance;

  yielding_result.start_yield_time = agent_reaction_time;
  yielding_result.arrive_yielding_point_decel =
      agent_arrive_yielding_point_decel;
  yielding_result.arrive_yielding_point_time = agent_arrive_yielding_point_time;
  yielding_result.waiting_duration = agent_waiting_duration;
  yielding_result.finish_yield_time =
      agent_arrive_yielding_point_time + agent_waiting_duration;
  yielding_result.v_at_yielding_point = agent_v_at_yielding_point;
  yielding_result.s_at_yielding_point = agent_s_at_yielding_point;

  VLOG_IF(2, enable_vlog) << "agent_v: " << agent_v << ", agent_a: " << agent_a
                          << ", agent_start_yield_v: " << agent_start_yield_v
                          << ", overlap_av_v: " << overlap_av_v
                          << ", overlap_agent_v: " << overlap_agent_v
                          << ", overlap_agent_acc_v: " << overlap_agent_acc_v
                          << "\nagent_overlap_s:" << agent_overlap_s
                          << ", agent_reaction_s:" << agent_reaction_s
                          << ", agent_follow_distance: "
                          << agent_follow_distance << ", "
                          << "agent_decel_dist: " << agent_decel_dist

                          << "\nagent_acc_decel: " << agent_acc_decel
                          << ", agent_braking_to_stop_decel: "
                          << agent_braking_to_stop_decel;
  VLOG_IF(2, enable_vlog)
      << "t_yield: " << t_yield << ", t_decel_to_acc: " << t_decel_to_acc
      << ", t_stop: " << t_stop << "\nagent_arrive_yielding_point_decel: "
      << agent_arrive_yielding_point_decel
      << ", agent_finish_yield_decel: " << agent_finish_yield_decel
      << ", agent_v_at_yielding_point: " << agent_v_at_yielding_point
      << ", agent_arrive_yielding_point_time: "
      << agent_arrive_yielding_point_time
      << ", agent_waiting_duration: " << agent_waiting_duration;

  const auto av_ttc_info = GetAvOverlapTtcInformation(
      overlap_infos, overlap_meta, path, speed_profile);
  const auto object_ttc_info =
      GetObjectOverlapTtcInfo(overlap_info, spacetime_obj, std::nullopt);
  const bool is_av_reach_first =
      av_ttc_info.ttc_upper_limit < object_ttc_info.ttc_lower_limit ||
      (overlap_meta.priority() == StOverlapMetaProto::HIGH &&
       av_ttc_info.ttc_lower_limit < object_ttc_info.ttc_upper_limit);

  yielding_result.has_yield_intention =
      IsAvInObjectFov(path.front(), spacetime_obj.planner_object(),
                      vehicle_geom) &&
      (is_av_reach_first || is_parallel_merging);
  return yielding_result;
}

double CalculateAgentDecelDiscomfort(
    StOverlapMetaProto::OverlapPriority av_priority, double decel,
    bool is_parallel_front_merging, bool is_large_vehicle) {
  if (is_parallel_front_merging) {
    return kLowPrioDecelDiscomfortPlf(decel);
  }
  switch (av_priority) {
    case StOverlapMetaProto::HIGH: {
      return kLowPrioDecelDiscomfortPlf(decel);
    }
    case StOverlapMetaProto::UNKNOWN_PRIORITY:
    case StOverlapMetaProto::EQUAL: {
      return is_large_vehicle ? kLargeVehicleDecelDiscomfortPlf(decel)
                              : kEqualPrioDecelDiscomfortPlf(decel);
    }
    case StOverlapMetaProto::LOW: {
      return is_large_vehicle ? kLargeVehicleDecelDiscomfortPlf(decel)
                              : kHighPrioDecelDiscomfortPlf(decel);
    }
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

double CalculateAvDecelDiscomfort(
    StOverlapMetaProto::OverlapPriority av_priority, double decel) {
  switch (av_priority) {
    case StOverlapMetaProto::HIGH:
    case StOverlapMetaProto::UNKNOWN_PRIORITY:
    case StOverlapMetaProto::EQUAL: {
      return kAvEqualPrioDecelDiscomfortPlf(decel);
    }
    case StOverlapMetaProto::LOW: {
      return kAvLowPrioDecelDiscomfortPlf(decel);
    }
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

double CalculateAgentWaitingDiscomfort(const StOverlapMetaProto& overlap_meta,
                                       double waiting_duration) {
  if (waiting_duration <= 0.0) return 0.0;

  if (overlap_meta.priority() != StOverlapMetaProto::HIGH) return kInf;

  if ((overlap_meta.has_is_making_u_turn() &&
       overlap_meta.is_making_u_turn()) ||
      (overlap_meta.has_is_crossing_straight_lane() &&
       overlap_meta.is_crossing_straight_lane()) ||
      (overlap_meta.has_is_merging_straight_lane() &&
       overlap_meta.is_merging_straight_lane())) {
    VLOG(2) << "AV drives with high priority, suppose agent will wait longer.";
    return kAgentAggresiveWaitingDiscomfortPlf(waiting_duration);
  }

  return kAgentWaitingDiscomfortPlf(waiting_duration);
}

double CalculateAvWaitingDiscomfort(const StOverlapMetaProto& overlap_meta,
                                    double waiting_duration) {
  if (waiting_duration <= 0.0) return 0.0;

  switch (overlap_meta.priority()) {
    case StOverlapMetaProto::HIGH: {
      return kBillion;
    }
    case StOverlapMetaProto::UNKNOWN_PRIORITY:
    case StOverlapMetaProto::EQUAL: {
      return kAvWaitingDiscomfortPlf(waiting_duration);
    }
    case StOverlapMetaProto::LOW: {
      return kAvAggresiveWaitingDiscomfortPlf(waiting_duration);
    }
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::optional<double> CalculateAvYieldingCost(
    const SpacetimeObjectTrajectory& spacetime_obj,
    const OverlapInfo& overlap_info, const DiscretizedPath& path,
    const SpeedVector& speed_profile, const StBoundary& st_boundary,
    const StOverlapMetaProto& overlap_meta, const double av_decel_factor = 1.0,
    bool enable_first_pt_check = true, bool enable_last_pt_check = true,
    double av_follow_distance = 4.0) {
  const auto first_av_speed_point =
      speed_profile.EvaluateByTime(st_boundary.bottom_left_point().t());
  if (enable_first_pt_check) {
    if (!first_av_speed_point.has_value() ||
        st_boundary.bottom_left_point().s() <
            first_av_speed_point->s() + av_follow_distance) {
      return std::nullopt;
    }
  }

  const auto last_av_speed_point =
      speed_profile.EvaluateByTime(st_boundary.bottom_right_point().t());
  if (enable_last_pt_check) {
    if (!last_av_speed_point.has_value() ||
        st_boundary.bottom_right_point().s() <
            last_av_speed_point->s() + av_follow_distance) {
      return std::nullopt;
    }
  }
  const double av_start_v = speed_profile.front().v();
  const double av_end_s_first =
      st_boundary.bottom_left_point().s() - av_follow_distance;
  const double av_yield_time_first = st_boundary.bottom_left_point().t();
  const double av_end_s_last = last_av_speed_point->s();
  const double av_yield_time_last = st_boundary.bottom_right_point().t();

  const double t_stop_last = 2.0 * av_end_s_last / (av_start_v + kEps);

  double av_acc_last, t_wait_last;

  if (t_stop_last > av_yield_time_last) {
    av_acc_last = 2.0 * (av_end_s_last / Sqr(av_yield_time_last + kEps) -
                         av_start_v / (av_yield_time_last + kEps));
    t_wait_last = 0.0;
  } else {
    av_acc_last = -av_start_v / t_stop_last;
    t_wait_last = av_yield_time_last - t_stop_last;
  }

  const double s_first_with_av_acc_last =
      0.5 * av_acc_last * av_yield_time_first * av_yield_time_last +
      av_start_v * av_yield_time_first;
  if (s_first_with_av_acc_last > av_end_s_first) {
    const double t_stop_first = 2.0 * av_end_s_first / (av_start_v + kEps);
    double av_acc_first, t_wait_first;
    if (t_stop_first > av_yield_time_first) {
      av_acc_first = 2.0 * (av_end_s_first / Sqr(av_yield_time_first + kEps) -
                            av_start_v / (av_yield_time_first + kEps));
      t_wait_first = 0.0;
    } else {
      av_acc_first = -av_start_v / t_stop_first;
      t_wait_first = av_yield_time_first - t_stop_first;
    }
    return CalculateAvDecelDiscomfort(overlap_meta.priority(),
                                      av_acc_first * av_decel_factor) +
           CalculateAvWaitingDiscomfort(overlap_meta, t_wait_first);

  } else {
    return CalculateAvDecelDiscomfort(overlap_meta.priority(),
                                      av_acc_last * av_decel_factor) +
           CalculateAvWaitingDiscomfort(overlap_meta, t_wait_last);
  }
}

std::vector<InteractiveResult> CalculateInteractiveResults(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const SpeedVector& speed_profile,
    absl::Span<const StBoundaryWithDecision* const> interactive_st_boundaries,
    const absl::flat_hash_map<std::string, const StBoundaryWithDecision*>&
        protective_st_boundary_wd_map) {
  std::vector<InteractiveResult> interactive_results;
  interactive_results.reserve(interactive_st_boundaries.size());

  for (const auto* boundary_with_decision : interactive_st_boundaries) {
    bool debug = false;
#if DEBUG_GAME_THEORY
    if (boundary_with_decision->object_id().value() == "40774") {
      debug = true;
    }
#endif
    if (boundary_with_decision->raw_st_boundary()->is_protective()) {
      continue;
    }
    const auto traj_id = boundary_with_decision->traj_id();
    CHECK(traj_id.has_value());
    const auto* st_boundary = boundary_with_decision->st_boundary();
    CHECK(st_boundary->overlap_meta().has_value());
    const auto av_priority = st_boundary->overlap_meta()->priority();

    const bool is_unprotected_left_turn =
        st_boundary->overlap_meta()->has_is_crossing_straight_lane() &&
        st_boundary->overlap_meta()->is_crossing_straight_lane();
    VLOG(3) << "st-boundary: " << boundary_with_decision->id() << ", source: "
            << StOverlapMetaProto::OverlapSource_Name(
                   st_boundary->overlap_meta()->source())
            << ", av_priority: "
            << StOverlapMetaProto::OverlapPriority_Name(av_priority)
            << (is_unprotected_left_turn ? ", agent is unprotected left turn."
                                         : "");

    const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
    CHECK_NOTNULL(spacetime_obj);

    auto overlap_infos = st_boundary->overlap_infos();
    const auto* protective_st_boundary_ptr =
        FindOrNull(protective_st_boundary_wd_map, boundary_with_decision->id());
    if (nullptr != protective_st_boundary_ptr &&
        (*protective_st_boundary_ptr)->raw_st_boundary()->protection_type() ==
            StBoundaryProto::SMALL_ANGLE_CUT_IN) {
      const auto& protective_overlap_infos =
          (*protective_st_boundary_ptr)->st_boundary()->overlap_infos();
      overlap_infos.insert(overlap_infos.begin(),
                           protective_overlap_infos.begin(),
                           protective_overlap_infos.end());
    }
    const auto& first_overlap_info = overlap_infos.front();
    const double agent_reaction_time = CalculateAgentReactionTime(
        *st_boundary->overlap_meta(), *spacetime_obj);
    auto yielding_result = CalculateYieldingResultIfAgentYieldAV(
        vehicle_geom, *spacetime_obj, overlap_infos, path, speed_profile,
        *st_boundary->overlap_meta(), st_boundary->object_type(),
        std::max(agent_reaction_time,
                 boundary_with_decision->decision_param().agent_reaction_time),
        boundary_with_decision->decision_param()
            .geometry_theory_use_particular_follow_distance,
        boundary_with_decision->decision_param()
            .particular_agent_follow_distance,
        boundary_with_decision->decision_param()
            .interact_obj_follow_dist_buffer);
    const auto av_yielding_cost = CalculateAvYieldingCost(
        *spacetime_obj, first_overlap_info, path, speed_profile, *st_boundary,
        *st_boundary->overlap_meta(),
        boundary_with_decision->decision_param().interact_av_decel_factor,
        boundary_with_decision->decision_param()
            .enable_interact_first_point_decision,
        boundary_with_decision->decision_param()
            .enable_interact_last_point_decision,
        boundary_with_decision->decision_param()
            .geometry_theory_av_follow_distance);

    if (!yielding_result.has_value() || yielding_result->already_overtake) {
      if (debug) {
        std::cout << "yield result has value : " << yielding_result.has_value()
                  << "  already_overtake: " << yielding_result->already_overtake
                  << std::endl;
      }
      if (av_yielding_cost.has_value()) {
        yielding_result->already_overtake = true;
        interactive_results.push_back({.cost = *av_yielding_cost,
                                       .yielding_result = *yielding_result,
                                       .traj_id = *traj_id,
                                       .decision = StBoundaryProto::YIELD,
                                       .spacetime_obj = spacetime_obj,
                                       .st_boundary = st_boundary});
        if (debug) {
          std::cout << "1.av yield has value and decision yield" << std::endl;
        }
        continue;
      } else {
        interactive_results.push_back(
            {.cost = kInf,
             .traj_id = *traj_id,
             .decision = boundary_with_decision->decision_type(),
             .spacetime_obj = spacetime_obj,
             .st_boundary = st_boundary});
        if (debug) {
          std::cout << "2.av yield no value and decision dp : "
                    << boundary_with_decision->decision_type() << std::endl;
        }

        break;
      }
    }
    if (yielding_result->already_yield) {
      interactive_results.push_back({.cost = 0.0,
                                     .yielding_result = *yielding_result,
                                     .traj_id = *traj_id,
                                     .decision = StBoundaryProto::OVERTAKE,
                                     .spacetime_obj = spacetime_obj,
                                     .st_boundary = st_boundary});
      if (debug) {
        std::cout << "3.already_yield and decision overtake " << std::endl;
      }
      continue;
    }

    constexpr double kAvYieldingCoeff = 0.8;
    const double agent_decel_discomfort = CalculateAgentDecelDiscomfort(
        av_priority, yielding_result->arrive_yielding_point_decel,
        yielding_result->is_parallel_front_merging,
        spacetime_obj->planner_object().is_large_vehicle());
    const double agent_waiting_discomfort = CalculateAgentWaitingDiscomfort(
        *st_boundary->overlap_meta(), yielding_result->waiting_duration);

    if (av_yielding_cost.has_value()) {
      if (!yielding_result->is_parallel_front_merging &&
          kAvYieldingCoeff * (*av_yielding_cost) <
              (agent_decel_discomfort + agent_waiting_discomfort)) {
        yielding_result->already_overtake = true;
        interactive_results.push_back({.cost = *av_yielding_cost,
                                       .yielding_result = *yielding_result,
                                       .traj_id = *traj_id,
                                       .decision = StBoundaryProto::YIELD,
                                       .spacetime_obj = spacetime_obj,
                                       .st_boundary = st_boundary});
        if (debug) {
          std::cout << " kAvYieldingCoeff * (*av_yielding_cost): "
                    << kAvYieldingCoeff * (*av_yielding_cost)
                    << "   agent cost: "
                    << (agent_decel_discomfort + agent_waiting_discomfort)
                    << std::endl;
          std::cout << "4.yield by cost " << std::endl;
        }

        continue;
      } else {
        if (debug) {
          std::cout << " kAvYieldingCoeff * (*av_yielding_cost): "
                    << kAvYieldingCoeff * (*av_yielding_cost)
                    << "   agent cost: "
                    << (agent_decel_discomfort + agent_waiting_discomfort)
                    << std::endl;
        }
        VLOG(2) << "AV do not yield this agent because of larger cost";
      }
    } else {
      if (debug) {
        std::cout << "av no cost " << " obj_cost "
                  << agent_decel_discomfort + agent_waiting_discomfort
                  << std::endl;
      }
      VLOG(2) << "AV cannot yield this agent";
    }

    if (debug) {
      std::cout << " 5. decision overtake " << std::endl;
    }

    interactive_results.push_back(
        {.cost = agent_decel_discomfort + agent_waiting_discomfort,
         .yielding_result = *yielding_result,
         .traj_id = *traj_id,
         .decision = StBoundaryProto::OVERTAKE,
         .spacetime_obj = spacetime_obj,
         .st_boundary = st_boundary});
  }

  return interactive_results;
}

void SmoothAccelPointList(std::vector<AccelPoint>* accel_point_list) {
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
    const VehicleGeometryParamsProto& vehicle_geom, const DiscretizedPath& path,
    const SpeedVector& speed_profile,
    absl::Span<const InteractiveResult> interactive_results,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
        modification_info) {
  for (const auto& interactive_result : interactive_results) {
    if (!interactive_result.yielding_result.has_value()) {
      continue;
    }
    if (interactive_result.yielding_result->already_overtake) {
      continue;
    }
    if (interactive_result.st_boundary->is_protective()) {
      continue;
    }
    auto accel_point_list = CalculateAgentAccelPointList(
        vehicle_geom, *interactive_result.spacetime_obj,
        interactive_result.st_boundary->overlap_infos().back(), path,
        speed_profile, *interactive_result.yielding_result);

    SmoothAccelPointList(&accel_point_list);

    CHECK(!accel_point_list.empty());
    (*modification_info)[interactive_result.traj_id] = {
        .modifier_type = StBoundaryModifierProto::LON_INTERACTIVE,
        .decision = interactive_result.decision,
        .is_decision_changed = true,
        .accel_point_list = std::move(accel_point_list)};
  }
}

absl::StatusOr<SpeedVector> SelectBestInteractivePreliminarySpeed(
    const SpeedPlanningParamsProto::SamplingDpSpeedParamsProto& dp_params,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    absl::Span<const PreliminarySpeedWithCost> candidate_speed_profiles,
    absl::Span<const StBoundaryWithDecision* const> interactive_st_boundaries,
    const absl::flat_hash_map<std::string, const StBoundaryWithDecision*>&
        protective_st_boundary_wd_map,
    std::vector<InteractiveResult>* final_interactive_results) {
  double min_cost = kInf;
  std::optional<int> min_cost_index = std::nullopt;
  for (int i = 0; i < candidate_speed_profiles.size(); ++i) {
#if DEBUG_GAME_THEORY
    std::cout << "-----------------" << std::endl;
    std::cout << "candidate speed profile : " << i << std::endl;
#endif
    VLOG(3) << "------------------ " << i << "-th" << " ------------------";
    auto interactive_results = CalculateInteractiveResults(
        vehicle_geom, st_traj_mgr, path,
        candidate_speed_profiles[i].preliminary_speed,
        interactive_st_boundaries, protective_st_boundary_wd_map);

    double cost = candidate_speed_profiles[i].cost;

    constexpr double kInteractiveCompensationDistance = 40.0;
    bool decision_changed = false;
    for (const auto& result : interactive_results) {
      if (!result.yielding_result->already_overtake &&
          !result.yielding_result->already_yield &&
          !result.yielding_result->has_yield_intention) {
        decision_changed = true;
      }
      cost += result.cost;
    }
    if (decision_changed) {
      const int dimension_time =
          CeilToInteger(
              candidate_speed_profiles[i].preliminary_speed.back().t() /
              dp_params.unit_t()) +
          1;
      cost += kInteractiveCompensationDistance *
              static_cast<double>(dimension_time) *
              dp_params.spatial_potential_weight();
    }
    cost +=
        CalculateConsistencyCost(interactive_st_boundaries,
                                 candidate_speed_profiles[i].preliminary_speed);

    if (std::isinf(cost)) {
      continue;
    }

    if (cost < min_cost) {
      min_cost = cost;
      min_cost_index = i;
      *final_interactive_results = std::move(interactive_results);
    }
  }

  VLOG(2) << "-------------------------------------------";

  if (!min_cost_index.has_value()) {
    return absl::NotFoundError("No interactive speed profile found.");
  }
#if DEBUG_GAME_THEORY
  std::cout << "select speed profile : " << min_cost_index.value() << std::endl;
#endif
  return candidate_speed_profiles[*min_cost_index].preliminary_speed;
}

absl::Status GetPreliminarySpeedBySamplingDp(
    const SpeedLimitProvider& speed_limit_provider, double path_length,
    double current_v, double current_a, double max_acceleration,
    double max_deceleration,
    const SpeedPlanningParamsProto& speed_planning_params, double speed_cap,
    int traj_steps, PreliminarySpeedWithCost* preliminary_speed_with_cost,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    SamplingDpDebugProto* sampling_dp_debug, WorkerThreadManager* thread_pool,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  CHECK_NOTNULL(preliminary_speed_with_cost);
  CHECK_NOTNULL(st_boundaries_with_decision);

  std::vector<StBoundaryWithDecision*> st_boundaries_wd_ptr;
  st_boundaries_wd_ptr.reserve(st_boundaries_with_decision->size());
  for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    st_boundaries_wd_ptr.push_back(&st_boundary_with_decision);
  }
  StGraphData st_graph_data(&speed_limit_provider, speed_cap, path_length,
                            traj_steps * kTrajectoryTimeStep);
  GriddedSvtGraph gridded_svt_graph(
      &st_graph_data, current_v, current_a, max_acceleration + kMaxAccMargin,
      max_deceleration + kMaxDecelMargin, &speed_planning_params, speed_cap,
      std::move(st_boundaries_wd_ptr), st_traj_mgr);
  RETURN_IF_ERROR(gridded_svt_graph.FindOptimalPreliminarySpeedWithCost(
      preliminary_speed_with_cost, sampling_dp_debug, thread_pool));

  return absl::OkStatus();
}

absl::Status GenerateSpeedProfileCandidateSet(
    const SpeedLimitProvider& speed_limit_provider, double path_length,
    double current_v, double current_a, double max_acceleration,
    double max_deceleration,
    const SpeedPlanningParamsProto& speed_planning_params, double speed_cap,
    int traj_steps,
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
    std::vector<StBoundaryWithDecision*>* st_boundaries_with_decision,
    SamplingDpDebugProto* sampling_dp_debug,
    InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
    WorkerThreadManager* thread_pool,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  CHECK_NOTNULL(candidate_speed_profiles);
  CHECK_NOTNULL(st_boundaries_with_decision);

  StGraphData st_graph_data(&speed_limit_provider, speed_cap, path_length,
                            traj_steps * kTrajectoryTimeStep);
  GriddedSvtGraph gridded_svt_graph(
      &st_graph_data, current_v, current_a, max_acceleration + kMaxAccMargin,
      max_deceleration + kMaxDecelMargin, &speed_planning_params, speed_cap,
      std::move(*st_boundaries_with_decision), st_traj_mgr);
  RETURN_IF_ERROR(gridded_svt_graph.GenerateSamplingDpSpeedProfileCandidateSet(
      candidate_speed_profiles, sampling_dp_debug, candidate_set_debug,
      thread_pool));

  gridded_svt_graph.SwapStBoundariesWithDecision(st_boundaries_with_decision);

  return absl::OkStatus();
}

void FilterCandidateSpeedProfiles(
    const DiscretizedPath& path,
    absl::Span<const StBoundaryWithDecision* const> interactive_st_boundaries,
    absl::Span<const StBoundaryWithDecision* const>
        non_interactive_st_boundaries,
    const SpeedVector& lower_bound_speed, double max_cost,
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles) {
  CHECK_GE(lower_bound_speed.size(), 2);

  std::vector<PreliminarySpeedWithCost> filtered_speed_profiles;
  filtered_speed_profiles.reserve(candidate_speed_profiles->size());

  double first_stopline_s = kInf;
  for (const auto* non_inter_st_boundary : non_interactive_st_boundaries) {
    const auto* st_boundary = non_inter_st_boundary->st_boundary();
    if (st_boundary->source_type() == StBoundarySourceTypeProto::VIRTUAL) {
      first_stopline_s = std::min(first_stopline_s, st_boundary->min_s());
    }
  }

  double min_interactive_s = kInf;
  for (const auto* interactive_st_boundary : interactive_st_boundaries) {
    const int av_start_idx = interactive_st_boundary->st_boundary()
                                 ->overlap_infos()
                                 .front()
                                 .av_start_idx;
    CHECK_GT(path.size(), av_start_idx)
        << interactive_st_boundary->st_boundary()->id();
    min_interactive_s = std::min(min_interactive_s, path[av_start_idx].s());
  }

  const double lower_bound_speed_t_step =
      lower_bound_speed[1].t() - lower_bound_speed[0].t();
  for (auto& speed_profile : *candidate_speed_profiles) {
    if (speed_profile.cost > max_cost) {
      continue;
    }
    const auto& preliminary_speed = speed_profile.preliminary_speed;
    CHECK_GE(preliminary_speed.size(), 2);

    if (preliminary_speed.back().s() > first_stopline_s) {
      continue;
    }

    if (preliminary_speed.back().s() < min_interactive_s) {
      continue;
    }

    const double t_step = preliminary_speed[1].t() - preliminary_speed[0].t();

    if (std::abs(t_step - lower_bound_speed_t_step) > kEps) {
      filtered_speed_profiles.push_back(std::move(speed_profile));
      continue;
    }
    const int size =
        std::min(preliminary_speed.size(), lower_bound_speed.size());
    for (int i = 0; i < size; ++i) {
      if (lower_bound_speed[i].s() < preliminary_speed[i].s()) {
        filtered_speed_profiles.push_back(std::move(speed_profile));
        break;
      }
    }
  }

  candidate_speed_profiles->swap(filtered_speed_profiles);
}

absl::flat_hash_set<std::string> PreFilterNonInteractiveStBoundries(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const PathPoint& plan_start_path_point) {
  absl::flat_hash_set<std::string> non_interactive_ids;

  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    VLOG(3) << "-- Pre-filter: " << boundary_with_decision.id() << " --";

    if (boundary_with_decision.raw_st_boundary()->is_protective()) {
      continue;
    }

    if (!boundary_with_decision.st_boundary()->overlap_meta().has_value()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "No overlap meta data.";
      continue;
    }
    const auto& overlap_meta =
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
      case StOverlapMetaProto::NON_MODIFIABLE:
      case StOverlapMetaProto::LON_LAT_MODIFIABLE: {
        non_interactive_ids.insert(boundary_with_decision.id());
        VLOG(2) << "Not considered modification type.";
        continue;
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

    if (boundary_with_decision.traj_id().has_value() &&
        processed_st_objects.find(*boundary_with_decision.traj_id()) !=
            processed_st_objects.end()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "Modified by others.";
      continue;
    }

    if (overlap_meta.source() == StOverlapMetaProto::AV_CUTIN &&
        boundary_with_decision.traj_id().has_value()) {
      const auto* spacetime_obj =
          st_traj_mgr.FindTrajectoryById(*boundary_with_decision.traj_id());
      const auto pos = spacetime_obj->pose().pos();
      const auto heading_perp =
          Vec2d::FastUnitFromAngle(spacetime_obj->pose().theta() - M_PI * 0.5);
      const HalfPlane agent_view(pos - heading_perp, pos + heading_perp);
      const auto av_box = ComputeAvBox(
          Vec2d(plan_start_path_point.x(), plan_start_path_point.y()),
          plan_start_path_point.theta(), vehicle_geom);
      bool is_av_inside_agent_view = false;
      for (const auto& corner : av_box.GetCornersCounterClockwise()) {
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

  return non_interactive_ids;
}

void PostFilterNonInteractiveStBoundries(
    const SpeedProfile& empty_road_speed_profile,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    absl::flat_hash_set<std::string>* non_interactive_ids) {
  constexpr double kConsiderAsFrontAgentTimeBuffer = 1.0;
  constexpr double kAVTimeLaterThres = 1.5;
  constexpr double kConsiderAsInteractiveTimeThreshold = 5.0;

  absl::flat_hash_map<std::string, const StBoundaryWithDecision*> traj_id_map;
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (non_interactive_ids->find(boundary_with_decision.id()) !=
        non_interactive_ids->end()) {
      continue;
    }

    if (boundary_with_decision.raw_st_boundary()->is_protective()) {
      continue;
    }

    VLOG(3) << "---- Post-filter: " << boundary_with_decision.id() << " ----";

    if (boundary_with_decision.decision_type() == StBoundaryProto::OVERTAKE) {
      non_interactive_ids->insert(boundary_with_decision.id());
      VLOG(2) << "Already LEAD.";
      continue;
    }

    if (boundary_with_decision.decision_param()
            .enable_add_collision_risk_cost_for_dp) {
      non_interactive_ids->insert(boundary_with_decision.id());
      VLOG(2) << "Can not overtake.";
      continue;
    }

    const double min_time_to_overlap = empty_road_speed_profile.GetTimeAtS(
        boundary_with_decision.st_boundary()->bottom_left_point().s());
    if (min_time_to_overlap > kConsiderAsInteractiveTimeThreshold) {
      non_interactive_ids->insert(boundary_with_decision.id());
      VLOG(2) << "Long time to overlap: " << min_time_to_overlap;
      continue;
    }

    if (boundary_with_decision.decision_type() == StBoundaryProto::YIELD ||
        boundary_with_decision.decision_type() == StBoundaryProto::FOLLOW) {
      const double real_min_t =
          boundary_with_decision.raw_st_boundary()->min_t();
      const bool is_agent_near_overlap =
          real_min_t < kConsiderAsFrontAgentTimeBuffer;
      const bool is_av_need_longer_time_to_reach_overlap =
          min_time_to_overlap - real_min_t > kAVTimeLaterThres;
      if (is_agent_near_overlap && is_av_need_longer_time_to_reach_overlap) {
        non_interactive_ids->insert(boundary_with_decision.id());
        VLOG(2) << "Is front agent, real_min_t:" << real_min_t
                << ", min_time_to_overlap" << min_time_to_overlap;
        continue;
      }
    }

    if (boundary_with_decision.traj_id().has_value()) {
      const auto traj_id = *boundary_with_decision.traj_id();
      const auto it = traj_id_map.find(traj_id);
      if (it != traj_id_map.end() &&
          non_interactive_ids->find(it->second->id()) ==
              non_interactive_ids->end()) {
        if (boundary_with_decision.st_boundary()->min_t() >
            it->second->st_boundary()->min_t()) {
          non_interactive_ids->insert(boundary_with_decision.id());
          VLOG(2) << "Less influential multi st-boundary: "
                  << boundary_with_decision.id();
          continue;
        } else {
          non_interactive_ids->insert(it->second->id());
          VLOG(2) << "Less influential multi st-boundary: " << it->second->id();
        }
      }
      traj_id_map[traj_id] = &boundary_with_decision;
    }
  }
}

bool NeedModifyStBoundary(const SpacetimeObjectTrajectory& spacetime_obj,
                          const YieldingResult& yielding_result) {
  if (yielding_result.already_yield ||
      spacetime_obj.trajectory().points().size() < 2) {
    return false;
  }
  const double arrive_yielding_point_time =
      yielding_result.arrive_yielding_point_time;
  const double arrive_yielding_point_decel =
      yielding_result.arrive_yielding_point_decel;
  const auto& pred_traj_points = spacetime_obj.trajectory().points();
  const double t_step = pred_traj_points[1].t() - pred_traj_points[0].t();
  const int finish_yield_index =
      std::min(pred_traj_points.size() - 1,
               static_cast<size_t>(std::ceil(arrive_yielding_point_time /
                                             std::max(kEps, t_step))));
  const double agent_v = spacetime_obj.planner_object().pose().v();
  const double finish_yield_s =
      agent_v * arrive_yielding_point_time +
      0.5 * arrive_yielding_point_decel * Sqr(arrive_yielding_point_time);
  if (finish_yield_index + 1 > spacetime_obj.states().size()) {
    return false;
  }
  if (finish_yield_s >
      spacetime_obj.states()[finish_yield_index].traj_point->s()) {
    return false;
  }
  return true;
}

void GenerateOvertakeStBoundaryModificationInfo(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const SpeedVector& speed_profile,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const absl::flat_hash_map<std::string, const StBoundaryWithDecision*>&
        protective_st_boundary_wd_map,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
        modification_info,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs) {
  CHECK_NOTNULL(modification_info);

  const bool enable_vlog = false;

  absl::flat_hash_map<std::string, std::pair<std::string, double>> min_t_map;

  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (boundary_with_decision.raw_st_boundary()->is_protective() &&
        boundary_with_decision.raw_st_boundary()->protection_type() !=
            StBoundaryProto::LANE_CHANGE_GAP) {
      continue;
    }
    const auto traj_id = boundary_with_decision.traj_id();
    if (!traj_id.has_value()) continue;
    const auto it = min_t_map.find(*traj_id);
    if (it == min_t_map.end() ||
        boundary_with_decision.st_boundary()->min_t() < it->second.second) {
      min_t_map[*traj_id] =
          std::make_pair(boundary_with_decision.id(),
                         boundary_with_decision.st_boundary()->min_t());
    }
  }

  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (boundary_with_decision.decision_type() != StBoundaryProto::OVERTAKE) {
      continue;
    }

    if (boundary_with_decision.raw_st_boundary()->is_protective() &&
        boundary_with_decision.raw_st_boundary()->protection_type() !=
            StBoundaryProto::LANE_CHANGE_GAP) {
      continue;
    }

    const auto traj_id = boundary_with_decision.traj_id();
    if (!traj_id.has_value()) continue;

    if (min_t_map[*traj_id].first != boundary_with_decision.id()) continue;

    if (processed_st_objects.find(*traj_id) != processed_st_objects.end()) {
      continue;
    }

    VLOG_IF(2, enable_vlog) << "traj_id: " << *traj_id;
    const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
    const auto* st_boundary = boundary_with_decision.st_boundary();
    auto overlap_infos = st_boundary->overlap_infos();
    const auto* protective_st_boundary_ptr =
        FindOrNull(protective_st_boundary_wd_map, boundary_with_decision.id());
    if (nullptr != protective_st_boundary_ptr &&
        (*protective_st_boundary_ptr)->raw_st_boundary()->protection_type() ==
            StBoundaryProto::SMALL_ANGLE_CUT_IN) {
      const auto& protective_overlap_infos =
          (*protective_st_boundary_ptr)->st_boundary()->overlap_infos();
      overlap_infos.insert(overlap_infos.begin(),
                           protective_overlap_infos.begin(),
                           protective_overlap_infos.end());
    }
    if (!st_boundary->overlap_meta().has_value()) {
      continue;
    }
    const double agent_reaction_time = CalculateAgentReactionTime(
        *st_boundary->overlap_meta(), *spacetime_obj);

    const auto yielding_result = CalculateYieldingResultIfAgentYieldAV(
        vehicle_geom, *spacetime_obj, overlap_infos, path, speed_profile,
        *st_boundary->overlap_meta(), st_boundary->object_type(),
        agent_reaction_time, false, 0.0, enable_vlog);
    if (!yielding_result.has_value()) {
      continue;
    }

    if (!NeedModifyStBoundary(*spacetime_obj, *yielding_result)) {
      continue;
    }

    auto accel_point_list = CalculateAgentAccelPointList(
        vehicle_geom, *spacetime_obj, overlap_infos.back(), path, speed_profile,
        *yielding_result);

    const auto modifier_type =
        boundary_with_decision.decision_reason() == StBoundaryProto::PRE_DECIDER
            ? StBoundaryModifierProto::PRE_DECISION
            : StBoundaryModifierProto::LEADING;
    CHECK(!accel_point_list.empty());
    (*modification_info)[*traj_id] = {
        .modifier_type = modifier_type,
        .decision = boundary_with_decision.decision_type(),
        .is_decision_changed = false,
        .accel_point_list = std::move(accel_point_list)};
  }
}

void PostProcessOvertakeGapObsStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    const LaneChangeStateProto lane_change_state) {
  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         const StBoundaryProto::DecisionReason& decision_reason,
         StBoundaryWithDecision& st_boundary_wd) {
        st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd.set_decision_reason(decision_reason);
        st_boundary_wd.set_ignore_reason(ignore_reason);
        st_boundary_wd.set_decision_info(decision_info);
      };

  bool have_overtake_gap_obs = false;
  std::string overtake_gap_id = "-1";
  StBoundaryWithDecision* overtake_gap_stw;
  for (auto& boundary_with_decision : *st_boundaries_with_decision) {
    if (boundary_with_decision.decision_type() != StBoundaryProto::OVERTAKE) {
      continue;
    }
    if ((boundary_with_decision.raw_st_boundary()->is_protective() &&
         boundary_with_decision.raw_st_boundary()->protection_type() ==
             StBoundaryProto::LANE_CHANGE_GAP) ||
        (lane_change_state.stage() != LaneChangeStage::LCS_NONE)) {
      have_overtake_gap_obs = true;
      overtake_gap_id = boundary_with_decision.object_id().value();
      overtake_gap_stw = &boundary_with_decision;
      break;
    }
  }
  if (!have_overtake_gap_obs || overtake_gap_id == "-1") {
    return;
  }

  double nearest_stationary_s = std::numeric_limits<double>::max();
  const std::string* nearest_stationary_id_ptr = nullptr;
  for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (st_boundary->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary_with_decision.decision_type() != StBoundaryProto::FOLLOW) {
      continue;
    }
    if (!st_boundary->is_stationary()) continue;
    if (st_boundary->min_s() < 0.01) continue;
    if (const double follow_s =
            st_boundary->min_s() -
            st_boundary_with_decision.follow_standstill_distance();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id_ptr = &st_boundary->id();
    }
  }

  const auto& st_boundary = overtake_gap_stw->raw_st_boundary();
  if (overtake_gap_stw->raw_st_boundary()->upper_right_point().s() +
          overtake_gap_stw->lead_standstill_distance() >
      nearest_stationary_s) {
    make_ignore_decision(
        "ignore overtake back gap obs when there is a stationary obs ahead",
        StBoundaryProto::BACK_CUT_IN,
        StBoundaryProto::POSTPROCESS_GAP_LON_DECISION, *overtake_gap_stw);
    return;
  }

  for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (st_boundary->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary_with_decision.decision_type() != StBoundaryProto::FOLLOW) {
      continue;
    }
    if (st_boundary->is_stationary()) continue;
    double min_t = overtake_gap_stw->raw_st_boundary()->min_t();
    if (!InRange(min_t, st_boundary->min_t(), st_boundary->max_t())) {
      continue;
    }
    const auto follow_s_range = st_boundary->GetBoundarySRange(min_t);
    const auto gap_s_range =
        overtake_gap_stw->raw_st_boundary()->GetBoundarySRange(min_t);
    if (gap_s_range->first + overtake_gap_stw->lead_standstill_distance() >
        follow_s_range->second) {
      make_ignore_decision(
          "ignore overtake back gap obs when there is a stationary obs ahead",
          StBoundaryProto::BACK_CUT_IN,
          StBoundaryProto::POSTPROCESS_GAP_LON_DECISION, *overtake_gap_stw);
      return;
    }

    double max_t = overtake_gap_stw->raw_st_boundary()->max_t();
    if (!InRange(max_t, st_boundary->min_t(), st_boundary->max_t())) {
      continue;
    }
    const auto follow_s_range_max_t = st_boundary->GetBoundarySRange(max_t);
    const auto gap_s_range_max_t =
        overtake_gap_stw->raw_st_boundary()->GetBoundarySRange(max_t);
    if (gap_s_range_max_t->first +
            overtake_gap_stw->lead_standstill_distance() >
        follow_s_range_max_t->second) {
      make_ignore_decision(
          "ignore overtake back gap obs when there is a stationary obs ahead",
          StBoundaryProto::BACK_CUT_IN,
          StBoundaryProto::POSTPROCESS_GAP_LON_DECISION, *overtake_gap_stw);
      return;
    }
  }
}

absl::Status GenerateInteractiveSpeedProfile(
    std::string_view base_name, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& path, double current_v, double current_a,
    const SpeedPlanningParamsProto& speed_planning_params, double speed_cap,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const absl::flat_hash_map<std::string, const StBoundaryWithDecision*>&
        protective_st_boundary_wd_map,
    int traj_steps, SpeedLimitProvider* speed_limit_provider,
    SpeedVector* preliminary_speed,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
        modification_info,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    WorkerThreadManager* thread_pool) {
  CHECK_NOTNULL(preliminary_speed);
  CHECK_NOTNULL(modification_info);
  CHECK_NOTNULL(st_boundaries_with_decision);
  CHECK_NOTNULL(interactive_speed_debug);

  VLOG(2) << "-----------interactive-decision-----------";

  Timer start_time;

  absl::flat_hash_set<std::string> non_interactive_ids;
  if (speed_planning_params.enable_interactive_speed_decision()) {
    non_interactive_ids = PreFilterNonInteractiveStBoundries(
        vehicle_geom, st_traj_mgr, *st_boundaries_with_decision,
        processed_st_objects, path.front());
  }

  PreliminarySpeedWithCost non_interactive_preliminary_speed;
  RETURN_IF_ERROR(GetPreliminarySpeedBySamplingDp(
      *speed_limit_provider, path.length(), current_v, current_a,
      spacetime_constraint_params.max_acceleration(),
      spacetime_constraint_params.max_deceleration(), speed_planning_params,
      speed_cap, traj_steps, &non_interactive_preliminary_speed,
      st_boundaries_with_decision,
      interactive_speed_debug->mutable_non_interactive_sampling_dp(),
      thread_pool, st_traj_mgr));
  non_interactive_preliminary_speed.preliminary_speed.ToProto(
      interactive_speed_debug->mutable_non_interactive_speed_profile());

  if (!speed_planning_params.enable_interactive_speed_decision()) {
    *preliminary_speed = non_interactive_preliminary_speed.preliminary_speed;
    return absl::OkStatus();
  }

  auto speed_limit = MakePedestrainPreBrakeDecision(
      speed_planning_params.pre_brake_decision_params(), st_traj_mgr, path,
      current_v, Mph2Mps(spacetime_constraint_params.default_speed_limit()),
      speed_limit_provider->time_step(), traj_steps,
      st_boundaries_with_decision);

  if (speed_limit.has_value()) {
    speed_limit_provider->AddVtSpeedLimit(
        SpeedLimitTypeProto::UNCERTAIN_PEDESTRAIN, *speed_limit);
  }

  const auto empty_road_speed_profile = CreateEmptyRoadSpeedProfile(
      spacetime_constraint_params, path,
      speed_limit_provider->GetCombinationStaticSpeedLimit(), current_v,
      traj_steps);
  PostFilterNonInteractiveStBoundries(empty_road_speed_profile,
                                      *st_boundaries_with_decision,
                                      &non_interactive_ids);

  for (const auto& [id, protective_st_boundary] :
       protective_st_boundary_wd_map) {
    if (ContainsKey(non_interactive_ids, id)) {
      non_interactive_ids.insert(protective_st_boundary->id());
    }
  }

  if (non_interactive_ids.size() == st_boundaries_with_decision->size()) {
    *preliminary_speed = non_interactive_preliminary_speed.preliminary_speed;
    VLOG(2) << "No interactive st boundary found.";
    return absl::OkStatus();
  }

  std::vector<StBoundaryWithDecision*> non_interactive_st_boundaries;
  non_interactive_st_boundaries.reserve(non_interactive_ids.size());
  std::vector<StBoundaryWithDecision*> interactive_st_boundaries;
  interactive_st_boundaries.reserve(st_boundaries_with_decision->size() -
                                    non_interactive_ids.size());
  for (auto& st_boundary : *st_boundaries_with_decision) {
    if (non_interactive_ids.find(st_boundary.id()) !=
        non_interactive_ids.end()) {
      non_interactive_st_boundaries.push_back(&st_boundary);
    } else {
      interactive_st_boundaries.push_back(&st_boundary);
    }
  }

  std::vector<PreliminarySpeedWithCost> candidate_speed_profiles;
  RETURN_IF_ERROR(GenerateSpeedProfileCandidateSet(
      *speed_limit_provider, path.length(), current_v, current_a,
      spacetime_constraint_params.max_acceleration(),
      spacetime_constraint_params.max_deceleration(), speed_planning_params,
      speed_cap, traj_steps, &candidate_speed_profiles,
      &non_interactive_st_boundaries,
      interactive_speed_debug->mutable_interactive_sampling_dp(),
      interactive_speed_debug->mutable_candidate_set(), thread_pool,
      st_traj_mgr));

  FilterCandidateSpeedProfiles(
      path, interactive_st_boundaries, non_interactive_st_boundaries,
      non_interactive_preliminary_speed.preliminary_speed,
      non_interactive_preliminary_speed.cost, &candidate_speed_profiles);

  if (candidate_speed_profiles.empty()) {
    *preliminary_speed = non_interactive_preliminary_speed.preliminary_speed;
    VLOG(2) << "No candidate speed profile found.";
    return absl::OkStatus();
  }

  candidate_speed_profiles.push_back(non_interactive_preliminary_speed);

  std::vector<InteractiveResult> final_interactive_results;

  auto result = SelectBestInteractivePreliminarySpeed(
      speed_planning_params.sampling_dp_speed_params(), vehicle_geom,
      st_traj_mgr, path, candidate_speed_profiles, interactive_st_boundaries,
      protective_st_boundary_wd_map, &final_interactive_results);

  if (result.ok()) {
    *preliminary_speed = *result;
    GenerateInteractiveStBoundaryModificationInfo(
        vehicle_geom, path, *preliminary_speed, final_interactive_results,
        modification_info);
  } else {
    *preliminary_speed = non_interactive_preliminary_speed.preliminary_speed;
    VLOG(2) << "Interactive speed decision failed: "
            << result.status().message();

    if (FLAGS_enable_interactive_speed_decision_draw_st_traj) {
      for (const auto* boundary_with_decision : interactive_st_boundaries) {
        const auto traj_id = boundary_with_decision->traj_id();
        CHECK(traj_id.has_value());
        const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
      }
    }
  }

  const auto interactive_speed_decision_time = start_time.TimeNs() / 1e6;
  constexpr double kInteractiveSpeedDecisionTimeoutThres = 15.0;
  if (interactive_speed_decision_time > kInteractiveSpeedDecisionTimeoutThres) {
  }

  return absl::OkStatus();
}

}  // namespace

#define DEBUG_PEDESTRIAN (0)

void PostProcessForPedestrian(StBoundaryWithDecision& st_boundary_wd,
                              const SpacetimeTrajectoryManager& st_traj_mgr,
                              double current_v) {
  constexpr double kAvMinVel = Kph2Mps(5.0);
  constexpr double kSafeDistHighSpeed = 0.0;

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         const StBoundaryProto::DecisionReason& decision_reason,
         StBoundaryWithDecision& st_boundary_wd) {
        st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd.set_decision_reason(decision_reason);
        st_boundary_wd.set_ignore_reason(ignore_reason);
        st_boundary_wd.set_decision_info(decision_info);
      };

  if (StBoundaryProto::PEDESTRIAN !=
      st_boundary_wd.st_boundary()->object_type()) {
    return;
  }
  if (st_boundary_wd.decision_type() != StBoundaryProto::YIELD) return;
  if (current_v <= kAvMinVel) return;

  if (st_boundary_wd.raw_st_boundary()->is_protective()) return;
  const auto traj_id = st_boundary_wd.traj_id();
  CHECK(traj_id.has_value());
  const auto* st_boundary = st_boundary_wd.st_boundary();

  const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
  CHECK_NOTNULL(spacetime_obj);

  const int overlap_agent_idx =
      st_boundary_wd.st_boundary()->overlap_infos().front().obj_idx;
  CHECK_GT(spacetime_obj->states().size(), overlap_agent_idx);
  const auto* overlap_agent_traj_point =
      spacetime_obj->states()[overlap_agent_idx].traj_point;

  const double agent_v = spacetime_obj->planner_object().pose().v();
  double agent_t = overlap_agent_traj_point->s() / (agent_v + kEps);

  constexpr double kHighVelAvMinAcc = -3.0;
  constexpr double kLowerVelAvMinAcc = -4.0;
  double high_speed_reaction_time = 0.5;
  double low_speed_reaction_time = 0.0;
  double av_start_v = current_v;
  double av_dist_to_collision = st_boundary->bottom_left_point().s();
  double av_brake_dist, reaction_time, kAvMinAcc;
  double safe_dist;

  if (av_start_v >= Kph2Mps(10.0)) {
    reaction_time = high_speed_reaction_time;
    kAvMinAcc = kHighVelAvMinAcc;
    safe_dist = kSafeDistHighSpeed;
  } else {
    reaction_time = low_speed_reaction_time;
    kAvMinAcc = kLowerVelAvMinAcc;
    safe_dist = 0.0;
  }
  agent_t = std::max(agent_t, reaction_time);

  double av_stop_t = -av_start_v / kAvMinAcc;

  if (agent_t < av_stop_t) {
    av_brake_dist =
        av_start_v * agent_t + 0.5 * kAvMinAcc * Sqr(agent_t - reaction_time);
  } else {
    av_brake_dist = -Sqr(av_start_v) / (2 * kAvMinAcc);
  }
  av_brake_dist += safe_dist;

#if DEBUG_PEDESTRIAN
  if (st_boundary_wd.object_id().has_value()) {
    std::cout << "-----------------" << std::endl;
    std::cout << "pedestrian post_process : " << std::endl;
    std::cout << "obj_id " << st_boundary_wd.object_id().value()
              << " av_start_v " << av_start_v << " av_dist_to_collision "
              << av_dist_to_collision << " av_brake_dist " << av_brake_dist
              << " av_stop_t " << av_stop_t << " agent_t " << agent_t
              << " agent_s " << overlap_agent_traj_point->s() << " agent_v "
              << agent_v << std::endl;
  }
#endif
  if (av_brake_dist >= av_dist_to_collision) {
    make_ignore_decision("ignore pedestrian when av can not brake",
                         StBoundaryProto::POSTPROCESS_PEDESTRIAN,
                         StBoundaryProto::POSTPROCESS_PEDESTRIAN_LON_DECISION,
                         st_boundary_wd);
#if DEBUG_PEDESTRIAN
    std::cout << " ignore pedestrian when av can not brake " << std::endl;
#endif
  }
}

#define DEBUG_ST_SCENE (0)

void PostProcessForStraightTurnRightScene(
    StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeTrajectoryManager& st_traj_mgr, double current_v,
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path) {
  constexpr double kAvMinVel = Kph2Mps(10.0);
  constexpr double kMaxCurvature = 0.002;
  constexpr double kConsiderMaxPathLength = 30.0;

  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         const StBoundaryProto::DecisionReason& decision_reason,
         StBoundaryWithDecision& st_boundary_wd) {
        st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd.set_decision_reason(decision_reason);
        st_boundary_wd.set_ignore_reason(ignore_reason);
        st_boundary_wd.set_decision_info(decision_info);
      };

  if (StBoundaryProto::VEHICLE != st_boundary_wd.st_boundary()->object_type()) {
    return;
  }

  double average_curvature = 0.0;
  int32_t point_nums = 0;

  for (const auto& point : path) {
    average_curvature += std::abs(point.kappa());
    point_nums++;
    if (point.s() > kConsiderMaxPathLength) {
      break;
    }
  }
  if (0 != point_nums) {
    average_curvature /= point_nums;
  }

#if DEBUG_ST_SCENE
  std::cout << " average_curvature " << average_curvature << std::endl;
#endif
  if (kMaxCurvature < average_curvature) return;

  if (st_boundary_wd.decision_type() != StBoundaryProto::YIELD) return;
  if (current_v <= kAvMinVel) return;
  if (st_boundary_wd.raw_st_boundary()->is_protective()) return;

  const auto traj_id = st_boundary_wd.traj_id();
  CHECK(traj_id.has_value());
  const auto* st_boundary = st_boundary_wd.st_boundary();

  const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
  CHECK_NOTNULL(spacetime_obj);

  if (st_boundary_wd.decision_param().enable_add_collision_risk_cost_for_dp) {
    double delta_dist = st_boundary_wd.obj_frenet_polygon().s_max -
                        vehicle_geom.front_edge_to_center();
    if (st_boundary_wd.object_id().has_value()) {
#if DEBUG_ST_SCENE
      std::cout << " postprocess_st_tr: obj_id "
                << st_boundary_wd.object_id().value() << " delta_dist "
                << delta_dist << " max_s "
                << st_boundary_wd.obj_frenet_polygon().s_max
                << " front_edge_to_center "
                << vehicle_geom.front_edge_to_center() << std::endl;
#endif
    }
    if (delta_dist <= 2.1 && std::fabs(st_boundary_wd.dl()) >= (0.001)) {
      make_ignore_decision(
          "ignore turn_right obj when ds < 0",
          StBoundaryProto::POSTPROCESS_STRAIGHT_TURN_RIGHT_SCENE,
          StBoundaryProto::POSTPROCESS_STRAIGHT_TURN_RIGHT_SCENE_LON_DECISION,
          st_boundary_wd);
    }
  }
}

#define DEBUG_NUDGE_IGNORE (0)

void PostProcessForNudgeScene(StBoundaryWithDecision& st_boundary_wd,
                              const NudgeObjectInfo* nudge_object_info,
                              const VehicleGeometryParamsProto& vehicle_geom) {
  const auto make_ignore_decision =
      [](const std::string& decision_info,
         const StBoundaryProto::IgnoreReason& ignore_reason,
         const StBoundaryProto::DecisionReason& decision_reason,
         StBoundaryWithDecision& st_boundary_wd) {
        st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd.set_decision_reason(decision_reason);
        st_boundary_wd.set_ignore_reason(ignore_reason);
        st_boundary_wd.set_decision_info(decision_info);
      };

  if (nullptr == nudge_object_info) return;

  if (StBoundaryProto::CYCLIST != st_boundary_wd.st_boundary()->object_type() &&
      StBoundaryProto::PEDESTRIAN !=
          st_boundary_wd.st_boundary()->object_type()) {
    return;
  }
  if (!st_boundary_wd.object_id().has_value()) return;
  if (nudge_object_info->id != st_boundary_wd.object_id().value()) return;

  if (st_boundary_wd.decision_type() != StBoundaryProto::YIELD) return;

  double delta_dist = st_boundary_wd.obj_frenet_polygon().s_max -
                      vehicle_geom.front_edge_to_center();
#if DEBUG_NUDGE_IGNORE
  std::cout << " postprocess_nudge: obj_id "
            << st_boundary_wd.object_id().value() << " delta_dist "
            << delta_dist << " max_s "
            << st_boundary_wd.obj_frenet_polygon().s_max
            << " front_edge_to_center " << vehicle_geom.front_edge_to_center()
            << " min_t " << st_boundary_wd.st_boundary()->min_t() << std::endl;
#endif
  if (st_boundary_wd.st_boundary()->min_t() >= 2.0) {
    make_ignore_decision("ignore nudge obj", StBoundaryProto::IGNORE_NUDGE_OBJ,
                         StBoundaryProto::IGNORE_NUDGE_OBJ_LON_DECISION,
                         st_boundary_wd);
  }

  return;
}

void PostProcessForLonDecision(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& st_traj_mgr, double current_v,
    const VehicleGeometryParamsProto& vehicle_geom, const DiscretizedPath& path,
    const NudgeObjectInfo* nudge_object_info) {
  for (auto& st_boundary_wd : *st_boundaries_with_decision) {
    PostProcessForPedestrian(st_boundary_wd, st_traj_mgr, current_v);
    PostProcessForStraightTurnRightScene(st_boundary_wd, st_traj_mgr, current_v,
                                         vehicle_geom, path);
    PostProcessForNudgeScene(st_boundary_wd, nudge_object_info, vehicle_geom);
  }
}

absl::Status MakeSpeedSearchAndInteractiveDecision(
    std::string_view base_name, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& path, double current_v, double current_a,
    const SpeedPlanningParamsProto& speed_planning_params, double speed_cap,
    int traj_steps, SpeedLimitProvider* speed_limit_provider,
    SpeedVector* preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    WorkerThreadManager* thread_pool,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_objs,
    const std::vector<VehicleShapeBasePtr>& av_shapes,
    const PlanPassage* plan_passage,
    const LaneChangeStateProto lane_change_state,
    const NudgeObjectInfo* nudge_object_info) {
  CHECK_NOTNULL(preliminary_speed);
  CHECK_NOTNULL(st_boundaries_with_decision);
  CHECK_NOTNULL(processed_st_objects);
  CHECK_NOTNULL(interactive_speed_debug);

  absl::flat_hash_map<std::string, StBoundaryModificationInfo>
      modification_info;

  absl::flat_hash_map<std::string, const StBoundaryWithDecision*>
      protective_st_boundary_wd_map;
  for (const auto& st_boundary_wd : *st_boundaries_with_decision) {
    if (!st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    const auto& protected_st_boundary_id =
        st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
    if (!protected_st_boundary_id.has_value()) continue;
    protective_st_boundary_wd_map.emplace(*protected_st_boundary_id,
                                          &st_boundary_wd);
  }

  RETURN_IF_ERROR(GenerateInteractiveSpeedProfile(
      base_name, vehicle_geom, spacetime_constraint_params, st_graph,
      st_traj_mgr, path, current_v, current_a, speed_planning_params, speed_cap,
      *processed_st_objects, protective_st_boundary_wd_map, traj_steps,
      speed_limit_provider, preliminary_speed, &modification_info,
      st_boundaries_with_decision, interactive_speed_debug, thread_pool));

  PostProcessOvertakeGapObsStBoundary(st_boundaries_with_decision,
                                      lane_change_state);

  GenerateOvertakeStBoundaryModificationInfo(
      vehicle_geom, st_traj_mgr, path, *preliminary_speed,
      *st_boundaries_with_decision, *processed_st_objects,
      protective_st_boundary_wd_map, &modification_info, leading_objs);

  const PostStboundaryModifierInput modifier_input{
      .st_graph = &st_graph,
      .st_traj_mgr = &st_traj_mgr,
      .modification_info_map = &modification_info};
  PostModifyStBoundaries(modifier_input, st_boundaries_with_decision,
                         processed_st_objects);

  PostProcessForLonDecision(st_boundaries_with_decision, st_traj_mgr, current_v,
                            vehicle_geom, path, nudge_object_info);

  RETURN_IF_ERROR(InteractiveAgentProcess(
      vehicle_geom, st_graph, st_traj_mgr, path, current_v, current_a,
      st_boundaries_with_decision, processed_st_objects, leading_objs,
      av_shapes, plan_passage, lane_change_state));

  return absl::OkStatus();
}
}  // namespace e2e_noa::planning
