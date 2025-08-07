#include "speed/dp_svt_cost.h"

#include <algorithm>
#include <optional>
#include <ostream>
#include <utility>

#include "gflags/gflags.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "speed/st_boundary.h"
#include "speed/svt_point.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "util/map_util.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace e2e_noa::planning {
namespace {
constexpr double kMaxSpeedLimitDiff = 10.0;
constexpr double kEps = 0.0001;
constexpr double kBillion = 1e9;

using DecisionType = StBoundaryProto::DecisionType;

DecisionType MakeStBoundaryDecision(double s, double s_lower, double s_upper,
                                    double follow_lead_ratio = 0.5) {
  if (s < s_lower + follow_lead_ratio * (s_upper - s_lower)) {
    return StBoundaryProto::YIELD;
  } else {
    return StBoundaryProto::OVERTAKE;
  }
}

#define DEBUG_DP_DECISION_COST (0)

double GetExternalPedestrianCostForLowerSpeed(
    double agent_dist_to_collision, double agent_v,
    double av_dist_to_leave_collision, double av_speed,
    e2e_noa::StBoundaryProto_ObjectType object_type,
    bool is_debug_decision_cost) {
  constexpr double kPedestrianFollowDist = 2.0;
  constexpr double kAvMaxVelForPedestrianScene = Kph2Mps(10.0);
  double pedestrian_risk_cost = 0;
  const PiecewiseLinearFunction<double> kPedestrianCollisionRiskPlf(
      {-10.0 - kEps, -10.0, -7.0, -5.5, 0.0},
      {kBillion, 100.0, 90.0, 80.0, 10.0});

  if (StBoundaryProto::PEDESTRIAN != object_type) return pedestrian_risk_cost;
  if (av_speed >= kAvMaxVelForPedestrianScene) return pedestrian_risk_cost;

  double agent_t =
      std::max(agent_dist_to_collision - kPedestrianFollowDist, 0.0) /
      (agent_v + kEps);
  double av_through_dist = av_speed * agent_t;

  double delta_dist = av_through_dist - av_dist_to_leave_collision;

  pedestrian_risk_cost = kPedestrianCollisionRiskPlf(delta_dist);

#if DEBUG_DP_DECISION_COST
  if (is_debug_decision_cost) {
    std::cout << "[ Pedestrian_external_cost ]:" << " agent_v " << agent_v
              << " agent_dist " << agent_dist_to_collision << " agent_t "
              << agent_t << " av_speed " << av_speed << std::endl;
    std::cout << "[ Pedestrian_external_cost ]:" << " av_through_dist "
              << av_through_dist << " av_dist_to_leave_collision "
              << av_dist_to_leave_collision << " delta_dist " << delta_dist
              << std::endl;
    std::cout << "[ Pedestrian_external_cost ]:" << " pedestrian_risk_cost "
              << pedestrian_risk_cost << std::endl;
  }
#endif

  return pedestrian_risk_cost;
}

double calc_yield_result(double ego_vel, double obj_t,
                         double ego_dist_to_collision) {
  double yield_acc = 0.0;

  if (obj_t <= (0.0 + kEps)) return yield_acc;
  if (ego_vel <= (0.0 + kEps)) return yield_acc;
  if (ego_dist_to_collision <= (0.0 + kEps)) return yield_acc;
  double ego_t = ego_dist_to_collision / (ego_vel + kEps);
  if ((ego_t - obj_t) >= 2.0) return yield_acc;
  yield_acc = (ego_dist_to_collision - ego_vel * obj_t) / (0.5 * Sqr(obj_t));

  return yield_acc;
}

double GetStBoundaryCost(
    DecisionType decision, double av_speed, const StBoundary& st_boundary,
    double s, double s_lower, double s_upper, double t,
    double follow_standstill_distance,
    const PiecewiseLinearFunction<double>& follow_distance_rel_speed_plf,
    const SpeedPlanningParamsProto& speed_planning_params,
    const SpeedPlanningParamsProto::SamplingDpSpeedParamsProto& params,
    bool enable_add_collision_risk_cost_for_dp,
    bool enable_left_turn_overtake_cost, double delta_t, double agent_vel,
    double agent_dist_to_collision, double ego_yield_acc, double obj_yield_acc,
    double av_dist_to_collision, bool is_debug_decision_cost) {
  double cost = 0.0;
  const PiecewiseLinearFunction<double> kCollisionRiskPlf(
      {-2.5, -1.0, 0.0, 5.0, 5.0 - kEps}, {0.0, 40.0, 80, 100.0, kBillion});
  const PiecewiseLinearFunction<double> kRationalityPlf(
      {-15.0 - kEps, -15.0, -5.0, -2.0, 0.0}, {kBillion, 100.0, 50, 20, 0.0});

  if (decision == StBoundaryProto::YIELD ||
      decision == StBoundaryProto::FOLLOW) {
    double follow_distance_s = 0.0;
    if (st_boundary.source_type() == StBoundarySourceTypeProto::VIRTUAL ||
        st_boundary.source_type() ==
            StBoundarySourceTypeProto::IMPASSABLE_BOUNDARY ||
        st_boundary.source_type() == StBoundarySourceTypeProto::PATH_BOUNDARY ||
        st_boundary.protection_type() ==
            StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT) {
      follow_distance_s = follow_standstill_distance;
    } else {
      const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
      CHECK(object_v.has_value());
      const double follow_time_headway =
          st_boundary.is_large_vehicle()
              ? speed_planning_params.large_vehicle_follow_time_headway()
              : speed_planning_params.follow_time_headway();
      follow_distance_s = follow_time_headway * std::max(0.0, *object_v) +
                          follow_standstill_distance;

      const double rel_speed_gain =
          follow_distance_rel_speed_plf(*object_v - av_speed);
      follow_distance_s = std::max(follow_distance_s * rel_speed_gain,
                                   follow_standstill_distance);
    }

    if (enable_add_collision_risk_cost_for_dp) {
      if (av_speed > Kph2Mps(15.0) && ego_yield_acc < 0 &&
          av_dist_to_collision <= 10.0 && agent_dist_to_collision <= 10.0) {
        cost = kRationalityPlf(ego_yield_acc - obj_yield_acc);
#if DEBUG_DP_DECISION_COST
        if (is_debug_decision_cost) {
          std::cout << "[ rationality_cost ]: cost " << cost << " delta_acc "
                    << ego_yield_acc - obj_yield_acc << std::endl;
        }
#endif
      }
    }

    if (s + follow_distance_s > s_lower) {
      const double s_diff = follow_distance_s - s_lower + s;
      cost +=
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else if (decision == StBoundaryProto::OVERTAKE) {
    const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
    CHECK(object_v.has_value());
    const double overtake_distance_s =
        st_boundary.protection_type() ==
                StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
            ? 0.0
            : speed_planning_params.lead_time_headway() *
                      std::max(*object_v, 0.0) +
                  speed_planning_params.lead_standstill_distance();

    if (enable_add_collision_risk_cost_for_dp) {
      cost = kCollisionRiskPlf(delta_t);
    }

    constexpr double kSpeedEps = 1e-3;
    const double av_reach_time =
        av_dist_to_collision / std::max(av_speed, kSpeedEps);
    const double agent_reach_time =
        agent_dist_to_collision / std::max(agent_vel, kSpeedEps);
    constexpr double kTimeDiffThreshold = 2.5;
    if (std::fabs(av_reach_time - agent_reach_time) < kTimeDiffThreshold &&
        enable_left_turn_overtake_cost) {
      cost = speed_planning_params.obj_scene_params().lt_overtake_cost();
    }

    double obj_length =
        st_boundary.obj_scenario_info().obj_sl_info.frenet_polygon.s_max -
        st_boundary.obj_scenario_info().obj_sl_info.frenet_polygon.s_min;

#if DEBUG_DP_DECISION_COST
    if (is_debug_decision_cost) {
      std::cout << "[ risk_cost ]: cost " << cost << " delta_t " << delta_t
                << " min_s " << st_boundary.min_s() << " max_s "
                << st_boundary.max_s() << " obj_length " << obj_length
                << std::endl;
    }
#endif

    cost += GetExternalPedestrianCostForLowerSpeed(
        agent_dist_to_collision, agent_vel, st_boundary.min_s() + obj_length,
        av_speed, st_boundary.object_type(), is_debug_decision_cost);

    if (s < s_upper + overtake_distance_s) {
      const double s_diff = overtake_distance_s + s_upper - s;
      cost +=
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else {
    LOG(FATAL) << "Bad unknown decision for st-boundary " << st_boundary.id()
               << " at s = " << s << " t = " << t;
  }
  return cost;
}

}  // namespace

DpSvtCost::DpSvtCost(const SpeedPlanningParamsProto* speed_planning_params,
                     double total_t, double total_s,
                     const std::vector<StBoundaryWithDecision*>*
                         sorted_st_boundaries_with_decision,
                     const SpacetimeTrajectoryManager& st_traj_mgr)
    : speed_planning_params_(CHECK_NOTNULL(speed_planning_params)),
      params_(&speed_planning_params->sampling_dp_speed_params()),
      sorted_st_boundaries_with_decision_(
          CHECK_NOTNULL(sorted_st_boundaries_with_decision)),
      unit_t_(params_->unit_t()),
      total_s_(total_s),
      st_traj_mgr_(st_traj_mgr) {
  int index = 0;
  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    boundary_map_[st_boundary_with_decision->id()] = index++;
  }
  const auto dimension_t = CeilToInteger(total_t / unit_t_) + 1;
  boundary_cost_.resize(sorted_st_boundaries_with_decision_->size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }

  for (const auto* st_boundary_wd : *sorted_st_boundaries_with_decision_) {
    if (st_boundary_wd->raw_st_boundary()->is_protective()) {
      continue;
    }
    original_st_boundary_wd_map_[st_boundary_wd->id()] = st_boundary_wd;
  }
}

std::vector<SvtGraphPoint::StBoundaryDecision>
DpSvtCost::GetStBoundaryDecisionsForInitPoint(
    const SvtGraphPoint& svt_graph_point) {
  std::vector<SvtGraphPoint::StBoundaryDecision> st_boundary_decisions;
  const double s = svt_graph_point.point().s();
  const double t = svt_graph_point.point().t();

  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    const auto& st_boundary = *st_boundary_with_decision->st_boundary();
    if (t < st_boundary.min_t() || t > st_boundary.max_t()) {
      continue;
    }
    if (st_boundary_with_decision->decision_type() == StBoundaryProto::IGNORE) {
      continue;
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    const int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      if (boundary_cost_[boundary_index][svt_graph_point.index_t()].first <
          0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        CHECK(s_range.has_value());
        boundary_cost_[boundary_index][svt_graph_point.index_t()] = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].first;
        s_lower =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].second;
      }

      const auto decision = MakeStBoundaryDecision(s, s_lower, s_upper);

      st_boundary_decisions.emplace_back(st_boundary.id(), decision);

      if (st_boundary_with_decision->raw_st_boundary()->is_protective()) {
        const auto& protected_st_boundary_id =
            st_boundary_with_decision->raw_st_boundary()
                ->protected_st_boundary_id();
        if (protected_st_boundary_id.has_value()) {
          if (const auto original_st_boundary_wd = FindOrNull(
                  original_st_boundary_wd_map_, *protected_st_boundary_id);
              nullptr != original_st_boundary_wd &&
              (*original_st_boundary_wd)->decision_type() ==
                  StBoundaryProto::UNKNOWN) {
            st_boundary_decisions.emplace_back(*protected_st_boundary_id,
                                               decision);
          }
        }
      }
    }
  }
  return st_boundary_decisions;
}

void DpSvtCost::GetStBoundaryCostAndDecisions(
    const SvtGraphPoint& prev_svt_graph_point,
    const SvtGraphPoint& svt_graph_point, double av_speed,
    double* st_boundary_cost,
    std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions) {
  CHECK_NOTNULL(st_boundary_cost);
  CHECK_NOTNULL(st_boundary_decisions);
  CHECK(st_boundary_decisions->empty());
  constexpr double kDeltaVel = Kph2Mps(10.0);

  const double prev_s = prev_svt_graph_point.point().s();
  const double prev_t = prev_svt_graph_point.point().t();

  double cost = 0.0;
  for (const auto* st_boundary_with_decision :
       *sorted_st_boundaries_with_decision_) {
    if (st_boundary_with_decision->decision_type() == StBoundaryProto::IGNORE) {
      continue;
    }

    double s = svt_graph_point.point().s();
    double t = svt_graph_point.point().t();
    const auto& st_boundary = *st_boundary_with_decision->st_boundary();
    if (t < st_boundary.min_t() ||
        (prev_t >= st_boundary.min_t() && t > st_boundary.max_t())) {
      continue;
    }

    if (t >= st_boundary.max_t()) {
      s = Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.max_t()));
      t = st_boundary.max_t();
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      auto& boundary_bounds =
          boundary_cost_[boundary_index][svt_graph_point.index_t()];
      if (boundary_bounds.first < 0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        CHECK(s_range.has_value());
        boundary_bounds = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper = boundary_bounds.first;
        s_lower = boundary_bounds.second;
      }
    }

    DecisionType decision = StBoundaryProto::UNKNOWN;
    bool decided_by_protective = false;
    if (st_boundary_with_decision->decision_type() !=
        StBoundaryProto::UNKNOWN) {
      decision = st_boundary_with_decision->decision_type();
    } else {
      const auto prev_point_decision =
          prev_svt_graph_point.GetStBoundaryDecision(st_boundary.id());
      if (prev_point_decision) {
        CHECK_NE(*prev_point_decision, StBoundaryProto::UNKNOWN);

        decision = *prev_point_decision;
      } else {
        for (const auto& st_boundary_decision : *st_boundary_decisions) {
          if (st_boundary_decision.first == st_boundary.id()) {
            decision = st_boundary_decision.second;
            decided_by_protective = true;
            CHECK_NE(decision, StBoundaryProto::UNKNOWN);
            CHECK(ContainsKey(original_st_boundary_wd_map_, st_boundary.id()));
            break;
          }
        }
        if (!decided_by_protective) {
          const double decisive_s =
              Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.min_t()));
          const double first_s_upper = st_boundary.upper_points().front().s();
          const double first_s_lower = st_boundary.lower_points().front().s();
          decision = MakeStBoundaryDecision(
              decisive_s, first_s_lower, first_s_upper,
              st_boundary_with_decision->decision_param().dp_follow_lead_ratio);
        }
      }
      if (!decided_by_protective) {
        st_boundary_decisions->emplace_back(st_boundary.id(), decision);

        if (st_boundary_with_decision->raw_st_boundary()->is_protective()) {
          const auto& protected_st_boundary_id =
              st_boundary_with_decision->raw_st_boundary()
                  ->protected_st_boundary_id();
          if (protected_st_boundary_id.has_value()) {
            if (const auto original_st_boundary_wd = FindOrNull(
                    original_st_boundary_wd_map_, *protected_st_boundary_id);
                nullptr != original_st_boundary_wd &&
                (*original_st_boundary_wd)->decision_type() ==
                    StBoundaryProto::UNKNOWN) {
              st_boundary_decisions->emplace_back(*protected_st_boundary_id,
                                                  decision);
            }
          }
        }
      }
    }

    double delta_t = -std::numeric_limits<double>::max();
    double obj_v = st_boundary_with_decision->obj_pose_info().v();

    bool is_debug_decision_cost = false;
    std::string obj_id = "";
    if (st_boundary.object_id().has_value() &&
        st_boundary.object_id().value() == obj_id &&
        (StBoundaryProto::OVERTAKE == decision)) {
      is_debug_decision_cost = true;
    }

#if DEBUG_DP_DECISION_COST
    if (is_debug_decision_cost) {
      std::cout << "----------------------" << std::endl;
      std::cout << "------------debug decision cost" << std::endl;
      std::cout << "[ info - " << obj_id << " ]:" << " ego_vel "
                << Mps2Kph(av_speed) << " obj_v " << Mps2Kph(obj_v)
                << " enable_add_collision_risk_cost_for_dp "
                << st_boundary_with_decision->decision_param()
                       .enable_add_collision_risk_cost_for_dp
                << " decision " << decision << std::endl;
    }
#endif

    if (st_boundary_with_decision->decision_param()
            .enable_add_collision_risk_cost_for_dp &&
        ((av_speed - obj_v) > kDeltaVel)) {
      double av_t = st_boundary.min_s() / (av_speed + kEps);
      double obj_t = st_boundary.min_t();

      double obj_dist = std::max(
          std::min(
              std::fabs(st_boundary_with_decision->obj_frenet_polygon().l_max),
              std::fabs(
                  st_boundary_with_decision->obj_frenet_polygon().l_min)) -
              1.0,
          0.0);
      double tmp_t = std::numeric_limits<double>::max();
      if (obj_v > 0.0) {
        tmp_t = std::fabs(obj_dist / obj_v);
      }
      obj_t = std::min(obj_t, tmp_t);
      delta_t = av_t - obj_t;

#if DEBUG_DP_DECISION_COST
      if (is_debug_decision_cost) {
        std::cout << "[ risk_cost - " << obj_id << " ]:" << " curr_s " << s
                  << " t " << t << " ego_s " << st_boundary.min_s() << " ego_v "
                  << av_speed << " av_t " << av_t << " obj_t " << obj_t
                  << std::endl;
      }
#endif
    }

    double agent_dist_to_collision = st_boundary_with_decision->dl();
    if (st_boundary.traj_id().has_value()) {
      const SpacetimeObjectTrajectory* st_traj =
          st_traj_mgr_.FindTrajectoryById(st_boundary.traj_id().value());
      if (nullptr != st_traj && 0 != st_boundary.overlap_infos().size()) {
        const auto& first_overlap_info = st_boundary.overlap_infos().front();
        if (0 <= first_overlap_info.obj_idx &&
            first_overlap_info.obj_idx < st_traj->states().size()) {
          const auto* overlap_agent_traj_point =
              st_traj->states()[first_overlap_info.obj_idx].traj_point;
          agent_dist_to_collision = overlap_agent_traj_point->s();
        }
      }
    }

    double ego_yield_acc = 0.0, obj_yield_acc = 0.0;
    if (st_boundary_with_decision->decision_param()
            .enable_add_collision_risk_cost_for_dp) {
      ego_yield_acc =
          calc_yield_result(av_speed, st_boundary.min_t(), st_boundary.min_s());
      double ego_t = st_boundary.min_s() / (av_speed + kEps);
      obj_yield_acc = calc_yield_result(obj_v, ego_t, agent_dist_to_collision);
#if DEBUG_DP_DECISION_COST
      if (is_debug_decision_cost) {
        std::cout << "[ rationality_cost - " << obj_id
                  << " ]:" << " ego_yield_acc " << ego_yield_acc << " av_speed "
                  << av_speed << " obj_t " << st_boundary.min_t() << " ego_s "
                  << st_boundary.min_s() << " obj_yield_acc " << obj_yield_acc
                  << " obj_vel " << obj_v << " ego_t " << ego_t
                  << " agent_dist_to_collision " << agent_dist_to_collision
                  << std::endl;
      }
#endif
    }

    cost += GetStBoundaryCost(
        decision, av_speed, st_boundary, s, s_lower, s_upper, t,
        st_boundary_with_decision->follow_standstill_distance(),
        follow_distance_rel_speed_plf_, *speed_planning_params_, *params_,
        st_boundary_with_decision->decision_param()
            .enable_add_collision_risk_cost_for_dp,
        st_boundary_with_decision->decision_param()
            .enable_left_turn_overtake_cost,
        delta_t, obj_v, agent_dist_to_collision, ego_yield_acc, obj_yield_acc,
        st_boundary.min_s(), is_debug_decision_cost);
  }
  *st_boundary_cost = cost * unit_t_;
}

double DpSvtCost::GetSpatialPotentialCost(double s) const {
  return (total_s_ - s) * params_->spatial_potential_weight();
}

double DpSvtCost::GetSpeedLimitCost(double speed, double speed_limit) const {
  double cost = 0.0;
  const double det_speed =
      std::clamp(speed - speed_limit, -kMaxSpeedLimitDiff, kMaxSpeedLimitDiff);
  if (det_speed > 0) {
    cost += params_->exceed_speed_penalty() * params_->speed_weight() *
            (det_speed * det_speed) * unit_t_;
  } else if (det_speed < 0) {
    cost += params_->low_speed_penalty() * params_->speed_weight() *
            (det_speed * det_speed) * unit_t_;
  }
  return cost;
}

double DpSvtCost::GetReferenceSpeedCost(double speed,
                                        double cruise_speed) const {
  double cost = 0.0;

  if (FLAGS_enable_sampling_dp_reference_speed) {
    const double diff_speed = speed - cruise_speed;
    cost += params_->reference_speed_penalty() * params_->speed_weight() *
            diff_speed * diff_speed * unit_t_;
  }

  return cost;
}

double DpSvtCost::GetAccelCost(double accel) const {
  double cost = 0.0;

  const double accel_sq = accel * accel;
  const double accel_penalty = params_->accel_penalty();
  const double decel_penalty = params_->decel_penalty();

  if (accel > 0.0) {
    cost = accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }

  return cost * unit_t_;
}

}  // namespace e2e_noa::planning
