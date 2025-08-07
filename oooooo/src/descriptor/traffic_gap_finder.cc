#include "descriptor/traffic_gap_finder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "prediction/predicted_trajectory.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {

namespace {

const std::vector<double> kDiscreteAccelerations = {
    0.5, 1.0, 1.3, -0.5, -1.0, 1.3, -1.5, -2.0,
};
const std::vector<double> kDiscreteDecAccelerations = {
    -0.5, -0.8, -1.0, -1.3, -1.5, -1.8,
};
const std::vector<double> kEvaluationKeys = {0.5, 1.0, 1.5, 2.0,
                                             2.5, 3.0, 3.5, 4.0};

constexpr double kMinCoastingSpeed = 3.0;

struct SVState {
  double s, v;
};

std::vector<SVState> FindObjectStates(
    const SpacetimeObjectTrajectory* object_traj, double start_s) {
  if (object_traj == nullptr) return std::vector<SVState>();

  if (object_traj->is_stationary()) {
    std::vector<SVState> result;
    result.reserve(kEvaluationKeys.size());
    for (size_t i = 0; i < kEvaluationKeys.size(); ++i) {
      result.emplace_back(SVState{.s = start_s, .v = 0.0});
    }
    return result;
  }

  std::vector<SVState> states;
  states.reserve(kEvaluationKeys.size());

  for (const double t : kEvaluationKeys) {
    const auto nearest_it = std::min_element(
        object_traj->states().begin(), object_traj->states().end(),
        [t](const SpacetimeObjectState& lhs, const SpacetimeObjectState& rhs) {
          return std::abs(t - lhs.traj_point->t()) <
                 std::abs(t - rhs.traj_point->t());
        });

    states.emplace_back(SVState{.s = start_s + nearest_it->traj_point->s(),
                                .v = nearest_it->traj_point->v()});
  }

  return states;
}

absl::StatusOr<int> EvaluateTrafficGap(const TrafficGap& candidate,
                                       const FrenetBox& ego_frenet_box,
                                       double ego_init_v, double* target_v,
                                       double* target_time, bool is_only_dec) {
  std::vector<std::vector<SVState>> all_leader_sv;
  for (const auto* traj : candidate.leader_trajectories) {
    all_leader_sv.emplace_back(FindObjectStates(traj, candidate.s_end));
  }

  std::vector<std::vector<SVState>> all_follower_sv;
  for (const auto* traj : candidate.follower_trajectories) {
    all_follower_sv.emplace_back(FindObjectStates(traj, candidate.s_start));
  }

  if (is_only_dec) {
    for (int strategy_index = 0;
         strategy_index < kDiscreteDecAccelerations.size(); ++strategy_index) {
      const double accel = kDiscreteDecAccelerations[strategy_index];
      for (int i = 0; i < kEvaluationKeys.size(); ++i) {
        const double t = kEvaluationKeys[i];

        const double accel_v = ego_init_v + accel * t;
        const double ego_v = std::max(kMinCoastingSpeed, accel_v);
        double ego_delta_s;
        if (accel_v < kMinCoastingSpeed) {
          const double break_t = std::abs(accel) < 1e-7
                                     ? 0.0
                                     : (kMinCoastingSpeed - ego_init_v) / accel;
          ego_delta_s = (kMinCoastingSpeed + ego_init_v) * break_t * 0.5 +
                        kMinCoastingSpeed * (t - break_t);
        } else {
          ego_delta_s = (ego_v + ego_init_v) * t * 0.5;
        }
        const double ego_front_s = ego_delta_s + ego_frenet_box.s_max;
        const double ego_rear_s = ego_delta_s + ego_frenet_box.s_min;

        const auto ttc_ok =
            [i, ego_v](const std::vector<std::vector<SVState>>& all_states,
                       double ego_s, bool leader) {
              for (const auto& states : all_states) {
                if (states.empty()) continue;

                if ((leader && ego_s > states[i].s) ||
                    (!leader && ego_s < states[i].s)) {
                  return false;
                }
                constexpr double kMinTTC = 2.5;
                const double ttc =
                    (states[i].s - ego_s) / (ego_v - states[i].v);
                if (ttc >= 0.0 && ttc < kMinTTC) {
                  return false;
                }
              }
              return true;
            };

        const bool leader_ok = ttc_ok(all_leader_sv, ego_front_s, true);
        const bool follower_ok = ttc_ok(all_follower_sv, ego_rear_s, false);

        if (leader_ok && follower_ok) {
          *target_v = accel_v;
          *target_time = t;
          return strategy_index;
        }
      }
    }
  } else {
    for (int strategy_index = 0; strategy_index < kDiscreteAccelerations.size();
         ++strategy_index) {
      const double accel = kDiscreteAccelerations[strategy_index];
      for (int i = 0; i < kEvaluationKeys.size(); ++i) {
        const double t = kEvaluationKeys[i];

        const double accel_v = ego_init_v + accel * t;
        const double ego_v = std::max(kMinCoastingSpeed, accel_v);
        double ego_delta_s;
        if (accel_v < kMinCoastingSpeed) {
          const double break_t = std::abs(accel) < 1e-7
                                     ? 0.0
                                     : (kMinCoastingSpeed - ego_init_v) / accel;
          ego_delta_s = (kMinCoastingSpeed + ego_init_v) * break_t * 0.5 +
                        kMinCoastingSpeed * (t - break_t);
        } else {
          ego_delta_s = (ego_v + ego_init_v) * t * 0.5;
        }
        const double ego_front_s = ego_delta_s + ego_frenet_box.s_max;
        const double ego_rear_s = ego_delta_s + ego_frenet_box.s_min;

        const auto ttc_ok =
            [i, ego_v](const std::vector<std::vector<SVState>>& all_states,
                       double ego_s, bool leader) {
              for (const auto& states : all_states) {
                if (states.empty()) continue;

                if ((leader && ego_s > states[i].s) ||
                    (!leader && ego_s < states[i].s)) {
                  return false;
                }
                constexpr double kMinTTC = 2.5;
                const double ttc =
                    (states[i].s - ego_s) / (ego_v - states[i].v);
                if (ttc >= 0.0 && ttc < kMinTTC) {
                  return false;
                }
              }
              return true;
            };

        const bool leader_ok = ttc_ok(all_leader_sv, ego_front_s, true);
        const bool follower_ok = ttc_ok(all_follower_sv, ego_rear_s, false);

        if (leader_ok && follower_ok) {
          *target_v = accel_v;
          *target_time = t;
          return strategy_index;
        }
      }
    }
  }

  return absl::NotFoundError("");
}

}  // namespace

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    std::vector<std::string> lc_lead_obj_ids) {
  struct PlannerObjectProjectionInfo {
    absl::Span<const SpacetimeObjectTrajectory* const> st_trajectories;
    FrenetBox frenet_box;
  };

  std::vector<PlannerObjectProjectionInfo> objects_on_lane;
  for (const auto& [_, trajectories] : st_traj_mgr.object_trajectories_map()) {
    if (trajectories.empty()) continue;
    const PlannerObject& obj = trajectories.front()->planner_object();

    if (obj.is_stationary()) {
      continue;
    }

    ASSIGN_OR_CONTINUE(
        const auto frenet_box,
        target_frenet_frame.QueryFrenetBoxAt(obj.bounding_box()));

    constexpr double kLateralThreshold = 1.0;
    if (frenet_box.l_min > kLateralThreshold ||
        frenet_box.l_max < -kLateralThreshold) {
      continue;
    }

    objects_on_lane.emplace_back(PlannerObjectProjectionInfo{
        .st_trajectories = trajectories, .frenet_box = frenet_box});
  }
  if (objects_on_lane.empty()) {
    return {};
  }

  std::stable_sort(objects_on_lane.begin(), objects_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.frenet_box.s_min < b.frenet_box.s_min;
                   });

  const double ego_center_s = ego_frenet_box.center_s();
  const auto closest_object_it = std::min_element(
      objects_on_lane.begin(), objects_on_lane.end(),
      [ego_center_s](const auto& lhs, const auto& rhs) {
        return std::abs(lhs.frenet_box.center_s() - ego_center_s) <
               std::abs(rhs.frenet_box.center_s() - ego_center_s);
      });

  TrafficGap follow;
  follow.leader_trajectories = closest_object_it->st_trajectories;
  follow.s_end = closest_object_it->frenet_box.s_min;
  if (closest_object_it == objects_on_lane.begin()) {
    follow.s_start = std::numeric_limits<double>::lowest();
  } else {
    follow.s_start = std::prev(closest_object_it)->frenet_box.s_max;
    follow.follower_trajectories =
        std::prev(closest_object_it)->st_trajectories;
  }

  TrafficGap lead;
  lead.follower_trajectories = closest_object_it->st_trajectories;
  lead.s_start = closest_object_it->frenet_box.s_max;
  if (std::next(closest_object_it) == objects_on_lane.end()) {
    lead.s_end = std::numeric_limits<double>::max();
  } else {
    lead.leader_trajectories = std::next(closest_object_it)->st_trajectories;
    lead.s_end = std::next(closest_object_it)->frenet_box.s_min;
  }

  for (int i = 0; i < objects_on_lane.size(); i++) {
    const PlannerObject& obj =
        objects_on_lane[i].st_trajectories.front()->planner_object();
    if (std::find(lc_lead_obj_ids.begin(), lc_lead_obj_ids.end(), obj.id()) !=
        lc_lead_obj_ids.end()) {
      TrafficGap dp;
      dp.leader_trajectories = objects_on_lane[i].st_trajectories;
      dp.s_end = objects_on_lane[i].frenet_box.s_min;
      if (i == 0) {
        dp.s_start = std::numeric_limits<double>::lowest();
      } else {
        dp.s_start = objects_on_lane[i - 1].frenet_box.s_max;
        dp.follower_trajectories = objects_on_lane[i - 1].st_trajectories;
      }
      return {follow, lead, dp};
    }
  }
  return {follow, lead};
}
absl::StatusOr<TrafficGapResult> EvaluateAndTakeBestTrafficGap(
    absl::Span<const TrafficGap> candidate_gaps,
    const FrenetBox& ego_frenet_box, const FrenetFrame& target_frenet_frame,
    double ego_init_v, double speed_limit, double navi_dist, int lc_num,
    bool no_acc_gap, double leader_obj_s_min, double leader_obj_v,
    ad_e2e::planning::PushDirection push_dir) {
  if (candidate_gaps.empty()) {
    return absl::NotFoundError("Input candidate gaps empty!");
  }

  double min_cost = DBL_MAX;
  int best_strategy = INT_MAX;
  int best_idx = -1;
  double target_v = -1.0;
  double target_time = -1.0;
  double best_v = -1.0;

  for (int i = 0; i < candidate_gaps.size(); ++i) {
    ASSIGN_OR_CONTINUE(
        const int strategy_index,
        EvaluateTrafficGap(candidate_gaps[i], ego_frenet_box, ego_init_v,
                           &target_v, &target_time, false));
    double single_cost =
        std::max(target_time - 2.0, 0.0) * 20.0 +
        std::fabs(kDiscreteAccelerations[strategy_index]) * 8.0;

    if (single_cost < min_cost) {
      min_cost = single_cost;
      best_idx = i;
      best_strategy = strategy_index;
      best_v = target_v;
    }
  }

  if (best_idx == -1) {
    return absl::NotFoundError("No viable candidate after evaluation!");
  }

  bool need_select_dec_gap = false;

  if (((navi_dist / ego_init_v) < 9.0) &&
      (best_strategy == 0 || best_strategy == 1 || best_strategy == 2 ||
       best_strategy == 5)) {
    need_select_dec_gap = true;
  }

  if ((best_strategy == 0 || best_strategy == 1 || best_strategy == 2 ||
       best_strategy == 5) &&
      (lc_num >= 2) && ((navi_dist / lc_num) < 80.0)) {
    need_select_dec_gap = true;
  }

  if ((best_strategy == 0 || best_strategy == 1 || best_strategy == 2 ||
       best_strategy == 5) &&
      no_acc_gap) {
    need_select_dec_gap = true;
  }

  if ((best_strategy == 0 || best_strategy == 1 || best_strategy == 2 ||
       best_strategy == 5) &&
      (leader_obj_s_min != -1.0) && (leader_obj_v != -1.0)) {
    double time_buffer = ego_init_v * 2.0 + 3.0;
    double ttc_dist = (leader_obj_v - ego_init_v) * 2.0 + 3.0;
    if ((time_buffer > leader_obj_s_min) || (ttc_dist > leader_obj_s_min)) {
      need_select_dec_gap = true;
    }
  }

  if (need_select_dec_gap) {
    double dec_min_cost = DBL_MAX;
    int best_strategy = INT_MAX;
    int dec_best_idx = -1;
    double target_v = -1.0;
    double dec_target_time = -1.0;
    for (int i = 0; i < candidate_gaps.size(); ++i) {
      ASSIGN_OR_CONTINUE(
          const int dec_strategy_index,
          EvaluateTrafficGap(candidate_gaps[i], ego_frenet_box, ego_init_v,
                             &target_v, &dec_target_time, true));
      double dec_single_cost =
          std::max(dec_target_time - 2.0, 0.0) * 20.0 +
          std::fabs(kDiscreteDecAccelerations[dec_strategy_index]) * 8.0;
      if (dec_min_cost > dec_single_cost) {
        dec_min_cost = dec_single_cost;
        dec_best_idx = i;
      }
    }
    if (dec_best_idx == -1) {
      return absl::NotFoundError("No viable candidate after evaluation!");
    } else {
      const auto& dec_best_gap = candidate_gaps[dec_best_idx];
      TrafficGapResult dec_result;
      if (!dec_best_gap.leader_trajectories.empty()) {
        dec_result.leader_id = {
            dec_best_gap.leader_trajectories.front()->object_id().data(),
            dec_best_gap.leader_trajectories.front()->object_id().size()};
      }
      if (!dec_best_gap.follower_trajectories.empty()) {
        dec_result.follower_id = {
            dec_best_gap.follower_trajectories.front()->object_id().data(),
            dec_best_gap.follower_trajectories.front()->object_id().size()};
      }
      double dec_a_buffer = -0.25;
      if (!dec_best_gap.leader_trajectories.empty()) {
        const auto object =
            dec_best_gap.leader_trajectories.front()->planner_object();
        double leader_object_a = object.pose().a();
        if (leader_object_a < -0.5) {
          dec_a_buffer = 0.5 * leader_object_a;
          dec_a_buffer = std::max(-0.6, dec_a_buffer);
        }
      }
      dec_result.dec_gap_target_a =
          kDiscreteDecAccelerations[dec_best_idx] + dec_a_buffer;
      dec_result.dec_gap_target_speed =
          std::max(ego_init_v + 2.0 * kDiscreteDecAccelerations[dec_best_idx] +
                       dec_a_buffer,
                   ego_init_v - 4.0);

      return dec_result;
    }
  } else {
    const auto& best_gap = candidate_gaps[best_idx];
    TrafficGapResult result;
    if (!best_gap.leader_trajectories.empty()) {
      result.leader_id = {
          best_gap.leader_trajectories.front()->object_id().data(),
          best_gap.leader_trajectories.front()->object_id().size()};
    }
    if (!best_gap.follower_trajectories.empty()) {
      result.follower_id = {
          best_gap.follower_trajectories.front()->object_id().data(),
          best_gap.follower_trajectories.front()->object_id().size()};
    }
    if ((best_strategy == 0 || best_strategy == 1 || best_strategy == 2 ||
         best_strategy == 5)) {
      best_v = std::min(best_v + 1.5,
                        std::max(speed_limit * 1.2, speed_limit + 3.8));
      if (!best_gap.leader_trajectories.empty() &&
          !best_gap.follower_trajectories.empty()) {
        const auto gap_leader =
            best_gap.leader_trajectories.front()->planner_object();
        const auto gap_follower =
            best_gap.follower_trajectories.front()->planner_object();
        ASSIGN_OR_RETURN(
            const auto gap_leader_frenet_box,
            target_frenet_frame.QueryFrenetBoxAt(gap_leader.bounding_box()),
            _ << "Cannot project gap leader box onto plan passage.");
        ASSIGN_OR_RETURN(
            const auto gap_follower_frenet_box,
            target_frenet_frame.QueryFrenetBoxAt(gap_follower.bounding_box()),
            _ << "Cannot project gap follower box onto plan passage.");
        if (gap_leader.pose().v() >= gap_follower.pose().v() &&
            gap_leader_frenet_box.s_min > ego_frenet_box.s_max) {
          best_v = std::min(std::max(gap_follower.pose().v() + 1.2, best_v),
                            speed_limit * 1.2);
        }
      }
      if (best_gap.leader_trajectories.empty() &&
          !best_gap.follower_trajectories.empty()) {
        const auto gap_follower =
            best_gap.follower_trajectories.front()->planner_object();
        ASSIGN_OR_RETURN(
            const auto gap_follower_frenet_box,
            target_frenet_frame.QueryFrenetBoxAt(gap_follower.bounding_box()),
            _ << "Cannot project gap follower box onto plan passage.");
        best_v = std::min(std::max(gap_follower.pose().v() + 1.2, best_v),
                          speed_limit * 1.2);
      }
    }
    if (!best_gap.leader_trajectories.empty() &&
        best_gap.follower_trajectories.empty()) {
      double target_follow_speed =
          best_gap.leader_trajectories.front()->planner_object().pose().v();
      double target_follow_acc =
          best_gap.leader_trajectories.front()->planner_object().pose().a();

      if (target_follow_acc < -0.5) {
        target_follow_speed = target_follow_speed - 3.0;

      } else {
        target_follow_speed = target_follow_speed - 2.0;
      }
      target_follow_speed = std::min(speed_limit, target_follow_speed);
      if (target_follow_speed > ego_init_v) {
        best_v = std::min(target_follow_speed, ego_init_v + 3.0);
      } else {
        best_v = std::max(target_follow_speed, ego_init_v - 3.0);
      }
    }
    double acc_gap_a = std::min((best_v - ego_init_v) / 2.0, 1.5);
    result.acc_gap_target_speed = best_v;
    result.acc_gap_target_a = acc_gap_a;
    return result;
  }
}

}  // namespace e2e_noa::planning
