#include "speed/decision/pre_st_boundary_modifier.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/hermite_spline.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/decision/st_boundary_modifier_util.h"
#include "speed/overlap_info.h"
#include "speed/speed_planning_util.h"
#include "speed/st_boundary.h"
#include "speed/st_point.h"
#include "speed_planning.pb.h"
#include "trajectory_point.pb.h"
#include "util/map_util.h"
#include "util/path_util.h"

namespace e2e_noa {
namespace planning {
namespace {

std::optional<StBoundaryModificationResult> ModifyOncomingStBoundary(
    const PreStboundaryModifierInput& input, const StGraph& st_graph,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  constexpr double kMaxTimeLimit = 1.5;

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return std::nullopt;
  }

  if (st_boundary.bottom_left_point().t() > kMaxTimeLimit) {
    return std::nullopt;
  }

  if (st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS) {
    return std::nullopt;
  }

  const auto& path_semantics = *input.path_semantics;
  const auto& plan_passage = *input.plan_passage;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_STRAIGHT) {
    const double reac_time = 1.5;
    const double safet_buffer = 8.0;
    double safet_dist =
        safet_buffer +
        (input.current_v + std::fabs(st_traj.planner_object().pose().v())) *
            reac_time;
    safet_dist = std::clamp(safet_dist, 0.0, 30.0);
    const auto& path_points = *input.path;
    Vec2d start_point(path_points.front().x(), path_points.front().y());
    const auto ego_fpos = plan_passage.QueryFrenetCoordinateAt(start_point);
    const auto obs_fbox = plan_passage.QueryFrenetBoxAtContour(
        st_traj.planner_object().contour());
    double ds = std::numeric_limits<double>::max();
    if (obs_fbox.ok() && ego_fpos.ok()) {
      ds = obs_fbox->s_min - ego_fpos->s -
           input.vehicle_geom->front_edge_to_center();
    }
    const auto& current_lane_info = *path_semantics.front().lane_info;

    const std::string debug = st_boundary_wd.object_id().value() +
                              " ds = " + std::to_string(ds) +
                              " safet_dist = " + std::to_string(safet_dist);

    if (!current_lane_info.junction_id().empty() &&
        (st_traj.trajectory().intention() ==
             TrajectoryIntention::INTENTION_TURN_LEFT ||
         st_traj.trajectory().intention() ==
             TrajectoryIntention::INTENTION_TURN_RIGHT)) {
      Log2FG::LogDataV0("ModifyOncomingStBoundary",
                        st_boundary_wd.object_id().value() + " INTENTION");
      return std::nullopt;
    }

    if (!current_lane_info.junction_id().empty() && ds < safet_dist) {
      Log2FG::LogDataV0("ModifyOncomingStBoundary",
                        st_boundary_wd.object_id().value() + "safet_dist");
      return std::nullopt;
    }
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      st_traj.states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kOnComingThreshold = 5.0 * M_PI / 6.0;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) <
      kOnComingThreshold) {
    return std::nullopt;
  }

  VLOG(2) << "St-boundary " << st_boundary_wd.id()
          << " is considered to be ONCOMING.";

  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() > const_speed_s) {
    return std::nullopt;
  }
  constexpr double kUncomfortableDecel = 0.8;
  constexpr double kAvMinVel = 3.0;
  const double max_decel_s =
      std::max(0.5 * Sqr(current_v) / kUncomfortableDecel,
               kAvMinVel * st_boundary.bottom_right_point().t());
  if (max_decel_s < st_boundary.bottom_right_point().s()) {
    const double estimated_av_decel =
        2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
        Sqr(st_boundary.bottom_right_point().t());
    if (estimated_av_decel < kUncomfortableDecel) {
      return std::nullopt;
    }
  }
  if (IsConsiderOncomingObs(*input.st_traj_mgr, st_boundary, plan_passage,
                            *input.vehicle_geom, current_v, path)) {
    return std::nullopt;
  }

  VLOG(2) << "Modify ONCOMING st-boundary " << st_boundary.id();
  constexpr double kOncomingReactionTime = 0.5;
  constexpr double kOncomingObjectDecel = -1.8;
  constexpr double kOncomingObjectMildDecel = -1.5;

  const double oncoming_obj_decel =
      st_boundary.overlap_meta()->source() == StOverlapMetaProto::AV_CUTIN
          ? std::min(st_traj.planner_object().pose().a(),
                     kOncomingObjectMildDecel)
          : std::min(st_traj.planner_object().pose().a(), kOncomingObjectDecel);

  auto new_st_traj = CreateSpacetimeTrajectoryByDecelerationAfterDelay(
      st_traj, kOncomingReactionTime, oncoming_obj_decel);

  auto st_boundary_output =
      st_graph.MapMovingSpacetimeObject(new_st_traj, false, false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::ONCOMING;
  for (auto& st_boundary : new_st_boundaries) {
    st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
    if (st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundaries_wd.emplace_back(std::move(st_boundary),
                                    st_boundary_wd.decision_type(),
                                    st_boundary_wd.decision_reason());
    } else {
      st_boundaries_wd.emplace_back(
          std::move(st_boundary), st_boundary_wd.decision_type(),
          st_boundary_wd.decision_reason(),
          absl::StrCat(
              st_boundary_wd.decision_info(), " and keep it after modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(), 0.0, 0.0);
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult>
ModifyLaneChangeBackObjectStBoundary(
    const PreStboundaryModifierInput& input, const StGraph& st_graph,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  if (!st_boundary_wd.raw_st_boundary()) return std::nullopt;
  const auto& origin_st_boundary = *st_boundary_wd.raw_st_boundary();
  double back_object_v = st_boundary_wd.obj_pose_info().v();

  constexpr double kTime = 7;
  constexpr double kLeadTimeHeadway = 0.6;
  constexpr double kAlimit = 0.5;
  constexpr double kFollowerimeHeadway = 1.0;
  double dist_safe_follower = std::numeric_limits<double>::max();
  bool falg_have_leading_obj = false;
  std::optional<std::pair<double, double>> follow_s_range = std::nullopt;
  double gap_s_buffer = 0.0;

  if (!input.leader_set) {
    const absl::flat_hash_set<std::string> leader_set = *input.leader_set;
    for (auto& st_boundary_with_decision : *input.st_boundaries_with_decision) {
      const auto& st_boundary = st_boundary_with_decision.raw_st_boundary();
      if (!st_boundary) continue;
      if (st_boundary->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
        continue;
      }
      const auto object_id = st_boundary->object_id();
      if (!object_id.has_value()) {
        continue;
      }
      if (leader_set.find(object_id.value()) == leader_set.end()) {
        continue;
      }
      if (!InRange(kTime, st_boundary->min_t(), st_boundary->max_t())) {
        continue;
      }

      follow_s_range = st_boundary->GetBoundarySRange(kTime);
      const auto object_v = st_boundary->GetStBoundarySpeedAtT(kTime);
      if (!object_v.has_value()) {
        continue;
      }

      double gap_s_buffer_tmp =
          *object_v * (kLeadTimeHeadway + kFollowerimeHeadway) +
          2.0 * st_boundary_wd.follow_standstill_distance();
      if (dist_safe_follower > follow_s_range->second - gap_s_buffer_tmp) {
        dist_safe_follower =
            std::max(follow_s_range->second - gap_s_buffer_tmp, 0.0);
        falg_have_leading_obj = true;
        gap_s_buffer = gap_s_buffer_tmp;
      }
    }
  }
  if (!falg_have_leading_obj) {
    gap_s_buffer = back_object_v * (2.0 * kLeadTimeHeadway) +
                   st_boundary_wd.follow_standstill_distance();
    dist_safe_follower =
        std::min(std::max(input.current_v * kTime +
                              0.5 * kAlimit * kTime * kTime - gap_s_buffer,
                          0.0),
                 dist_safe_follower);
  }

  constexpr double kReactionTime = 0.0;
  double obj_arrive_point_decel =
      2.0 * (dist_safe_follower - back_object_v * kTime) / Sqr(kTime);
  const double obj_speed_arrive_point =
      back_object_v + obj_arrive_point_decel * kTime;
  const std::string debug =
      st_boundary_wd.object_id().value() +
      " dist_safe_follower = " + std::to_string(dist_safe_follower) +
      " obj_arrive_point_decel = " + std::to_string(obj_arrive_point_decel) +
      " follow_s_range = " + std::to_string(follow_s_range->second) +
      " gap_s_buffer = " + std::to_string(gap_s_buffer) +
      " obj_speed_arrive_point = " + std::to_string(obj_speed_arrive_point);

  auto new_st_traj = CreateSpacetimeTrajectoryByDecelerationAfterDelay(
      st_traj, kReactionTime, obj_arrive_point_decel);

  auto st_boundary_output =
      st_graph.MapMovingSpacetimeObject(new_st_traj, false, false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  std::optional<std::pair<double, double>> new_s_range = std::nullopt;
  for (auto& st_boundary : new_st_boundaries) {
    if (!InRange(kTime, st_boundary->min_t(), st_boundary->max_t())) {
      continue;
    }
    new_s_range = st_boundary->GetBoundarySRange(kTime);
    break;
  }
  auto origin_s_range = origin_st_boundary.GetBoundarySRange(kTime);
  if (!new_s_range.has_value() || !origin_s_range.has_value() ||
      new_s_range->first >= origin_s_range->first) {
    Log2FG::LogDataV0("ModifyLaneChangeBackObjectStBoundary", "no need modif");
    return std::nullopt;
  }

  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::LANCHANGE_BACK_CAR;
  for (auto& st_boundary : new_st_boundaries) {
    st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
    if (st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundaries_wd.emplace_back(std::move(st_boundary),
                                    st_boundary_wd.decision_type(),
                                    st_boundary_wd.decision_reason());
    } else {
      st_boundaries_wd.emplace_back(
          std::move(st_boundary), st_boundary_wd.decision_type(),
          st_boundary_wd.decision_reason(),
          absl::StrCat(
              st_boundary_wd.decision_info(), " and keep it after modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(), 0.0, 0.0);
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult> ModifyRightTurnStBoundary(
    const PreStboundaryModifierInput& input, const StGraph& st_graph,
    const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  constexpr double kMinOverlapTime = 3.0;
  constexpr double kMinConsiderSpeed = 10.0;

  if (current_v > Kph2Mps(kMinConsiderSpeed)) {
    return std::nullopt;
  }

  if (st_boundary.bottom_left_point().t() < kMinOverlapTime) {
    return std::nullopt;
  }

  if (st_boundary.overlap_meta()->source() == StOverlapMetaProto::LANE_CROSS) {
    return std::nullopt;
  }

  if (current_v * st_boundary.upper_left_point().t() +
          0.5 * Sqr(st_boundary.upper_left_point().t()) <
      st_boundary.upper_left_point().s()) {
    return std::nullopt;
  }

  const auto& path_semantics = *input.path_semantics;
  const auto& plan_passage = *input.plan_passage;
  if (!path_semantics.empty() && path_semantics.front().lane_semantic ==
                                     LaneSemantic::INTERSECTION_RIGHT_TURN) {
    const double reac_time = 2.0;
    const double safety_buffer = 1.0;
    double safet_dist = std::max(
        1.0, safety_buffer +
                 (st_traj.planner_object().pose().v() - input.current_v) *
                     reac_time);
    const auto& path_points = *input.path;
    Vec2d start_point(path_points.front().x(), path_points.front().y());
    const auto ego_fpos = plan_passage.QueryFrenetCoordinateAt(start_point);
    const auto obs_fbox = plan_passage.QueryFrenetBoxAtContour(
        st_traj.planner_object().contour());
    const auto obs_fpos = plan_passage.QueryFrenetCoordinateAt(
        st_traj.planner_object().pose().pos());
    const auto& lane_seq = plan_passage.lane_seq_info()->lane_seq;
    const auto lane_boundary_info =
        plan_passage.QueryEnclosingLaneBoundariesAtS(ego_fpos->s);
    const double half_lane_width =
        (lane_boundary_info.left.has_value() &&
         lane_boundary_info.right.has_value())
            ? (lane_boundary_info.left->lat_offset -
               lane_boundary_info.right->lat_offset) *
                  0.5
            : kDefaultHalfLaneWidth * 0.5;
    double ds = std::numeric_limits<double>::max();
    double l_pos_diff = std::numeric_limits<double>::max();
    if (obs_fbox.ok() && ego_fpos.ok() && obs_fpos.ok()) {
      ds = ego_fpos->s - input.vehicle_geom->back_edge_to_center() -
           obs_fbox->s_max;
      l_pos_diff = std::abs(ego_fpos->l - obs_fpos->l);
    }
    const auto& current_lane_info = *path_semantics.front().lane_info;

    const std::string debug = st_boundary_wd.object_id().value() +
                              " ds = " + std::to_string(ds) +
                              " safet_dist = " + std::to_string(safet_dist);

    if (ds < safet_dist || l_pos_diff > half_lane_width) {
      Log2FG::LogDataV0("ModifyRightTurnStBoundary",
                        st_boundary_wd.object_id().value() + "safet_dist");
      return std::nullopt;
    }
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  CHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      st_traj.states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kRightTurnThreshold = M_PI / 6.0;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) >
      kRightTurnThreshold) {
    return std::nullopt;
  }

  VLOG(2) << "St-boundary " << st_boundary_wd.id()
          << " is considered to be RightTurn Back VRU.";

  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() < const_speed_s) {
    return std::nullopt;
  }

  VLOG(2) << "Modify RightTurn st-boundary " << st_boundary.id();
  constexpr double kRightTurnObjectMildDecel = -0.7;

  const double RightTurn_obj_decel =
      st_traj.planner_object().pose().a() > kRightTurnObjectMildDecel
          ? kRightTurnObjectMildDecel
          : st_traj.planner_object().pose().a() - 0.3;

  auto new_st_traj = CreateSpacetimeTrajectoryByDecelerationAfterDelay(
      st_traj, st_boundary.bottom_left_point().t() - 1.0, RightTurn_obj_decel);

  auto st_boundary_output =
      st_graph.MapMovingSpacetimeObject(new_st_traj, false, false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;
  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::RIGHT_TURN_BACK_CAR;
  for (auto& st_boundary : new_st_boundaries) {
    st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
    if (st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundaries_wd.emplace_back(std::move(st_boundary),
                                    st_boundary_wd.decision_type(),
                                    st_boundary_wd.decision_reason());
    } else {
      st_boundaries_wd.emplace_back(
          std::move(st_boundary), st_boundary_wd.decision_type(),
          st_boundary_wd.decision_reason(),
          absl::StrCat(
              st_boundary_wd.decision_info(), " and keep it after modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(), 0.0, 0.0);
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult>
ModifyStBoundaryForLaneChangeBackObj(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();

  if (!input.follower_set || !st_boundary.object_id().has_value()) return res;
  const absl::flat_hash_set<std::string>& follower_set = *input.follower_set;
  if (follower_set.find(*st_boundary.object_id()) == follower_set.end()) {
    return res;
  }
  if (input.lc_stage != LaneChangeStage::LCS_EXECUTING) return res;
  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) return res;

  if (st_boundary.object_type() != StBoundaryProto::VEHICLE) return res;

  const auto& traj_id = st_boundary.traj_id();
  if (!traj_id.has_value()) return res;

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  res = ModifyLaneChangeBackObjectStBoundary(input, *input.st_graph,
                                             st_boundary_wd, *traj,
                                             input.current_v, *input.path);
  if (res.has_value()) {
    return res;
  }

  return res;
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryForOncoming(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();

  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) return res;

  CHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
        st_boundary.object_type() == StBoundaryProto::CYCLIST ||
        st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  res = ModifyOncomingStBoundary(input, *input.st_graph, st_boundary_wd, *traj,
                                 input.current_v, *input.path);
  if (res.has_value()) {
    return res;
  }

  return res;
}

std::optional<StBoundaryModificationResult> ModifyStBoundaryForRightTurn(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd,
    OpenLoopSpeedLimit* open_loop_speed_limit) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.is_protective()) return res;

  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) {
    return res;
  }

  if (st_boundary.object_type() != StBoundaryProto::CYCLIST &&
      st_boundary.object_type() != StBoundaryProto::PEDESTRIAN) {
    return res;
  }

  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  res = ModifyRightTurnStBoundary(input, *input.st_graph, st_boundary_wd, *traj,
                                  input.current_v, *input.path);
  if (res.has_value()) {
    open_loop_speed_limit->AddALimit(0.5, std::nullopt,
                                     "Rightturn cutin VRU ALimit");
    return res;
  }

  return res;
}

}  // namespace

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    OpenLoopSpeedLimit* open_loop_speed_limit) {
  CHECK_NOTNULL(input.st_graph);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.path);

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      pre_processed_st_objects;

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PreStboundaryModifierInput&, const StBoundaryWithDecision&)>(
          ModifyStBoundaryForOncoming),
      &pre_processed_st_objects, st_boundaries_wd);

  for (auto& [traj_id, traj] : pre_processed_st_objects) {
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }

  pre_processed_st_objects.clear();

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PreStboundaryModifierInput&, const StBoundaryWithDecision&)>(
          ModifyStBoundaryForLaneChangeBackObj),
      &pre_processed_st_objects, st_boundaries_wd);

  for (auto& [traj_id, traj] : pre_processed_st_objects) {
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }

  pre_processed_st_objects.clear();

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PreStboundaryModifierInput&, const StBoundaryWithDecision&,
          OpenLoopSpeedLimit*)>(ModifyStBoundaryForRightTurn),
      &pre_processed_st_objects, st_boundaries_wd, open_loop_speed_limit);

  for (auto& [traj_id, traj] : pre_processed_st_objects) {
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

}  // namespace planning
}  // namespace e2e_noa
