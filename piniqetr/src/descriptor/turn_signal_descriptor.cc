#include "descriptor/turn_signal_descriptor.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "affine_transformation.pb.h"
#include "container/strong_int.h"
#include "maps/lane_point.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "math/vec.h"
#include "plan/planner_defs.h"

namespace e2e_noa {
namespace planning {

namespace {

using TurnType = ad_e2e::planning::TurnType;
using ReactionRule = ad_e2e::planning::LaneInteraction::ReactionRule;
using GeometricConfiguration =
    ad_e2e::planning::LaneInteraction::GeometricConfiguration;

constexpr double kPreviewDirectionDist = 20.0;
constexpr double kPreviewMergeForkTime = 2.5;
constexpr double kPreviewMergeForkMinDist = 30.0;

bool HasReachedDestination(const PlannerSemanticMapManager& psmm,
                           const mapping::LanePath& lane_path) {
  return false;
}

bool HasMapDictatedTurnSignal(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePath& lane_path,
                              TurnSignal* signal) {
  return false;
}

std::optional<TurnSignal> ComputeDirectionSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  e2e_noa::mapping::ElementId preview_lane_id =
      lane_path.ArclengthToLanePoint(kPreviewDirectionDist * 1.6667).lane_id();
  ad_e2e::planning::LaneConstPtr preview_lane_ptr =
      psmm.map_ptr()->GetLaneById(preview_lane_id);
  if (preview_lane_ptr == nullptr) {
    return std::nullopt;
  }
  if (preview_lane_ptr->turn_type() == TurnType::U_TURN) {
    return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
  }
  preview_lane_id =
      lane_path.ArclengthToLanePoint(kPreviewDirectionDist).lane_id();
  ad_e2e::planning::LaneConstPtr current_lane_ptr = psmm.map_ptr()->GetLaneById(
      lane_path.ArclengthToLanePoint(0.0).lane_id());

  if (preview_lane_ptr == nullptr) {
    return std::nullopt;
  } else {
    if (preview_lane_ptr->turn_type() == TurnType::NO_TURN &&
        current_lane_ptr != nullptr) {
      switch (current_lane_ptr->turn_type()) {
        case TurnType::NO_TURN:
          return std::nullopt;
        case TurnType::LEFT_TURN:
          return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
        case TurnType::RIGHT_TURN:
          return std::make_optional<TurnSignal>(TURN_SIGNAL_RIGHT);
        case TurnType::U_TURN:
          return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
      }
    }
    if (preview_lane_ptr->turn_type() == TurnType::LEFT_TURN)
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
    if (preview_lane_ptr->turn_type() == TurnType::RIGHT_TURN)
      return std::make_optional<TurnSignal>(TURN_SIGNAL_RIGHT);
    if (preview_lane_ptr->turn_type() == TurnType::U_TURN)
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
  }
  return std::nullopt;
}

absl::StatusOr<TurnSignal> DetectSignalFromAdjacentLane(
    const PlannerSemanticMapManager& psmm,
    const ad_e2e::planning::Lane& lane_info,
    const ad_e2e::planning::Lane& target_lane_info) {
  const auto target_sec_id = target_lane_info.section_id();
  const Vec2d target_end_point = target_lane_info.LerpPointFromFraction(1.0);

  double min_sqr_dist = DBL_MAX;
  Vec2d nearest_end_point;
  for (const auto next_lane_id : lane_info.next_lane_ids()) {
    if (next_lane_id == target_lane_info.id()) continue;
    const auto next_lane_info = psmm.map_ptr()->GetLaneById(next_lane_id);
    if (!next_lane_info) break;
    if (next_lane_info->section_id() != target_sec_id) continue;

    if (!next_lane_info->IsVirtual()) {
      const Vec2d next_end_point = next_lane_info->LerpPointFromFraction(1.0);
      const double sqr_dist = next_end_point.DistanceSquareTo(target_end_point);
      if (sqr_dist < min_sqr_dist) {
        min_sqr_dist = sqr_dist;
        nearest_end_point = next_end_point;
      }
    }
  }
  if (min_sqr_dist < DBL_MAX) {
    const Vec2d fork_point = lane_info.LerpPointFromFraction(1.0);
    return (nearest_end_point - fork_point)
                       .CrossProd(target_end_point - fork_point) > 0.0
               ? TURN_SIGNAL_LEFT
               : TURN_SIGNAL_RIGHT;
  }
  return absl::NotFoundError("No neighboring real lane found.");
}

absl::StatusOr<TurnSignal> ComputeForkSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id, double ego_v,
    Vec2d ego_pose) {
  const double preview_dist =
      std::max(kPreviewMergeForkMinDist, ego_v * kPreviewMergeForkTime);
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();
  const auto stop_lane_id = redlight_lane_id.has_value()
                                ? *redlight_lane_id
                                : mapping::kInvalidElementId;

  for (int i = 0; i < lane_path.lane_ids().size(); ++i) {
    const auto lane_id = lane_path.lane_id(i);
    if (lane_id == far_lane_id) break;

    const auto next_lane_id = lane_path.lane_id(i + 1);
    if (next_lane_id == stop_lane_id) break;

    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.next_lane_ids().size() < 2) continue;

    SMM_ASSIGN_LANE_OR_ERROR(next_lane_info, psmm, next_lane_id);
    if (next_lane_info.IsVirtual()) {
      const auto real_lane_or =
          DetectSignalFromAdjacentLane(psmm, lane_info, next_lane_info);
      if (real_lane_or.ok()) {
        return *real_lane_or;
      }
    }

    const auto& map = psmm.map_ptr();
    const auto& lane = map->GetLaneById(lane_id);
    const auto& next_lane = map->GetLaneById(next_lane_id);
    if (lane && next_lane) {
      if (!lane->is_split() && !next_lane->is_split()) {
        break;
      }
      if ((!lane->junction_id().empty()) ||
          (!next_lane->junction_id().empty())) {
        break;
      }
      constexpr double kQueryDistAfterForkPoint = 30.0;
      const double query_fraction = std::min(
          1.0, kQueryDistAfterForkPoint / next_lane_info.curve_length());
      const Vec2d next_query_point =
          next_lane_info.LerpPointFromFraction(query_fraction);
      double next_point_s, next_point_l;

      lane->center_line().GetProjection(next_query_point, &next_point_s,
                                        &next_point_l);

      double ego_s, ego_l;
      lane->center_line().GetProjection(ego_pose, &ego_s, &ego_l);
      double l = next_point_l - ego_l;

      constexpr double kForkLatDistThreshold = 1.0;
      if (std::abs(l) > kForkLatDistThreshold) {
        return l > 0 ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT;
      }
    }
  }

  return absl::CancelledError("No need to turn fork signal.");
}

absl::StatusOr<TurnSignal> TurnOnMergeSignal(
    const PlannerSemanticMapManager& psmm,
    const ad_e2e::planning::Lane& lane_info) {
  mapping::ElementId other_id = mapping::kInvalidElementId;
  for (const auto interaction : lane_info.interactions()) {
    if (interaction.geometric_configuration != GeometricConfiguration::MERGE) {
      continue;
    }
    if (interaction.reaction_rule == ReactionRule::YIELD_MERGE ||
        interaction.reaction_rule == ReactionRule::YIELD) {
      other_id = mapping::ElementId(interaction.other_lane_id);
      break;
    }
  }
  if (other_id == mapping::kInvalidElementId) {
    return absl::NotFoundError(
        "Can not find the merging lane, please check the map.");
  }

  constexpr double kQueryDistBeforeMergePoint = 10.0;
  const double query_fraction =
      std::max(0.0, (lane_info.curve_length() - kQueryDistBeforeMergePoint) /
                        lane_info.curve_length());
  const Vec2d this_lane_pos = lane_info.LerpPointFromFraction(query_fraction);

  SMM_ASSIGN_LANE_OR_ERROR(other_lane_info, psmm, other_id);
  const Vec2d other_lane_tangent = other_lane_info.GetTangent(query_fraction);
  const Vec2d other_lane_pos =
      other_lane_info.LerpPointFromFraction(query_fraction);

  return other_lane_tangent.CrossProd(this_lane_pos - other_lane_pos) < 0.0
             ? TURN_SIGNAL_LEFT
             : TURN_SIGNAL_RIGHT;
}

absl::StatusOr<TurnSignal> EnableMergeSignalWorkaround(
    const PlannerSemanticMapManager& psmm,
    const ad_e2e::planning::Lane& lane_info) {
  if (!psmm.map_ptr())
    return absl::CancelledError("No map ptr for decide merge sigs.");
  ad_e2e::planning::LaneConstPtr merged_lane;
  for (auto merged_lane_id : lane_info.next_lane_ids()) {
    merged_lane = psmm.map_ptr()->GetLaneById(merged_lane_id);
    if (merged_lane) break;
  }

  if (!merged_lane) return absl::CancelledError("No merged targe lane.");

  const ad_e2e::planning::LaneConstPtr l_lane =
      psmm.map_ptr()->GetLaneById(lane_info.left_lane_id());
  if (l_lane) {
    for (auto& other_next : l_lane->next_lane_ids()) {
      const ad_e2e::planning::LaneConstPtr other_merge_lane =
          psmm.map_ptr()->GetLaneById(other_next);
      if (other_merge_lane && (other_merge_lane->id() == merged_lane->id())) {
        return TURN_SIGNAL_LEFT;
      }
    }
  }

  const ad_e2e::planning::LaneConstPtr r_lane =
      psmm.map_ptr()->GetLaneById(lane_info.right_lane_id());
  if (r_lane) {
    for (auto& other_next : r_lane->next_lane_ids()) {
      const ad_e2e::planning::LaneConstPtr other_merge_lane =
          psmm.map_ptr()->GetLaneById(other_next);
      if (other_merge_lane && (other_merge_lane->id() == merged_lane->id())) {
        return TURN_SIGNAL_RIGHT;
      }
    }
  }

  return absl::CancelledError("No need to turn on merge signal.");
}

absl::StatusOr<TurnSignal> ComputeMergeSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double ego_v) {
  const double preview_dist =
      std::max(kPreviewMergeForkMinDist, ego_v * kPreviewMergeForkTime);
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();

  for (const auto lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.is_merge() && lane_info.junction_id().empty()) {
      return EnableMergeSignalWorkaround(psmm, lane_info);
    }
    if (lane_id == far_lane_id) break;
  }
  return absl::CancelledError("No need to turn on merge signal.");
}

}  // namespace

TurnSignalResult DecideTurnSignal(
    const PlannerSemanticMapManager& psmm, TurnSignal pre_lane_change_signal,
    const mapping::LanePath& current_lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id,
    const LaneChangeStateProto& lc_state, const PlanPassage& plan_passage,
    const FrenetBox& ego_sl_box, const std::optional<PNPInfos>& pnp_infos,
    const TurnSignalResult& planned_result, const PoseProto& ego_pose,
    bool if_continue_lc, const ad_e2e::planning::LaneSeqInfoPtr& lane_seq_info,
    TurnSignal turn_type_signal) {
  constexpr double kStopSignalDistanceAhead = 30.0;
  if (HasReachedDestination(psmm, current_lane_path) &&
      current_lane_path.length() < kStopSignalDistanceAhead) {
    return {TURN_SIGNAL_RIGHT, PULL_OVER_TURN_SIGNAL};
  }

  TurnSignal map_signal;
  if (HasMapDictatedTurnSignal(psmm, current_lane_path, &map_signal)) {
    return {map_signal, MAP_DICTATED_TURN_SIGNAL};
  }

  const double ego_v =
      Hypot(ego_pose.vel_smooth().x(), ego_pose.vel_smooth().y());
  std::optional<TurnSignal> direction_signal = std::nullopt;
  direction_signal = ComputeDirectionSignal(psmm, current_lane_path);

  if (direction_signal.has_value() && ego_v > -0.1) {
    return {direction_signal.value(), TURNING_TURN_SIGNAL};
  }

  if (lc_state.stage() != LCS_NONE) {
    if (if_continue_lc) {
      return {lc_state.lc_left() ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT,
              CONTINUE_LANE_CHANGE_SIGNAL};
    }
    return {lc_state.lc_left() ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT,
            LANE_CHANGE_TURN_SIGNAL};
  }

  if (!pnp_infos && planned_result.signal != TurnSignal::TURN_SIGNAL_NONE) {
    return planned_result;
  }

  const auto merge_signal = ComputeMergeSignal(psmm, current_lane_path, ego_v);
  if (merge_signal.ok()) {
    return {merge_signal.value(), MERGE_TURN_SIGNAL};
  }
  const auto fork_signal = ComputeForkSignal(
      psmm, current_lane_path, redlight_lane_id, ego_v,
      Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));
  if (fork_signal.ok()) {
    return {fork_signal.value(), FORK_TURN_SIGNAL};
  }

  if (!direction_signal.has_value() && ego_v > -0.1) {
    return {turn_type_signal, TURNING_TURN_SIGNAL};
  }

  if (!pnp_infos && pre_lane_change_signal != TurnSignal::TURN_SIGNAL_NONE) {
    return {pre_lane_change_signal, PREPARE_LANE_CHANGE_TURN_SIGNAL};
  }

  return {TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
}

}  // namespace planning
}  // namespace e2e_noa
