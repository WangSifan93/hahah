#include "speed/decision/post_st_boundary_modifier.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "object/planner_object.h"
#include "plan/trajectory_util.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/st_boundary.h"
#include "util/map_util.h"
#include "util/path_util.h"

DEFINE_bool(enable_interactive_speed_decision_draw_st_traj, false,
            "Whether enable interactive speed decision draw st trajectory.");

namespace e2e_noa::planning {
namespace {
constexpr double kEps = 1e-6;
const PiecewiseLinearFunction<double> kAvCutBrakePlf({2.0, 3.0, 5.0, 7.0},
                                                     {-0.5, -1.0, -1.5, -2.0});
const PiecewiseLinearFunction<double> kBackInteferenceBrakePlf(
    {2.0, 3.0, 5.0, 7.0}, {-0.5, -1.0, -1.5, -2.5});
const PiecewiseLinearFunction<double> kNonPrioritizedBrakePlf(
    {1.0, 2.0, 3.0, 5.0, 7.0}, {0.0, -0.5, -1.0, -1.5, -2.0});
const PiecewiseLinearFunction<double> kPriorityOvertakeBufferPLF(
    {0.0, 1.0, 3.0, 5.0}, {0.0, 0.5, 1.0, 2.0});

absl::StatusOr<std::vector<prediction::PredictedTrajectoryPoint>>
GeneratePredTrajPointsWithAccelPointList(
    absl::Span<const AccelPoint> accel_point_list,
    absl::Span<const prediction::PredictedTrajectoryPoint> pred_traj_points,
    double current_v, double current_a) {
  CHECK(!pred_traj_points.empty());

  if (accel_point_list.empty()) {
    return absl::InternalError("Accel point list empty.");
  }

  bool is_t_monotonic_increasing = true;
  for (int i = 0; i < accel_point_list.size() - 1; ++i) {
    if (accel_point_list[i].t >= accel_point_list[i + 1].t) {
      is_t_monotonic_increasing = false;
      break;
    }
  }
  if (!is_t_monotonic_increasing) {
    return absl::InternalError(
        "Time sequence in accel point list is not monotonic increasing.");
  }

  std::vector<prediction::PredictedTrajectoryPoint> new_traj_points;
  new_traj_points.reserve(pred_traj_points.size());
  const double t_step = pred_traj_points.size() > 1
                            ? pred_traj_points[1].t() - pred_traj_points[0].t()
                            : prediction::kPredictionTimeStep;
  auto prev_point = pred_traj_points.front();
  prev_point.set_v(current_v);
  if (accel_point_list.front().t < kEps) {
    current_a = accel_point_list.front().a;
  }
  if (prev_point.v() + current_a * t_step < 0.0) {
    current_a = -prev_point.v() / t_step;
  }
  prev_point.set_a(current_a);

  new_traj_points.push_back(prev_point);

  constexpr double kMinSpeed = 1e-6;
  constexpr double kMinDist = 1e-6;
  const double pred_traj_total_s = pred_traj_points.back().s();
  size_t last_step_index = 0;
  for (int i = 1; i < pred_traj_points.size(); ++i) {
    const double curr_t = i * t_step;

    double curr_a = 0.0;
    if (curr_t < accel_point_list.front().t) {
      curr_a = prev_point.a();
    } else if (curr_t >= accel_point_list.back().t) {
      curr_a = accel_point_list.back().a;
    } else {
      for (size_t i = last_step_index; i + 1 < accel_point_list.size(); ++i) {
        if (curr_t >= accel_point_list[i].t &&
            curr_t < accel_point_list[i + 1].t) {
          curr_a = accel_point_list[i].a;
          last_step_index = i;
          break;
        }
      }
    }

    const double dist = std::max(
        kMinDist, prev_point.v() * t_step + 0.5 * prev_point.a() * Sqr(t_step));
    const double curr_s = prev_point.s() + dist;
    prediction::PredictedTrajectoryPoint curr_point;
    if (curr_s >= pred_traj_total_s) {
      const auto prev_path_point =
          GetPathPointFromPredictedTrajectoryPoint(prev_point);
      const auto curr_path_point =
          GetPathPointAlongCircle(prev_path_point, dist);
      AssignSpatialInfoFromPathPoint(curr_path_point, &curr_point);
    } else {
      curr_point = QueryTrajectoryPointByS(pred_traj_points, curr_s);
    }

    curr_point.set_t(curr_t);
    curr_point.set_v(
        std::max(kMinSpeed, prev_point.v() + t_step * prev_point.a()));
    if (curr_point.v() + curr_a * t_step < 0.0) {
      curr_a = -curr_point.v() / t_step;
    }
    curr_point.set_a(curr_a);

    new_traj_points.push_back(curr_point);
    prev_point = std::move(curr_point);
  }

  return new_traj_points;
}

StBoundaryModificationResult ModifyStBoundaryViaModificationInfo(
    const StGraph& st_graph, const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj,
    const StBoundaryModificationInfo& modification_info) {
  CHECK(modification_info.decision != StBoundaryProto::UNKNOWN);

  const bool is_interactive_modifier = modification_info.modifier_type ==
                                       StBoundaryModifierProto::LON_INTERACTIVE;

  const auto modifier_type = StBoundaryModifierProto::ModifierType_Name(
      modification_info.modifier_type);

  std::string decision_info;
  StBoundaryProto::DecisionReason decision_reason =
      StBoundaryProto::UNKNOWN_REASON;
  if (modification_info.is_decision_changed) {
    decision_info = absl::StrCat("decision changed by ", modifier_type);
    CHECK(is_interactive_modifier);
    decision_reason = StBoundaryProto::INTERACTIVE_DECIDER;
  } else {
    decision_info =
        absl::StrCat(st_boundary_wd.decision_info(),
                     " and keep it after modified by ", modifier_type);
    decision_reason = st_boundary_wd.decision_reason();
  }

  auto new_st_traj = CreateSpacetimeTrajectoryWithAccelPointList(
      modification_info.accel_point_list, st_traj);

  bool generate_lane_change_gap = false;
  if (st_boundary_wd.raw_st_boundary()->is_protective() &&
      st_boundary_wd.raw_st_boundary()->protection_type() ==
          StBoundaryProto::LANE_CHANGE_GAP) {
    generate_lane_change_gap = true;
  }

  auto st_boundary_output = st_graph.MapMovingSpacetimeObject(
      new_st_traj, generate_lane_change_gap, false, nullptr);
  auto& new_st_boundaries = st_boundary_output.st_boundaries;

  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  if ((st_boundary_wd.raw_st_boundary()->is_protective() &&
       st_boundary_wd.raw_st_boundary()->protection_type() ==
           StBoundaryProto::LANE_CHANGE_GAP) ||
      !st_boundary_wd.raw_st_boundary()->is_protective()) {
    st_boundaries_wd.reserve(new_st_boundaries.size());
    for (auto& st_boundary : new_st_boundaries) {
      DLOG(INFO) << "post new not protective: " << st_boundary->id()
                 << " proc: " << st_boundary->protection_type();
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

std::optional<StBoundaryModificationResult> PostModifyStBoundary(
    const PostStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();

  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return std::nullopt;
  }
  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) {
    return std::nullopt;
  }

  const auto& traj_id = st_boundary.traj_id();
  CHECK(traj_id.has_value());

  const auto* modification_info =
      FindOrNull(*input.modification_info_map, *traj_id);
  if (modification_info == nullptr) return std::nullopt;

  const auto* traj =
      CHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  return ModifyStBoundaryViaModificationInfo(*input.st_graph, st_boundary_wd,
                                             *traj, *modification_info);
}

}  // namespace

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryWithAccelPointList(
    absl::Span<const AccelPoint> accel_point_list,
    const SpacetimeObjectTrajectory& st_object) {
  auto generated_pred_traj = GeneratePredTrajPointsWithAccelPointList(
      accel_point_list, st_object.trajectory().points(),
      st_object.planner_object().pose().v(),
      st_object.planner_object().pose().a());
  auto new_pred_traj = st_object.trajectory();

  if (generated_pred_traj.ok()) {
    *new_pred_traj.mutable_points() = std::move(*generated_pred_traj);
  } else {
    LOG(WARNING) << "Generate new pred traj for " << st_object.traj_id()
                 << " failed: " << generated_pred_traj.status().message()
                 << " Use origin pred traj.";
    *new_pred_traj.mutable_points() = st_object.trajectory().points();
  }

  return st_object.CreateTrajectoryMutatedInstance(std::move(new_pred_traj));
}

void PostModifyStBoundaries(
    const PostStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects) {
  CHECK_NOTNULL(input.st_graph);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.modification_info_map);

  const auto& modification_info_map = *input.modification_info_map;

  for (const auto& [id, _] : modification_info_map) {
    DCHECK(processed_st_objects->find(id) == processed_st_objects->end())
        << " processed_st_objects contains " << id
        << " in modification_info_map";
  }

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      interactive_processed_st_objects;

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PostStboundaryModifierInput&, const StBoundaryWithDecision&)>(
          PostModifyStBoundary),
      &interactive_processed_st_objects, st_boundaries_wd);

  for (auto& [traj_id, traj] : interactive_processed_st_objects) {
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

}  // namespace e2e_noa::planning
