#include "speed/gridded_svt_graph.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "speed/speed_point.h"
#include "speed/svt_point.h"
#include "speed_planning_params.pb.h"
#include "util/status_macros.h"

DEFINE_bool(enable_sampling_dp_reference_speed, false,
            "True to penalize sampling dp result towards default cruise speed");

namespace e2e_noa::planning {

namespace {
constexpr double kEps = 1.0e-6;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct BestEndPoint {
  BestEndPoint() = default;
  BestEndPoint(const SvtGraphPoint* p, double c) : point(p), cost(c) {}
  const SvtGraphPoint* point = nullptr;
  double cost = kInf;
};

absl::StatusOr<int> FindOptimalCandidate(
    absl::Span<const SvtGraphPointRef> candidate_points) {
  if (candidate_points.empty()) {
    return absl::FailedPreconditionError("Candidate points empty.");
  }
  const auto optimal_point = std::min_element(
      candidate_points.begin(), candidate_points.end(),
      [](const SvtGraphPointRef& point1, const SvtGraphPointRef& point2) {
        return point1->total_cost() < point2->total_cost();
      });
  return optimal_point - candidate_points.begin();
}

std::vector<std::vector<double>> GenerateAccMatrix(
    double max_acceleration, double max_deceleration,
    double coarse_sampling_decel_thres, double coarse_sampling_num,
    double unit_acc, double total_length_s, int dimension_t, int unit_t,
    double cur_v) {
  constexpr int kKeepUnitAccLayerIndex = 2;
  constexpr int kNoNeedToAccelerateRapidlyLayerIndex = 3;
  constexpr int kNoNeedToDecelerateHardLayerIndex = 5;
  constexpr double kGentleAccelerateAcc = 1.0;
  constexpr double kGentleDecelerateAcc = -2.0;
  constexpr double kMaxAccStep = 1.0;
  constexpr double kAccStepGain = 0.1;
  constexpr double kMaxSampleGapToMaxValue = 0.3;

  std::vector<std::vector<double>> acc_matrix;
  acc_matrix.reserve(dimension_t);
  const int acc_sample_max_size =
      FloorToInt((max_acceleration - max_deceleration) / unit_acc) + 2;
  for (int i = 0; i < dimension_t - 1; ++i) {
    if (i > kNoNeedToAccelerateRapidlyLayerIndex) {
      max_acceleration = kGentleAccelerateAcc;
    }
    if (i > kNoNeedToDecelerateHardLayerIndex) {
      max_deceleration = kGentleDecelerateAcc;
    }

    std::vector<double> cur_layer_acc;
    cur_layer_acc.reserve(acc_sample_max_size);
    double acc_step = unit_acc;
    if (i > kKeepUnitAccLayerIndex) {
      acc_step = std::min(
          acc_step + (i - kKeepUnitAccLayerIndex) * kAccStepGain, kMaxAccStep);
    }

    for (double acc = 0.0; acc <= max_acceleration; acc += acc_step) {
      cur_layer_acc.push_back(acc);
    }
    if (max_acceleration - cur_layer_acc.back() > kMaxSampleGapToMaxValue) {
      cur_layer_acc.push_back(max_acceleration);
    } else {
      cur_layer_acc.back() = max_acceleration;
    }
    const double acc_coarse_step = std::max(
        (coarse_sampling_decel_thres - max_deceleration) / coarse_sampling_num,
        acc_step);
    for (double acc = -acc_step, acc_thres = coarse_sampling_decel_thres + kEps;
         acc >= max_deceleration; acc -= acc_step) {
      if (acc < acc_thres) {
        acc_step = acc_coarse_step;
      }
      cur_layer_acc.push_back(acc);
    }
    if (cur_layer_acc.back() - max_deceleration > kMaxSampleGapToMaxValue) {
      cur_layer_acc.push_back(max_deceleration);
    } else {
      cur_layer_acc.back() = max_deceleration;
    }
    acc_matrix.push_back(std::move(cur_layer_acc));
  }

  const double stoppable_decel = -cur_v / unit_t;
  if (stoppable_decel > max_deceleration) {
    acc_matrix.front().push_back(stoppable_decel);
  }

  return acc_matrix;
}
}  // namespace

GriddedSvtGraph::GriddedSvtGraph(
    const StGraphData* st_graph_data, double init_v, double init_a,
    double max_acceleration, double max_deceleration,
    const SpeedPlanningParamsProto* speed_planning_params, double speed_cap,
    std::vector<StBoundaryWithDecision*> st_boundaries_wd,
    const SpacetimeTrajectoryManager& st_traj_mgr)
    : st_graph_data_(st_graph_data),
      sorted_st_boundaries_with_decision_(std::move(st_boundaries_wd)),
      init_v_(init_v),
      init_a_(init_a),
      speed_planning_params_(CHECK_NOTNULL(speed_planning_params)),
      dp_params_(&speed_planning_params->sampling_dp_speed_params()),
      dp_svt_cost_(speed_planning_params_, st_graph_data_->total_time(),
                   st_graph_data_->path_length(),
                   &sorted_st_boundaries_with_decision_, st_traj_mgr),
      total_duration_t_(st_graph_data_->total_time()),
      unit_t_(dp_params_->unit_t()),
      total_length_s_(st_graph_data_->path_length()),
      total_length_v_(std::max(init_v_, speed_cap) +
                      dp_params_->speed_exceeding_cap()),
      unit_v_(dp_params_->unit_v()),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration),
      st_traj_mgr_(st_traj_mgr) {
  int protective_idx = 0;
  for (int i = 0; i < sorted_st_boundaries_with_decision_.size(); ++i) {
    if (sorted_st_boundaries_with_decision_[i]
            ->raw_st_boundary()
            ->is_protective()) {
      if (protective_idx != i) {
        std::swap(sorted_st_boundaries_with_decision_[i],
                  sorted_st_boundaries_with_decision_[protective_idx]);
      }
      ++protective_idx;
    }
  }
  dp_svt_cost_.SetFollowDistanceRelSpeedPlf(PiecewiseLinearFunctionFromProto(
      speed_planning_params->follow_distance_rel_speed_plf()));
}

absl::Status GriddedSvtGraph::InitLayers() {
  unit_s_ = std::max(total_length_s_ / dp_params_->s_dimension_size(), 0.15);
  unit_inv_t_ = 1.0 / unit_t_;
  unit_inv_s_ = 1.0 / unit_s_;
  unit_inv_v_ = 1.0 / unit_v_;

  dimension_t_ = CeilToInteger(total_duration_t_ * unit_inv_t_) + 1;
  dimension_grid_v_ = CeilToInteger(total_length_v_ * unit_inv_v_);
  dimension_grid_s_ = CeilToInteger(total_length_s_ * unit_inv_s_);

  if (unit_t_ < kEps) {
    return absl::InternalError("unit_t is smaller than the kEps.");
  }
  if (unit_v_ < kEps) {
    return absl::InternalError("unit_v is smaller than the kEps.");
  }
  if (unit_s_ < kEps) {
    return absl::InternalError("unit_s is smaller than the kEps.");
  }

  if (dimension_grid_s_ < 1 || dimension_grid_v_ < 1 || dimension_t_ < 1) {
    return absl::InternalError(absl::StrFormat(
        "Sampling-dp discretized space dimension error, s:%d, v:%d, t:%d",
        dimension_grid_s_, dimension_grid_v_, dimension_t_));
  }

  layers_ =
      std::vector<std::vector<std::vector<SvtGraphPointRef>>>(dimension_t_);
  for (auto& layer : layers_) {
    layer.resize(dimension_grid_s_);
    for (auto& s : layer) {
      s.resize(dimension_grid_v_);
    }
  }

  s_knots_.clear();
  s_knots_.reserve(dimension_grid_s_ + 1);
  v_knots_.clear();
  v_knots_.reserve(dimension_grid_v_ + 1);
  t_knots_.clear();
  t_knots_.reserve(dimension_t_);

  double cur_s = 0.0;
  for (int i = 0; i < dimension_grid_s_; ++i, cur_s += unit_s_) {
    s_knots_.push_back(cur_s);
  }
  s_knots_.push_back(total_length_s_);

  double cur_v = 0.0;
  for (int i = 0; i < dimension_grid_v_; ++i, cur_v += unit_v_) {
    v_knots_.push_back(cur_v);
  }
  v_knots_.push_back(total_length_v_);

  double cur_t = 0.0;
  for (int i = 0; i < dimension_t_ - 1; ++i, cur_t += unit_t_) {
    t_knots_.push_back(cur_t);
  }
  t_knots_.push_back(total_duration_t_);

  acc_matrix_ = GenerateAccMatrix(max_acceleration_, max_deceleration_,
                                  dp_params_->coarse_sampling_decel_thres(),
                                  dp_params_->coarse_sampling_num(),
                                  dp_params_->unit_acc(), total_length_s_,
                                  dimension_t_, unit_t_, init_v_);

  return absl::OkStatus();
}

absl::StatusOr<SvGridIndexSet> GriddedSvtGraph::SearchAndReturnFinalLayerPoints(
    WorkerThreadManager* thread_pool) {
  const double cruise_speed = st_graph_data_->cruise_speed();
  const auto& speed_limit_provider = st_graph_data_->speed_limit_provider();

  RETURN_IF_ERROR(InitLayers());

  SvGridIndexSet cur_layer_indices;
  SvGridIndexSet next_layer_indices;
  next_layer_indices.reserve(dimension_grid_s_ * dimension_grid_v_);

  std::vector<std::vector<std::vector<SvtGraphPointRef>>>
      next_layer_candidate_points(dimension_grid_s_);
  for (auto& layer : next_layer_candidate_points) {
    layer.resize(dimension_grid_v_);
  }

  constexpr int kPointsNumPerGrid = 15;
  for (auto& row : next_layer_candidate_points) {
    for (auto& grid : row) {
      grid.reserve(kPointsNumPerGrid);
    }
  }

  double cur_t = 0.0;

  for (size_t i = 0; i + 1 < layers_.size(); ++i, cur_t += unit_t_) {
    auto& cur_layer = layers_[i];
    auto& next_layer = layers_[i + 1];
    const double next_t = cur_t + unit_t_;
    const int next_point_index_t = i + 1;
    next_layer_indices.clear();

    if (i == 0) {
      if (init_v_ > total_length_v_) {
        return absl::InternalError(
            "Init point speed exceeds total length speed.");
      }
      const int init_point_grid_index_v = FloorToInt(init_v_ * unit_inv_v_);
      layers_[0][0][init_point_grid_index_v] = std::make_unique<SvtGraphPoint>(
          0, init_point_grid_index_v, 0, SvtPoint(0.0, init_v_, 0.0));
      layers_[0][0][init_point_grid_index_v]->set_total_cost(0.0);

      cur_layer_indices.emplace(0, init_point_grid_index_v);

      const auto st_boundary_decisions =
          dp_svt_cost_.GetStBoundaryDecisionsForInitPoint(
              *layers_[0][0][init_point_grid_index_v]);
      layers_[0][0][init_point_grid_index_v]->UpdateStBoundaryDecisions(
          st_boundary_decisions);
    }

    for (auto& row : next_layer_candidate_points) {
      for (auto& grid : row) {
        grid.clear();
      }
    }

    for (const auto& index : cur_layer_indices) {
      auto& sv_grid = cur_layer[index.grid_index_s_][index.grid_index_v_];
      if (sv_grid == nullptr) {
        return absl::InternalError("Current sv grid not existing.");
      }
      ExpandToNextLayer(i, cur_t, next_t, next_point_index_t,
                        speed_limit_provider, cruise_speed, init_v_,
                        sv_grid.get(), &next_layer_candidate_points,
                        &next_layer_indices);
    }

    for (const auto& next_index : next_layer_indices) {
      auto& grid_candidate_points =
          next_layer_candidate_points[next_index.grid_index_s_]
                                     [next_index.grid_index_v_];
      auto& optimal_candidate =
          next_layer[next_index.grid_index_s_][next_index.grid_index_v_];
      ASSIGN_OR_RETURN(auto find_res,
                       FindOptimalCandidate(grid_candidate_points));
      optimal_candidate = std::move(grid_candidate_points[find_res]);
      optimal_candidate->pre_point()->set_next_point(&*optimal_candidate);
    }

    cur_layer_indices.swap(next_layer_indices);
  }

  return cur_layer_indices;
}

absl::Status GriddedSvtGraph::FindOptimalPreliminarySpeed(
    SpeedVector* preliminary_speed, SamplingDpDebugProto* sampling_dp_debug,
    WorkerThreadManager* thread_pool) {
  ASSIGN_OR_RETURN(const auto final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));

  auto preliminary_speed_with_cost =
      GetSpeedProfileAndCompleteStBoundariesWithDecision(final_layer_indices);
  if (!preliminary_speed_with_cost.ok()) {
    return preliminary_speed_with_cost.status();
  } else {
    *preliminary_speed = preliminary_speed_with_cost->preliminary_speed;
  }

  return absl::OkStatus();
}

absl::Status GriddedSvtGraph::FindOptimalPreliminarySpeedWithCost(
    PreliminarySpeedWithCost* preliminary_speed_with_cost,
    SamplingDpDebugProto* sampling_dp_debug, WorkerThreadManager* thread_pool) {
  ASSIGN_OR_RETURN(const SvGridIndexSet final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));
  ASSIGN_OR_RETURN(
      *preliminary_speed_with_cost,
      GetSpeedProfileAndCompleteStBoundariesWithDecision(final_layer_indices));
  return absl::OkStatus();
}

absl::Status GriddedSvtGraph::GenerateSamplingDpSpeedProfileCandidateSet(
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
    SamplingDpDebugProto* sampling_dp_debug,
    InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
    WorkerThreadManager* thread_pool) {
  ASSIGN_OR_RETURN(const SvGridIndexSet final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));
  *candidate_speed_profiles =
      SampleSpeedProfilesFromSamplingDp(final_layer_indices);
  return absl::OkStatus();
}

std::vector<SvtGraphPointRef> GriddedSvtGraph::ExpandByConstAccModel(
    int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
    const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
    double init_speed, SvtGraphPoint* cur_point) {
  using StBoundaryDecision = SvtGraphPoint::StBoundaryDecision;

  std::vector<SvtGraphPointRef> candidate_points;
  candidate_points.reserve(acc_matrix_[cur_layer_index].size());
  const double cur_s = cur_point->point().s();
  const double cur_v = cur_point->point().v();
  const auto prev_speed_limit_or =
      speed_limit_provider.GetSpeedLimitByTimeAndS(cur_t, cur_s);
  double prev_speed_limit = std::numeric_limits<double>::max();
  if (prev_speed_limit_or.has_value()) {
    prev_speed_limit = *prev_speed_limit_or;
  }
  for (const double acc : acc_matrix_[cur_layer_index]) {
    const double next_s = cur_s + cur_v * unit_t_ + 0.5 * acc * Sqr(unit_t_);
    const double next_v = cur_v + acc * unit_t_;

    if (next_v > (total_length_v_ - kEps) ||
        next_s > (total_length_s_ - kEps) || next_v < 0.0 || next_s < 0.0 ||
        next_s < cur_s) {
      continue;
    }

    const int next_point_grid_index_s = FloorToInt(next_s * unit_inv_s_);
    const int next_point_grid_index_v = FloorToInt(next_v * unit_inv_v_);

    auto& next_point =
        candidate_points.emplace_back(std::make_unique<SvtGraphPoint>(
            next_point_grid_index_s, next_point_grid_index_v,
            next_point_index_t, SvtPoint(next_s, next_v, next_t)));
    next_point->set_spatial_potential_cost(
        dp_svt_cost_.GetSpatialPotentialCost(next_s));
    next_point->set_vertex_cost(next_point->spatial_potential_cost());

    const auto curr_speed_limit_or =
        speed_limit_provider.GetSpeedLimitByTimeAndS(next_t, next_s);
    double curr_speed_limit = std::numeric_limits<double>::max();
    if (curr_speed_limit_or.has_value()) {
      curr_speed_limit = *curr_speed_limit_or;
    }

    prev_speed_limit = next_point_index_t == 1
                           ? curr_speed_limit
                           : std::min(prev_speed_limit, curr_speed_limit);

    const double average_speed = 0.5 * (cur_v + next_v);
    const double speed_limit_cost =
        dp_svt_cost_.GetSpeedLimitCost(average_speed, prev_speed_limit);
    const double reference_speed_cost =
        dp_svt_cost_.GetReferenceSpeedCost(average_speed, cruise_speed);
    const double accel_cost = dp_svt_cost_.GetAccelCost(acc);
    double object_cost = 0.0;
    std::vector<StBoundaryDecision> st_boundary_decisions;
    st_boundary_decisions.reserve(sorted_st_boundaries_with_decision_.size());
    dp_svt_cost_.GetStBoundaryCostAndDecisions(*cur_point, *next_point,
                                               init_speed, &object_cost,
                                               &st_boundary_decisions);
    const double edge_cost =
        speed_limit_cost + reference_speed_cost + accel_cost + object_cost;
    const double total_cost =
        next_point->vertex_cost() + edge_cost + cur_point->total_cost();

    next_point->set_speed_limit_cost(speed_limit_cost);
    next_point->set_reference_speed_cost(reference_speed_cost);
    next_point->set_accel_cost(accel_cost);
    next_point->set_object_cost(object_cost);
    next_point->set_edge_cost(edge_cost);
    next_point->set_total_cost(total_cost);
    next_point->set_pre_point(cur_point);
    next_point->set_acc_from_pre_point(acc);
    next_point->UpdateStBoundaryDecisions(st_boundary_decisions);
  }

  return candidate_points;
}

void GriddedSvtGraph::ExpandToNextLayer(
    int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
    const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
    double init_speed, SvtGraphPoint* cur_point,
    std::vector<std::vector<std::vector<SvtGraphPointRef>>>*
        next_layer_candidate_points,
    SvGridIndexSet* index_range) {
  auto candidate_points = ExpandByConstAccModel(
      cur_layer_index, cur_t, next_t, next_point_index_t, speed_limit_provider,
      cruise_speed, init_speed, cur_point);

  for (auto& candidate_point : candidate_points) {
    const int index_s = candidate_point->grid_index_s();
    const int index_v = candidate_point->grid_index_v();
    index_range->emplace(index_s, index_v);
    (*next_layer_candidate_points)[index_s][index_v].push_back(
        std::move(candidate_point));
  }
}

absl::StatusOr<PreliminarySpeedWithCost>
GriddedSvtGraph::GetSpeedProfileAndCompleteStBoundariesWithDecision(
    const SvGridIndexSet& final_layer_index_range) {
  double min_cost = kInf;
  const SvtGraphPoint* best_end_point = nullptr;

  for (const auto& final_layer_index : final_layer_index_range) {
    const auto& end_point = layers_.back()[final_layer_index.grid_index_s_]
                                          [final_layer_index.grid_index_v_];
    if (end_point->total_cost() < min_cost) {
      best_end_point = &*end_point;
      min_cost = end_point->total_cost();
    }
  }

  for (const auto& layer : layers_) {
    const auto& end_s_row = layer.back();
    for (const auto& end_point : end_s_row) {
      if (end_point == nullptr || !end_point->next_point().empty()) {
        continue;
      }
      if (end_point->total_cost() < min_cost) {
        best_end_point = &*end_point;
        min_cost = end_point->total_cost();
      }
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    return absl::InternalError(msg);
  }

  VLOG(2) << "Best end point cost: " << best_end_point->total_cost();
  std::vector<SpeedPoint> speed_profile;
  speed_profile.reserve(dimension_t_);
  const SvtGraphPoint* cur_point = best_end_point;

  while (cur_point != nullptr) {
    VLOG(3) << "Time: " << cur_point->point().t();
    VLOG(3) << "S: " << cur_point->point().s();
    VLOG(3) << "V: " << cur_point->point().v();

    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_v(cur_point->point().v());
    speed_point.set_a(cur_point->acc_from_pre_point());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(std::move(speed_point));
    for (auto* st_boundary_with_decision :
         sorted_st_boundaries_with_decision_) {
      if (st_boundary_with_decision->decision_type() !=
          StBoundaryProto::UNKNOWN) {
        continue;
      }
      const auto decision =
          cur_point->GetStBoundaryDecision(st_boundary_with_decision->id());
      if (decision) {
        if (*decision == StBoundaryProto::UNKNOWN) {
          return absl::InternalError(
              "Current point decision should not be UNKNOWN.");
        }

        st_boundary_with_decision->set_decision_type(*decision);
        st_boundary_with_decision->set_decision_reason(
            StBoundaryProto::SAMPLING_DP);
        st_boundary_with_decision->set_decision_info("decided by sampling dp");
      } else {
        continue;
      }
    }
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  for (int i = 1; i < speed_profile.size() - 1; ++i) {
    speed_profile[i].set_a(speed_profile[i + 1].a());
  }
  speed_profile.front().set_a(init_a_);

  return PreliminarySpeedWithCost(best_end_point->total_cost(),
                                  SpeedVector(std::move(speed_profile)));
}

std::vector<PreliminarySpeedWithCost>
GriddedSvtGraph::SampleSpeedProfilesFromSamplingDp(
    const SvGridIndexSet& final_layer_indices) {
  absl::flat_hash_map<int, BestEndPoint> final_layer_points;
  constexpr int kSDimensionSampleInterval = 4;
  for (const auto& index : final_layer_indices) {
    const auto& end_point =
        layers_.back()[index.grid_index_s_][index.grid_index_v_];
    const int sample_index = index.grid_index_s_ / kSDimensionSampleInterval;
    if (final_layer_points.find(sample_index) != final_layer_points.end()) {
      if (end_point->total_cost() < final_layer_points[sample_index].cost) {
        final_layer_points[sample_index] =
            BestEndPoint(&*end_point, end_point->total_cost());
      }
    } else {
      final_layer_points[sample_index] =
          BestEndPoint(&*end_point, end_point->total_cost());
    }
  }

  absl::flat_hash_map<int, BestEndPoint> layers_final_point;
  for (int i = 0; i < layers_.size() - 1; ++i) {
    for (const auto& end_point : layers_[i].back()) {
      if (end_point == nullptr || !end_point->next_point().empty()) {
        continue;
      }
      if (layers_final_point.find(i) != layers_final_point.end()) {
        if (end_point->total_cost() < layers_final_point[i].cost) {
          layers_final_point[i] =
              BestEndPoint(&*end_point, end_point->total_cost());
        }
      } else {
        layers_final_point[i] =
            BestEndPoint(&*end_point, end_point->total_cost());
      }
    }
  }

  const auto get_speed_profile = [](double init_point_a,
                                    const SvtGraphPoint* cur_point) {
    std::vector<SpeedPoint> speed_profile;
    speed_profile.reserve(cur_point->index_t() + 1);

    while (cur_point != nullptr) {
      SpeedPoint speed_point;
      speed_point.set_s(cur_point->point().s());
      speed_point.set_v(cur_point->point().v());
      speed_point.set_a(cur_point->acc_from_pre_point());
      speed_point.set_t(cur_point->point().t());
      speed_profile.push_back(std::move(speed_point));
      cur_point = cur_point->pre_point();
    }
    std::reverse(speed_profile.begin(), speed_profile.end());

    for (int i = 1; i < speed_profile.size() - 1; ++i) {
      speed_profile[i].set_a(speed_profile[i + 1].a());
    }
    speed_profile.front().set_a(init_point_a);

    return speed_profile;
  };

  std::vector<PreliminarySpeedWithCost> speed_profiles;
  speed_profiles.reserve(layers_final_point.size() + final_layer_points.size());

  for (const auto& [_, best_end_point] : final_layer_points) {
    speed_profiles.emplace_back(
        best_end_point.cost,
        SpeedVector(get_speed_profile(init_a_, best_end_point.point)));
  }
  for (const auto& [_, best_end_point] : layers_final_point) {
    speed_profiles.emplace_back(
        best_end_point.cost,
        SpeedVector(get_speed_profile(init_a_, best_end_point.point)));
  }

  return speed_profiles;
}

}  // namespace e2e_noa::planning
