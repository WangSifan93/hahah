#include "common/path_sl_boundary.h"

#include "log_data.h"
#include "math/geometry/util.h"
#include "math/piecewise_linear_function.h"
namespace e2e_noa::planning {
namespace {
constexpr double kDesiredPathSlBoundarySampleStep = 1.0;
constexpr int kPathSlBoundaryMinInterval = 2;
}  // namespace
void PathSlBoundary::DumpToDebugFrame(const std::string& plan_id) const {
  auto& debug_plan_passage = NOA_PLANPASSAGE_DEBUG_INFO(plan_id);
  debug_plan_passage.set_plan_id(plan_id);
  auto* draw_path_boundary = debug_plan_passage.add_path_boundaries();

  debug::VectorsToMarker(draw_path_boundary->mutable_sl_boundary_right(),
                         "sl_right", right_xy_vec_,
                         zark::e2e_noa::debug::Marker_Type::Marker_Type_LINE,
                         Log2FG::kAqua);
  debug::VectorsToMarker(
      draw_path_boundary->mutable_sl_boundary_left(), "sl_left", left_xy_vec_,
      zark::e2e_noa::debug::Marker_Type::Marker_Type_LINE, Log2FG::kAqua);

  debug::VectorsToMarker(draw_path_boundary->mutable_sl_opt_right(),
                         "sl_opt_right", opt_right_xy_vec_,
                         zark::e2e_noa::debug::Marker_Type::Marker_Type_LINE,
                         Log2FG::kLime);
  debug::VectorsToMarker(draw_path_boundary->mutable_sl_opt_left(),
                         "sl_opt_left", opt_left_xy_vec_,
                         zark::e2e_noa::debug::Marker_Type::Marker_Type_LINE,
                         Log2FG::kLime);

  debug::VectorsToMarker(draw_path_boundary->mutable_sl_target_right(),
                         "sl_target-right", target_right_xy_vec_,
                         zark::e2e_noa::debug::Marker_Type::Marker_Type_LINE,
                         Log2FG::kBlue);
  debug::VectorsToMarker(draw_path_boundary->mutable_sl_target_left(),
                         "sl_target-left", target_left_xy_vec_,
                         zark::e2e_noa::debug::Marker_Type::Marker_Type_LINE,
                         Log2FG::kBlue);
}

void PathSlBoundary::BuildResampledData() {
  PiecewiseLinearFunction<double, double> ref_center_l_s_plf(s_vec_,
                                                             ref_center_l_vec_);
  PiecewiseLinearFunction<double, double> right_l_s_plf(s_vec_, right_l_vec_);
  PiecewiseLinearFunction<double, double> left_l_s_plf(s_vec_, left_l_vec_);
  PiecewiseLinearFunction<double, double> opt_right_l_s_plf(s_vec_,
                                                            opt_right_l_vec_);
  PiecewiseLinearFunction<double, double> opt_left_l_s_plf(s_vec_,
                                                           opt_left_l_vec_);
  PiecewiseLinearFunction<double, double> target_right_l_s_plf(
      s_vec_, target_right_l_vec_);
  PiecewiseLinearFunction<double, double> target_left_l_s_plf(
      s_vec_, target_left_l_vec_);
  PiecewiseLinearFunction<Vec2d, double> ref_center_xy_s_plf(
      s_vec_, ref_center_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> right_xy_s_plf(s_vec_, right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> left_xy_s_plf(s_vec_, left_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> opt_right_xy_s_plf(s_vec_,
                                                            opt_right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> opt_left_xy_s_plf(s_vec_,
                                                           opt_left_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> target_right_xy_s_plf(
      s_vec_, target_right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> target_left_xy_s_plf(
      s_vec_, target_left_xy_vec_);
  const double start_s = s_vec_.front();
  const double end_s = s_vec_.back();
  const int num_interval = std::max<int>(
      static_cast<int>((end_s - start_s) / kDesiredPathSlBoundarySampleStep),
      kPathSlBoundaryMinInterval);
  sample_interval_ = (end_s - start_s) / num_interval;
  const int num_points = num_interval + 1;
  resampled_s_vec_.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const double cur_s = start_s + i * sample_interval_;
    resampled_s_vec_.push_back(cur_s);
  }
  resampled_ref_center_l_vec_ = ref_center_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_right_l_vec_ = right_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_left_l_vec_ = left_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_opt_right_l_vec_ = opt_right_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_opt_left_l_vec_ = opt_left_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_right_l_vec_ =
      target_right_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_left_l_vec_ = target_left_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_ref_center_xy_vec_ = ref_center_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_right_xy_vec_ = right_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_left_xy_vec_ = left_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_opt_right_xy_vec_ = opt_right_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_opt_left_xy_vec_ = opt_left_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_right_xy_vec_ =
      target_right_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_left_xy_vec_ =
      target_left_xy_s_plf.Evaluate(resampled_s_vec_);
}

PathSlBoundary::PathSlBoundary(
    std::vector<double> s, std::vector<double> ref_center_l,
    std::vector<double> right_l, std::vector<double> left_l,
    std::vector<double> opt_right_l, std::vector<double> opt_left_l,
    std::vector<double> target_right_l, std::vector<double> target_left_l,
    std::vector<Vec2d> ref_center_xy, std::vector<Vec2d> right_xy,
    std::vector<Vec2d> left_xy, std::vector<Vec2d> opt_right_xy,
    std::vector<Vec2d> opt_left_xy, std::vector<Vec2d> target_right_xy,
    std::vector<Vec2d> target_left_xy)
    : s_vec_(std::move(s)),
      ref_center_l_vec_(std::move(ref_center_l)),
      right_l_vec_(std::move(right_l)),
      left_l_vec_(std::move(left_l)),
      opt_right_l_vec_(std::move(opt_right_l)),
      opt_left_l_vec_(std::move(opt_left_l)),
      target_right_l_vec_(std::move(target_right_l)),
      target_left_l_vec_(std::move(target_left_l)),
      ref_center_xy_vec_(std::move(ref_center_xy)),
      right_xy_vec_(std::move(right_xy)),
      left_xy_vec_(std::move(left_xy)),
      opt_right_xy_vec_(std::move(opt_right_xy)),
      opt_left_xy_vec_(std::move(opt_left_xy)),
      target_right_xy_vec_(std::move(target_right_xy)),
      target_left_xy_vec_(std::move(target_left_xy)) {
  const int vec_size = s_vec_.size();
  CHECK_GT(vec_size, 1);
  CHECK_EQ(vec_size, ref_center_l_vec_.size());
  CHECK_EQ(vec_size, ref_center_xy_vec_.size());
  CHECK_EQ(vec_size, right_l_vec_.size());
  CHECK_EQ(vec_size, left_l_vec_.size());
  CHECK_EQ(vec_size, opt_right_l_vec_.size());
  CHECK_EQ(vec_size, opt_left_l_vec_.size());
  CHECK_EQ(vec_size, target_right_l_vec_.size());
  CHECK_EQ(vec_size, target_left_l_vec_.size());
  CHECK_EQ(vec_size, right_xy_vec_.size());
  CHECK_EQ(vec_size, left_xy_vec_.size());
  CHECK_EQ(vec_size, opt_right_xy_vec_.size());
  CHECK_EQ(vec_size, opt_left_xy_vec_.size());
  CHECK_EQ(vec_size, target_right_xy_vec_.size());
  CHECK_EQ(vec_size, target_left_xy_vec_.size());
  BuildResampledData();
}

PathSlBoundary::PathSlBoundary(
    std::vector<double> s, std::vector<double> right_l,
    std::vector<double> left_l, std::vector<double> opt_right_l,
    std::vector<double> opt_left_l, std::vector<double> target_right_l,
    std::vector<double> target_left_l, std::vector<Vec2d> right_xy,
    std::vector<Vec2d> left_xy, std::vector<Vec2d> opt_right_xy,
    std::vector<Vec2d> opt_left_xy, std::vector<Vec2d> target_right_xy,
    std::vector<Vec2d> target_left_xy)
    : s_vec_(std::move(s)),
      right_l_vec_(std::move(right_l)),
      left_l_vec_(std::move(left_l)),
      opt_right_l_vec_(std::move(opt_right_l)),
      opt_left_l_vec_(std::move(opt_left_l)),
      target_right_l_vec_(std::move(target_right_l)),
      target_left_l_vec_(std::move(target_left_l)),
      right_xy_vec_(std::move(right_xy)),
      left_xy_vec_(std::move(left_xy)),
      opt_right_xy_vec_(std::move(opt_right_xy)),
      opt_left_xy_vec_(std::move(opt_left_xy)),
      target_right_xy_vec_(std::move(target_right_xy)),
      target_left_xy_vec_(std::move(target_left_xy)) {
  const int vec_size = s_vec_.size();
  CHECK_GT(vec_size, 1);
  CHECK_EQ(vec_size, right_l_vec_.size());
  CHECK_EQ(vec_size, left_l_vec_.size());
  CHECK_EQ(vec_size, opt_right_l_vec_.size());
  CHECK_EQ(vec_size, opt_left_l_vec_.size());
  CHECK_EQ(vec_size, target_right_l_vec_.size());
  CHECK_EQ(vec_size, target_left_l_vec_.size());
  CHECK_EQ(vec_size, right_xy_vec_.size());
  CHECK_EQ(vec_size, left_xy_vec_.size());
  CHECK_EQ(vec_size, opt_right_xy_vec_.size());
  CHECK_EQ(vec_size, opt_left_xy_vec_.size());
  CHECK_EQ(vec_size, target_right_xy_vec_.size());
  CHECK_EQ(vec_size, target_left_xy_vec_.size());

  ref_center_l_vec_.reserve(vec_size);
  ref_center_xy_vec_.reserve(vec_size);
  for (int i = 0; i < vec_size; ++i) {
    ref_center_l_vec_.emplace_back(0.5 * (right_l_vec_[i] + left_l_vec_[i]));
    ref_center_xy_vec_.emplace_back(0.5 * (right_xy_vec_[i] + left_xy_vec_[i]));
  }
  BuildResampledData();
}

void PathSlBoundary::ToProto(
    DecisionExplorationdPathBoundaryProto* proto) const {
  const int size = this->size();
  proto->mutable_reference_center()->Reserve(size);
  proto->mutable_left_boundary()->Reserve(size);
  proto->mutable_right_boundary()->Reserve(size);
  proto->mutable_opt_left_boundary()->Reserve(size);
  proto->mutable_opt_right_boundary()->Reserve(size);
  proto->mutable_target_left_boundary()->Reserve(size);
  proto->mutable_target_right_boundary()->Reserve(size);

  for (const auto& pt : resampled_ref_center_xy_vec_) {
    Vec2dToProto(pt, proto->add_reference_center());
  }
  for (const auto& pt : resampled_left_xy_vec_) {
    Vec2dToProto(pt, proto->add_left_boundary());
  }
  for (const auto& pt : resampled_right_xy_vec_) {
    Vec2dToProto(pt, proto->add_right_boundary());
  }
  for (const auto& pt : resampled_opt_left_xy_vec_) {
    Vec2dToProto(pt, proto->add_opt_left_boundary());
  }
  for (const auto& pt : resampled_opt_right_xy_vec_) {
    Vec2dToProto(pt, proto->add_opt_right_boundary());
  }
  for (const auto& pt : resampled_target_left_xy_vec_) {
    Vec2dToProto(pt, proto->add_target_left_boundary());
  }
  for (const auto& pt : resampled_target_right_xy_vec_) {
    Vec2dToProto(pt, proto->add_target_right_boundary());
  }
}

void PathSlBoundary::ModifySoftBoundByDpLabel(const PlanPassage& plan_passage,
                                              const double s_catchup,
                                              const double s_overtake,
                                              const double l_press,
                                              const bool is_left) {
  const double ego_width = 2.05, lat_buffer = 1.6;
  for (int idx_s = 0; idx_s < s_vec_.size(); idx_s++) {
    if (s_vec_[idx_s] < s_catchup) continue;
    if (s_vec_[idx_s] > s_overtake) break;
    const auto& station = plan_passage.station(StationIndex(idx_s));
    if (is_left) {
      target_right_l_vec_[idx_s] =
          std::fmax(target_right_l_vec_[idx_s], l_press);
      target_left_l_vec_[idx_s] = std::fmax(target_left_l_vec_[idx_s],
                                            l_press + ego_width + lat_buffer);
    } else {
      target_left_l_vec_[idx_s] = std::fmin(target_left_l_vec_[idx_s], l_press);
      target_right_l_vec_[idx_s] = std::fmin(target_right_l_vec_[idx_s],
                                             l_press - ego_width - lat_buffer);
    }
    target_left_xy_vec_[idx_s] = station.lat_point(target_left_l_vec_[idx_s]);
    target_right_xy_vec_[idx_s] = station.lat_point(target_right_l_vec_[idx_s]);
  }

  PiecewiseLinearFunction<double, double> target_right_l_s_plf(
      s_vec_, target_right_l_vec_);
  PiecewiseLinearFunction<double, double> target_left_l_s_plf(
      s_vec_, target_left_l_vec_);
  PiecewiseLinearFunction<Vec2d, double> target_right_xy_s_plf(
      s_vec_, target_right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> target_left_xy_s_plf(
      s_vec_, target_left_xy_vec_);
  for (int idx_res_s = 0; idx_res_s < resampled_s_vec_.size(); idx_res_s++) {
    double resample_s = resampled_s_vec_[idx_res_s];
    if (resample_s < s_catchup) continue;
    if (resample_s > s_overtake) break;
    resampled_target_right_l_vec_[idx_res_s] =
        target_right_l_s_plf.Evaluate(resample_s);
    resampled_target_right_xy_vec_[idx_res_s] =
        target_right_xy_s_plf.Evaluate(resample_s);
    resampled_target_left_l_vec_[idx_res_s] =
        target_left_l_s_plf.Evaluate(resample_s);
    resampled_target_left_xy_vec_[idx_res_s] =
        target_left_xy_s_plf.Evaluate(resample_s);
  }
}

void PathSlBoundary::ModifyHardBoundByDpLabel(const PlanPassage& plan_passage,
                                              const double s_catchup,
                                              const double s_overtake,
                                              const double l_press,
                                              const bool is_left) {
  const double ego_width = 2.05, lat_buffer = 1.6;
  for (int idx_s = 0; idx_s < s_vec_.size(); idx_s++) {
    if (s_vec_[idx_s] < s_catchup) continue;
    if (s_vec_[idx_s] > s_overtake) break;
    const auto& station = plan_passage.station(StationIndex(idx_s));
    if (is_left) {
      right_l_vec_[idx_s] = std::fmax(right_l_vec_[idx_s], l_press);
      left_l_vec_[idx_s] =
          std::fmax(left_l_vec_[idx_s], l_press + ego_width + lat_buffer);
    } else {
      left_l_vec_[idx_s] = std::fmin(left_l_vec_[idx_s], l_press);
      right_l_vec_[idx_s] =
          std::fmin(right_l_vec_[idx_s], l_press - ego_width - lat_buffer);
    }
    left_xy_vec_[idx_s] = station.lat_point(left_l_vec_[idx_s]);
    right_xy_vec_[idx_s] = station.lat_point(right_l_vec_[idx_s]);
  }

  PiecewiseLinearFunction<double, double> right_l_s_plf(s_vec_, right_l_vec_);
  PiecewiseLinearFunction<double, double> left_l_s_plf(s_vec_, left_l_vec_);
  PiecewiseLinearFunction<Vec2d, double> right_xy_s_plf(s_vec_, right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> left_xy_s_plf(s_vec_, left_xy_vec_);
  for (int idx_res_s = 0; idx_res_s < resampled_s_vec_.size(); idx_res_s++) {
    double resample_s = resampled_s_vec_[idx_res_s];
    if (resample_s < s_catchup) continue;
    if (resample_s > s_overtake) break;
    resampled_right_l_vec_[idx_res_s] = right_l_s_plf.Evaluate(resample_s);
    resampled_right_xy_vec_[idx_res_s] = right_xy_s_plf.Evaluate(resample_s);
    resampled_left_l_vec_[idx_res_s] = left_l_s_plf.Evaluate(resample_s);
    resampled_left_xy_vec_[idx_res_s] = left_xy_s_plf.Evaluate(resample_s);
  }
}

}  // namespace e2e_noa::planning
