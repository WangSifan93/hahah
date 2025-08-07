#ifndef SPATIO_FORM_BUILDER_H_
#define SPATIO_FORM_BUILDER_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "common/log_data.h"
#include "trajectory_initialization.pb.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/piecewise_linear_function.h"
#include "math/vec.h"
#include "router/plan_passage.h"
#include "spacetime_search/spatio_graph/spatio_form.h"
#include "spacetime_search/spatio_graph/spatio_state.h"

namespace e2e_noa::planning {

struct PlanPassageSamplePoint {
  Vec2d xy;
  double l;
  double accumulated_s;
  int station_index;
};

class SpatioFormBuilder {
 public:
  SpatioFormBuilder() = default;

  explicit SpatioFormBuilder(const PlanPassage* passage,
                             double max_sampling_acc_s,
                             double s_from_start_with_diff);

  absl::StatusOr<PiecewiseLinearGeometry> BuildCubicSpiralGeometry(
      const SpatioState& start_state, const PlanPassageSamplePoint& end) const;

  absl::StatusOr<PiecewiseLinearGeometry> BuildCubicSpiralGeometry(
      const PlanPassageSamplePoint& start,
      const PlanPassageSamplePoint& end) const;

  absl::StatusOr<PiecewiseLinearGeometry> BuildQuinticSpiralGeometry(
      const SpatioState& start_state, const PlanPassageSamplePoint& end) const;

  absl::StatusOr<PiecewiseLinearGeometry> BuildQuinticSpiralGeometry(
      const PlanPassageSamplePoint& start,
      const PlanPassageSamplePoint& end) const;

  absl::StatusOr<PiecewiseLinearGeometry> BuildLateralQuinticPolyGeometry(
      const SpatioState& start_state, const PlanPassageSamplePoint& end) const;

  absl::StatusOr<PiecewiseLinearGeometry> BuildLateralQuinticPolyGeometry(
      const PlanPassageSamplePoint& start,
      const PlanPassageSamplePoint& end) const;

  double LookUpRefK(const double station) const {
    return k_s_.Evaluate(station);
  }

  FrenetCoordinate LookUpSL(const Vec2d& xy) const {
    return smoothed_frenet_frame_->XYToSL(xy);
  }

  double smooth_dp_sampling_acc_s() const {
    return default_max_sampling_acc_s_;
  }

  void FillSmoothPlanPassage(SpatioGraphProto* proto) const;

  void FillSmoothPlanPassage(
      zark::e2e_noa::debug::
          NOADebugInfo_PlanningDebugInfo_SpatioSearchDebugInfo_SpatioGraphProto*
              proto) const;

 private:
  const PlanPassage* passage_;
  double default_max_sampling_acc_s_;
  std::vector<double> station_k_;

  PiecewiseLinearFunction<double, double> k_s_;
  std::vector<Vec2d> smoothed_xy_;
  PiecewiseLinearFunction<double, double> smoothed_hsin_s_;
  PiecewiseLinearFunction<double, double> smoothed_hcos_s_;
  std::unique_ptr<FrenetFrame> smoothed_frenet_frame_;
  std::unique_ptr<FrenetFrame> frenet_frame_;
};

}  

#endif
