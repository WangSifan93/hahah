#include "common/spt/lat_planner/lat_planner_interface/cost/ref_s_cost.h"
namespace e2e_noa {
namespace spt {
double RefSCost::GetCost(const State & /*x*/, const Control & /*u*/,
                         const size_t step) const {
  double cost = 0.0;
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_S_COST_WEIGHT);
  const double threshold =
      spt_data_.GetSptConstant(SptConstant::REF_S_COST_HUBER_THRESHOLD);

  const double ref_ds_rear =
      rear_circle_model.GetCircleData(SptCircleData::S_OFFSET);
  cost += LatCostCommon::GetCostOneStep(weight, threshold, ref_ds_rear);
  return cost;
}

void RefSCost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                     const size_t step, Lx *const lx,
                                     Lu *const /*lu*/, Lxx *const lxx,
                                     Lxu *const /*lxu*/,
                                     Luu *const /*luu*/) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_S_COST_WEIGHT);
  const double threshold =
      spt_data_.GetSptConstant(SptConstant::REF_S_COST_HUBER_THRESHOLD);
  const double ref_ds_rear =
      rear_circle_model.GetCircleData(SptCircleData::S_OFFSET);
  const double sin_theta =
      spt_data_.GetSptStepData(step, SptStepData::SIN_THETA);
  const double cos_theta =
      spt_data_.GetSptStepData(step, SptStepData::COS_THETA);
  LatCostCommon::GetGradientAndHessianRefDsOneStep(
      weight, threshold, ref_ds_rear, cos_theta, sin_theta,
      rear_circle_model.Length(), rear_circle_model.Width(),
      rear_circle_model.GetCircleData(SptCircleData::MATCHED_REF_COS_THETA),
      rear_circle_model.GetCircleData(SptCircleData::MATCHED_REF_SIN_THETA), lx,
      lxx);
}
}  // namespace spt
}  // namespace e2e_noa