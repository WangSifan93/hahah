#include "common/spt/lat_planner/lat_planner_interface/cost/ref_path_cost.h"
namespace e2e_noa {
namespace spt {
double RefpathCost::GetCost(const State & /*x*/, const Control & /*u*/,
                            const size_t step) const {
  double cost = 0.0;
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &front_circle_model = circle_model.at(2);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_PATH_COST_WEIGHT);
  const double threshold =
      spt_data_.GetSptConstant(SptConstant::REF_PATH_COST_HUBER_THRESHOLD);
  const double d_front_offset =
      front_circle_model.GetCircleData(SptCircleData::LAT_OFFSET) -
      front_circle_model.GetCircleData(SptCircleData::REF_LAT_OFFSET);
  const double d_rear_offset =
      rear_circle_model.GetCircleData(SptCircleData::LAT_OFFSET) -
      rear_circle_model.GetCircleData(SptCircleData::REF_LAT_OFFSET);
  cost += LatCostCommon::GetCostOneStep(weight, threshold, d_front_offset);
  cost += LatCostCommon::GetCostOneStep(weight, threshold, d_rear_offset);
  return cost;
}

void RefpathCost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                        const size_t step, Lx *const lx,
                                        Lu *const /*lu*/, Lxx *const lxx,
                                        Lxu *const /*lxu*/,
                                        Luu *const /*luu*/) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &front_circle_model = circle_model.at(2);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_PATH_COST_WEIGHT);
  const double threshold =
      spt_data_.GetSptConstant(SptConstant::REF_PATH_COST_HUBER_THRESHOLD);
  const double d_front_offset =
      front_circle_model.GetCircleData(SptCircleData::LAT_OFFSET) -
      front_circle_model.GetCircleData(SptCircleData::REF_LAT_OFFSET);
  const double d_rear_offset =
      rear_circle_model.GetCircleData(SptCircleData::LAT_OFFSET) -
      rear_circle_model.GetCircleData(SptCircleData::REF_LAT_OFFSET);
  const double sin_theta =
      spt_data_.GetSptStepData(step, SptStepData::SIN_THETA);
  const double cos_theta =
      spt_data_.GetSptStepData(step, SptStepData::COS_THETA);
  LatCostCommon::GetGradientAndHessianOneStep(
      weight, threshold, d_front_offset, cos_theta, sin_theta,
      front_circle_model.Length(), front_circle_model.Width(),
      front_circle_model.GetCircleData(SptCircleData::MATCHED_REF_COS_THETA),
      front_circle_model.GetCircleData(SptCircleData::MATCHED_REF_SIN_THETA),
      lx, lxx);
  LatCostCommon::GetGradientAndHessianOneStep(
      weight, threshold, d_rear_offset, cos_theta, sin_theta,
      rear_circle_model.Length(), rear_circle_model.Width(),
      rear_circle_model.GetCircleData(SptCircleData::MATCHED_REF_COS_THETA),
      rear_circle_model.GetCircleData(SptCircleData::MATCHED_REF_SIN_THETA), lx,
      lxx);
}
}  // namespace spt
}  // namespace e2e_noa