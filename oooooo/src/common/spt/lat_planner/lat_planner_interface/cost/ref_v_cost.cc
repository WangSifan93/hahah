#include "common/spt/lat_planner/lat_planner_interface/cost/ref_v_cost.h"
namespace e2e_noa {
namespace spt {
double RefVCost::GetCost(const State &x, const Control & /*u*/,
                         const size_t step) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_V_COST_WEIGHT);
  const double diff_v = x[StateIndex::V] - rear_circle_model.GetCircleData(
                                               SptCircleData::MATCHED_REF_V);

  return QuadraticCost(weight, diff_v);
}

void RefVCost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                     const size_t step, Lx *const lx,
                                     Lu *const /*lu*/, Lxx *const lxx,
                                     Lxu *const /*lxu*/,
                                     Luu *const /*luu*/) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_V_COST_WEIGHT);
  const double diff_v = x[StateIndex::V] - rear_circle_model.GetCircleData(
                                               SptCircleData::MATCHED_REF_V);
  (*lx)(StateIndex::V) += weight * diff_v;
  (*lxx)(StateIndex::V, StateIndex::V) += weight;
}
}  // namespace spt
}  // namespace e2e_noa