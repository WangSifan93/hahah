#include "common/spt/lat_planner/lat_planner_interface/cost/ref_a_cost.h"
namespace e2e_noa {
namespace spt {
double RefACost::GetCost(const State &x, const Control & /*u*/,
                         const size_t step) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_A_WEIGHT);
  const double diff_a = x[StateIndex::ACC] - rear_circle_model.GetCircleData(
                                                 SptCircleData::MATCHED_REF_A);

  return QuadraticCost(weight, diff_a);
}

void RefACost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                     const size_t step, Lx *const lx,
                                     Lu *const /*lu*/, Lxx *const lxx,
                                     Lxu *const /*lxu*/,
                                     Luu *const /*luu*/) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::REF_A_WEIGHT);
  const double diff_a = x[StateIndex::ACC] - rear_circle_model.GetCircleData(
                                                 SptCircleData::MATCHED_REF_A);
  (*lx)(StateIndex::ACC) += weight * diff_a;
  (*lxx)(StateIndex::ACC, StateIndex::ACC) += weight;
}
}  // namespace spt
}  // namespace e2e_noa