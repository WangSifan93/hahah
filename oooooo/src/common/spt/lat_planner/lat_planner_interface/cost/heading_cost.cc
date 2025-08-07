#include "common/spt/lat_planner/lat_planner_interface/cost/heading_cost.h"
namespace e2e_noa {
namespace spt {
double HeadingCost::GetCost(const State &x, const Control & /*u*/,
                            const size_t step) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::HEADING_WEIGHT);
  const double diff_heading =
      x[StateIndex::THETA] -
      rear_circle_model.GetCircleData(SptCircleData::MATCHED_REF_ThETA);

  return QuadraticCost(weight, diff_heading);
}

void HeadingCost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                        const size_t step, Lx *const lx,
                                        Lu *const /*lu*/, Lxx *const lxx,
                                        Lxu *const /*lxu*/,
                                        Luu *const /*luu*/) const {
  const auto &circle_model = spt_data_.GetCircleModelInfo(step);
  const auto &rear_circle_model = circle_model.at(0);
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::HEADING_WEIGHT);
  const double diff_heading =
      x[StateIndex::THETA] -
      rear_circle_model.GetCircleData(SptCircleData::MATCHED_REF_ThETA);
  (*lx)(StateIndex::THETA) += weight * diff_heading;
  (*lxx)(StateIndex::THETA, StateIndex::THETA) += weight;
}
}  // namespace spt
}  // namespace e2e_noa