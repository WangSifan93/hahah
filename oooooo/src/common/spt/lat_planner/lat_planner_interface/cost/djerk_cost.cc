#include "common/spt/lat_planner/lat_planner_interface/cost/djerk_cost.h"
namespace e2e_noa {
namespace spt {
double DJerkCost::GetCost(const State & /*x*/, const Control &u,
                          const size_t step) const {
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::DJERK_WEIGHT);
  const double djerk = u[ControlIndex::DJERK];

  return QuadraticCost(weight, djerk);
}

void DJerkCost::GetGradientAndHessian(const State & /*x*/, const Control &u,
                                      const size_t step, Lx *const /*lx*/,
                                      Lu *const lu, Lxx *const /*lxx*/,
                                      Lxu *const /*lxu*/,
                                      Luu *const luu) const {
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::DJERK_WEIGHT);
  const double djerk = u[ControlIndex::DJERK];
  (*lu)(ControlIndex::DJERK) += weight * djerk;
  (*luu)(ControlIndex::DJERK, ControlIndex::DJERK) += weight;
}
}  // namespace spt
}  // namespace e2e_noa