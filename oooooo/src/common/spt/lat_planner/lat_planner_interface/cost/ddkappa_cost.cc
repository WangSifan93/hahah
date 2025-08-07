#include "common/spt/lat_planner/lat_planner_interface/cost/ddkappa_cost.h"
namespace e2e_noa {
namespace spt {
double DDKappaCost::GetCost(const State & /*x*/, const Control &u,
                            const size_t step) const {
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::DDKAPPA_WEIGHT);
  const double ddkappa = u[ControlIndex::DDKAPPA];

  return QuadraticCost(weight, ddkappa);
}

void DDKappaCost::GetGradientAndHessian(const State & /*x*/, const Control &u,
                                        const size_t step, Lx *const /*lx*/,
                                        Lu *const lu, Lxx *const /*lxx*/,
                                        Lxu *const /*lxu*/,
                                        Luu *const luu) const {
  const double weight =
      spt_data_.GetSptStepData(step, SptStepData::DDKAPPA_WEIGHT);
  const double ddkappa = u[ControlIndex::DDKAPPA];
  (*lu)(ControlIndex::DDKAPPA) += weight * ddkappa;
  (*luu)(ControlIndex::DDKAPPA, ControlIndex::DDKAPPA) += weight;
}
}  // namespace spt
}  // namespace e2e_noa