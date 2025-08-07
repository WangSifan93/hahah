#include "common/spt/lat_planner/lat_planner_interface/cost/v_limit_cost.h"
namespace e2e_noa {
namespace spt {
constexpr double kVLimit_ub = 10.0;
constexpr double kVLimit_lb = 5.0;
double VLimitCost::GetCost(const State &x, const Control & /*u*/,
                           const size_t step) const {
  assert(kVLimit_lb < kVLimit_ub);
  double cost = 0.0;
  if (x[StateIndex::V] > kVLimit_ub) {
    const double weight =
        spt_data_.GetSptStepData(step, SptStepData::V_LIMIT_UB_WEIGHT);
    const double diff_v = x[StateIndex::V] - kVLimit_ub;
    cost += QuadraticCost(weight, diff_v);
  }
  if (x[StateIndex::V] < kVLimit_lb) {
    const double weight =
        spt_data_.GetSptStepData(step, SptStepData::V_LIMIT_LB_WEIGHT);
    const double diff_v = x[StateIndex::V] - kVLimit_lb;
    cost += QuadraticCost(weight, diff_v);
  }

  return cost;
}

void VLimitCost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                       const size_t step, Lx *const lx,
                                       Lu *const /*lu*/, Lxx *const lxx,
                                       Lxu *const /*lxu*/,
                                       Luu *const /*luu*/) const {
  if (x[StateIndex::V] > kVLimit_ub) {
    const double weight =
        spt_data_.GetSptStepData(step, SptStepData::V_LIMIT_UB_WEIGHT);
    const double diff_v = x[StateIndex::V] - kVLimit_ub;
    (*lx)(StateIndex::V) += weight * diff_v;
    (*lxx)(StateIndex::V, StateIndex::V) += weight;
  }
  if (x[StateIndex::V] < kVLimit_lb) {
    const double weight =
        spt_data_.GetSptStepData(step, SptStepData::V_LIMIT_LB_WEIGHT);
    const double diff_v = x[StateIndex::V] - kVLimit_lb;
    (*lx)(StateIndex::V) += weight * diff_v;
    (*lxx)(StateIndex::V, StateIndex::V) += weight;
  }
}
}  // namespace spt
}  // namespace e2e_noa