#include "common/spt/lat_planner/lat_planner_interface/cost/long_jerk_cost.h"
namespace e2e_noa {
namespace spt {
double LONGJerkCost::GetCost(const State &x, const Control & /*u*/,
                            const size_t step) const {
  const double weight_jerk =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_WEIGHT);
  const double weight_jerk_ub =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_UB_WEIGHT);
  const double weight_jerk_lb =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_LB_WEIGHT);
  const double jerk_ub =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_BOUNDARY_UPPER);
  const double jerk_lb =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_BOUNDARY_LOWER);
  const double jerk = x[StateIndex::JERK];
  const double jerk_diff_ub = std::max(0.0, jerk - jerk_ub);
  const double jerk_diff_lb = std::max(0.0, jerk_lb - jerk);

  return QuadraticCost(weight_jerk, jerk) +
         QuadraticCost(weight_jerk_ub, jerk_diff_ub) +
         QuadraticCost(weight_jerk_lb, jerk_diff_lb);
}

void LONGJerkCost::GetGradientAndHessian(const State &x, const Control & /*u*/,
                                        const size_t step, Lx *const lx,
                                        Lu *const /*lu*/, Lxx *const lxx,
                                        Lxu *const /*lxu*/,
                                        Luu *const /*luu*/) const {
  const double weight_jerk =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_WEIGHT);
  const double weight_jerk_ub =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_UB_WEIGHT);
  const double weight_jerk_lb =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_LB_WEIGHT);
  const double jerk_ub =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_BOUNDARY_UPPER);
  const double jerk_lb =
      spt_data_.GetSptStepData(step, SptStepData::JERK_LONG_BOUNDARY_LOWER);
  const double jerk = x[StateIndex::JERK];
  const double jerk_diff_ub = jerk - jerk_ub;
  const double jerk_diff_lb = jerk - jerk_lb;
  if (jerk_diff_ub > 0.0) {
    (*lx)(StateIndex::JERK) += weight_jerk * jerk + weight_jerk_ub * jerk_diff_ub;
    (*lxx)(StateIndex::JERK, StateIndex::JERK) += weight_jerk + weight_jerk_ub;
  } else if (jerk_diff_lb < 0.0) {
    (*lx)(StateIndex::JERK) += weight_jerk * jerk + weight_jerk_lb * jerk_diff_lb;
    (*lxx)(StateIndex::JERK, StateIndex::JERK) += weight_jerk + weight_jerk_lb;
  } else {
    (*lx)(StateIndex::JERK) += weight_jerk * jerk;
    (*lxx)(StateIndex::JERK, StateIndex::JERK) += weight_jerk;
  }
}
}  // namespace spt
}  // namespace e2e_noa