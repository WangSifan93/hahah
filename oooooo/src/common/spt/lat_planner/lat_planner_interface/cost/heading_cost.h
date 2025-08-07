#pragma once
#include "common/spt/lat_planner/lat_planner_interface/cost/cost_common.h"
#include "common/spt/lat_planner/lat_planner_interface/data/spt_data.h"

namespace e2e_noa {
namespace spt {
using e2e_noa::spt::CostBase;
// Reference path tracking cost
class HeadingCost : public CostBase<STATE_SIZE, INPUT_SIZE> {
 public:
  explicit HeadingCost(const LatSptDataCache &data) : spt_data_(data) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientAndHessian(const State &x, const Control & /*u*/,
                             const size_t step, Lx *const lx, Lu *const /*lu*/,
                             Lxx *const lxx, Lxu *const /*lxu*/,
                             Luu *const /*luu*/) const override;
  std::uint8_t GetCostId() const override { return CostList::HEADING_COST; }

 private:
  const LatSptDataCache &spt_data_;
};
}  // namespace spt
}  // namespace e2e_noa