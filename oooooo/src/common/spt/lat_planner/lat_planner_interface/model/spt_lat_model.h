#pragma once

#include "common/spt/lat_planner/lat_planner_interface/data/spt_data.h"
#include "common/spt/solver/cost_base.h"
#include "common/spt/solver/utils.h"
namespace e2e_noa {
namespace spt {

class SptLatModel : public e2e_noa::spt::ModelBase<StateIndex::STATE_SIZE,
                                                   ControlIndex::INPUT_SIZE> {
 public:
  explicit SptLatModel(const LatSptDataCache &data) : spt_data_(data) {}
  State UpdateDynamicsOneStep(const State &x, const Control &u,
                              const size_t step) const override;
  void GetDynamicsDerivatives(const State &x, const Control &u,
                              const size_t step, Fx *const f_x,
                              Fu *const f_u) const override;

  static State UpdateDynamicsWithDt(const State &x, const Control &u,
                                    const double dt);
  static void CalRk4AndDynamicsDerivatives(const State &x, const Control &u,
                                           const double dt, Fx *f_x, Fu *f_u);
  void LimitControl(const State &x, Control *u,
                    const size_t step) const override {}

 private:
  const LatSptDataCache &spt_data_;
};

}  // namespace spt
}  // namespace e2e_noa
