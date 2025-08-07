#pragma once

#include <cstdint>

#include "utils.h"

namespace e2e_noa {
namespace spt {

template <uint8_t M, uint8_t N>
class ModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SOLVER_TYPES(M, N)
  ModelBase() {}

  virtual ~ModelBase() = default;

  virtual State UpdateDynamicsOneStep(const State &x, const Control &u,
                                      const size_t step) const = 0;

  virtual void GetDynamicsDerivatives(const State &x, const Control & /*u*/,
                                      const size_t step, Fx *const f_x,
                                      Fu *const f_u) const = 0;
  virtual void LimitControl(const State &x, Control *const u,
                            const size_t step) const = 0;
};

}  // namespace spt
}  // namespace e2e_noa
