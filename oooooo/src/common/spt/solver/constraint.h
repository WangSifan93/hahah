#pragma once

#include "common/spt/solver/model_base.h"
#include "common/spt/solver/utils.h"

namespace e2e_noa {
namespace spt {
template <uint8_t M, uint8_t N, uint8_t C>
class Constraint {
 public:
  CONSTRAIN_TYPES(M, N, C)

  Constraint() = default;
  virtual ~Constraint() = default;

  virtual void Evaluate(const State & /*x*/, const Control & /*u*/,
                        const size_t /*step*/,
                        ConFunc *const /*out*/) const = 0;
  virtual void Jacobian(const State & /*x*/, const Control & /*u*/,
                        const size_t /*step*/, ConJac *const /*jac*/) const = 0;
  virtual std::uint8_t GetConstraintId() const = 0;
};

template <uint8_t C>
class AugLagUtils {
 public:
  using Lambda = Eigen::Matrix<double, C, 1>;
  using LambdaJac = Eigen::Matrix<double, C, C>;

  AugLagUtils() = default;
  virtual ~AugLagUtils() = default;

  static void Projection(const Lambda &x, Lambda *const x_proj) {
    for (size_t i = 0; i < C; ++i) {
      (*x_proj)(i) = std::fmin(0.0, x(i));
    }
  }
  static void Jacobian(const Lambda &x, LambdaJac *const jac) {
    for (size_t i = 0; i < C; ++i) {
      (*jac)(i, i) = x(i) > 0.0 ? 0.0 : 1;
    }
  }
};

}  // namespace spt
}  // namespace e2e_noa
