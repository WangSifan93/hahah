#pragma once

#include <cstddef>

#include "common/spt/solver/constraint.h"

namespace e2e_noa {
namespace spt {

template <uint8_t M, uint8_t N>
class BoundConstraint {
 public:
  SOLVER_TYPES(M, N)
  using ConGrad = Eigen::Matrix<double, M + N, 1>;
  using ConJac = Eigen::Matrix<double, 2, M + N>;
  using Vector2d = Eigen::Vector2d;
  using Matrix2d = Eigen::Matrix2d;

  BoundConstraint() = default;

  void Init(Constraint<M, N, 2> *const constraint, const double constraint_tol,
            const e2e_noa::spt::SolverConfig *const config_ptr) {
    solver_config_ptr_ = config_ptr;
    m_ = M;
    n_ = N;
    constraint_ = constraint;
    constraint_tolerance_ = constraint_tol;
  }

  void Reset() {
    index_ = 0;

    c_.setZero();
    c_proj_.setZero();
    identity_rho_.setIdentity();

    jac_.setZero();
    lambda_.setZero(2);
    temp_lambda_.setZero(2);
    dxdu_.setZero();
    temp_dxdu_.setZero();
    lambda_proj_.setZero();
    proj_jac_.setZero();
    jac_proj_.setZero();
  }

  void UpdateIrho(const Vector2d &c, Matrix2d *I_rho) {
    const int c_size = c.size();
    for (int i = 0; i < c_size; ++i) {
      if (c(i) < 0 && std::fabs(lambda_(i)) < 1e-3) {
        (*I_rho)(i, i) = 0.0;
      } else {
        (*I_rho)(i, i) = rho_;
      }
    }
  }

  double AugLag(const State &x, const Control &u, const size_t step) {
    constraint_->Evaluate(x, u, step, &c_);
    temp_lambda_(0) = lambda_(0) - rho_ * c_(0);
    temp_lambda_(1) = lambda_(1) - rho_ * c_(1);
    AugLagUtils<2>::Projection(temp_lambda_, &lambda_proj_);
    const double lambda_proj_sqrnorm =
        lambda_proj_(0) * lambda_proj_(0) + lambda_proj_(1) * lambda_proj_(1);
    const double lambda_sqrnorm =
        lambda_(0) * lambda_(0) + lambda_(1) * lambda_(1);
    return (lambda_proj_sqrnorm - lambda_sqrnorm) / (2 * rho_);
  }

  void AugLagGradientHessian(const State &x, const Control &u,
                             const size_t step, State *const dx,
                             Control *const du, Lxx *const dxdx,
                             Lxu *const dxdu, Luu *const dudu) {
    constraint_->Evaluate(x, u, step, &c_);
    constraint_->Jacobian(x, u, step, &jac_);
    AugLagUtils<2>::Projection(temp_lambda_, &lambda_proj_);
    AugLagUtils<2>::Jacobian(temp_lambda_, &proj_jac_);

    if (lambda_proj_(0) < 0.0) {
      index_ = 0;
    } else if (lambda_proj_(1) < 0.0) {
      index_ = 1;
    } else {
      return;
    }
    temp_dxdu_.noalias() = proj_jac_(index_, index_) * jac_.row(index_);
    dxdu_.noalias() = -temp_dxdu_ * lambda_proj_(index_);
    (*dx).noalias() += dxdu_.head(m_);
    (*du).noalias() += dxdu_.tail(n_);
    jac_proj_.noalias() = proj_jac_ * jac_;
    (*dxdx).noalias() +=
        rho_ * temp_dxdu_.head(m_) * temp_dxdu_.head(m_).transpose();
    (*dxdu).noalias() +=
        rho_ * temp_dxdu_.head(m_) * temp_dxdu_.tail(n_).transpose();
    (*dudu).noalias() +=
        rho_ * temp_dxdu_.tail(n_) * temp_dxdu_.tail(n_).transpose();
  }

  void UpdateDuals(const State &x, const Control &u, const size_t step) {
    constraint_->Evaluate(x, u, step, &c_);
    AugLagUtils<2>::Projection(lambda_ - rho_ * c_, &lambda_);
  }

  const Vector2d &GetDuals() { return lambda_; }

  const double &GetPenalty() { return rho_; }

  void UpdatePenalties() { rho_ *= solver_config_ptr_->penalty_factor; }

  void ResetPenalties() { rho_ = solver_config_ptr_->init_rho; }
  void ResetDuals() {
    lambda_.resize(2);
    lambda_.setZero();
  }

  double MaxPenalty() { return rho_; }

  bool CheckViolation() { return MaxViolation() <= constraint_tolerance_; }

  template <int p = Eigen::Infinity>
  double MaxViolation() {
    AugLagUtils<2>::Projection(c_, &c_proj_);
    c_proj_ = c_ - c_proj_;
    return c_proj_.template lpNorm<p>();
  }

  std::uint8_t GetConstraintId() const {
    return constraint_->GetConstraintId();
  }

 private:
  uint8_t m_{5};
  uint8_t n_{1};
  Constraint<M, N, 2> *constraint_{nullptr};
  double constraint_tolerance_{0.0};
  Vector2d c_;
  Vector2d c_proj_;
  double rho_{1.0};
  Matrix2d identity_rho_;
  size_t index_{0};

  ConJac jac_;
  ConJac jac_proj_;
  Matrix2d proj_jac_;
  Vector2d lambda_;
  Vector2d lambda_proj_;
  Vector2d temp_lambda_;
  ConGrad dxdu_;
  ConGrad temp_dxdu_;

  const SolverConfig *solver_config_ptr_{nullptr};
};

}  // namespace spt
}  // namespace e2e_noa
