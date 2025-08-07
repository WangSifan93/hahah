#pragma once

#include <cstddef>

#include "common/spt/solver/bound_constraint.h"
#include "common/spt/solver/model_base.h"
#include "common/spt/solver/utils.h"

namespace e2e_noa {
namespace spt {

template <uint8_t M, uint8_t N>
class CostBase {
 public:
  SOLVER_TYPES(M, N)
  virtual ~CostBase() = default;
  virtual double GetCost(const State & /*x*/, const Control & /*u*/,
                         const size_t /*step*/) const {
    return 0.0;
  }
  virtual void UpdateCostCommon(const State & /*x*/, const Control & /*u*/,
                                const size_t /*step*/) {}
  virtual void GetGradientAndHessian(const State & /*x*/, const Control & /*u*/,
                                     const size_t /*step*/, Lx *const /*lx*/,
                                     Lu *const /*lu*/, Lxx *const /*lxx*/,
                                     Lxu *const /*lxu*/,
                                     Luu *const /*luu*/) const {}
  virtual uint8_t GetCostId() const { return 0; }
};

template <uint8_t M, uint8_t N, uint8_t C>
class CostManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SOLVER_TYPES(M, N)
  using ConVec = Eigen::Matrix<double, C, 1>;

  CostManager(const ModelBase<M, N> *const model_ptr,
              const SolverConfig *const solver_config_ptr,
              CostBase<M, N> *const common_term) {
    ilqr_model_ptr_ = model_ptr;
    solver_config_ptr_ = solver_config_ptr;
    common_term_ = common_term;
  }
  virtual ~CostManager() = default;

 public:
  void Reset() {
    for (auto &constraints : constraint_vec_) {
      for (auto &constraint : constraints) {
        constraint.Reset();
      }
    }
  }

  void InitGuess(const bool update_init_seed, StateVec *const x0,
                 ControlVec *const u0, double *const init_cost,
                 int *init_seed_index) {
    if ((!update_init_seed) && (!warm_start_uk_list_.empty())) {
      *init_cost = UpateDynamics(x0, u0);
      return;
    }
    StateVec x;
    ResizeVecAndResetEigen(x, GetHorizon() + 1);
    x[0] = (*x0)[0];
    *init_cost = std::numeric_limits<double>::max();
    *init_seed_index = -1;
    for (size_t i = 0; i < warm_start_uk_list_.size(); ++i) {
      auto &uk_list = warm_start_uk_list_[i];
      const double cost = UpateDynamics(&x, &uk_list);
      if (cost < *init_cost) {
        *init_seed_index = i;
        *init_cost = cost;
        *u0 = uk_list;
        *x0 = x;
      }
    }
  }

  double GetCost(const State &x, const Control &u, const size_t &step) {
    common_term_->UpdateCostCommon(x, u, step);
    double cost = 0.0;
    double result = 0.0;
    for (const auto &cost_temp : cost_stack_) {
      result = cost_temp->GetCost(x, u, step);
      cost += result;
      cost_map_ptr_->at(cost_temp->GetCostId())[step] = result;
    }
    return cost;
  }

  double GetTerminalCost(const State &x) {
    Control u = Control::Zero(GetInputSize());
    const size_t t_final = GetHorizon();
    common_term_->UpdateCostCommon(x, u, t_final);

    double cost = 0.0;
    double result = 0.0;
    for (const auto &cost_temp : cost_stack_) {
      result = cost_temp->GetCost(x, u, t_final);
      cost += result;
      // update cost map
      cost_map_ptr_->at(cost_temp->GetCostId())[t_final] = result;
    }
    return cost;
  }

  void UpdateCommonTerms(const StateVec &x, const ControlVec &u) {
    const size_t t_final = GetHorizon();
    for (size_t t = 0; t < t_final; ++t) {
      common_term_->UpdateCostCommon(x[t], u[t], t);
    }
    Control u0 = Control::Zero(GetInputSize());
    common_term_->UpdateCostCommon(x[t_final], u0, t_final);
  }

  // Updates cx, cu
  void GetGradientAndHessian(const State &x, const Control &u,
                             const size_t &step, Lx *const lx, Lu *const lu,
                             Lxx *const lxx, Lxu *const lxu, Luu *const luu) {
    for (const auto &cost : cost_stack_) {
      cost->GetGradientAndHessian(x, u, step, lx, lu, lxx, lxu, luu);
    }
  }

  void GetTerminalGradientHessian(const State &x, Lx *const lx,
                                  Lxx *const lxx) {
    Control u = Control::Zero(GetInputSize());
    const size_t t_final = GetHorizon();
    for (const auto &cost : cost_stack_) {
      cost->GetGradientAndHessian(x, u, t_final, lx, &lu_, lxx, &lxu_, &luu_);
    }
  }

  void AddCost(const CostBase<M, N> *cost) { cost_stack_.emplace_back(cost); }

  void SetConstraintSize(const size_t &size) { constraint_size_ = size; }

  double GetAugLagCost(const State &x, const Control &u, const size_t &step) {
    double augument_cost = 0.0;
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      augument_cost += constraint_vec_[step][i].AugLag(x, u, step);
    }
    return augument_cost;
  }

  double GetTerminalConstraint(const State &x) {
    Control u = Control::Zero(GetInputSize());
    const size_t t_final = GetHorizon();
    double constraint = 0.0;
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint += constraint_vec_[t_final][i].AugLag(x, u, t_final);
    }
    return constraint;
  }

  void GetConstraintGradientHessian(const State &x, const Control &u,
                                    const size_t &step, Lx *const lx,
                                    Lu *const lu, Lxx *const lxx,
                                    Lxu *const lxu, Luu *const luu) {
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint_vec_[step][i].AugLagGradientHessian(x, u, step, lx, lu, lxx,
                                                     lxu, luu);
    }
  }

  void GetConstraintTerminalGradientHessian(const State &x, Lx *const lx,
                                            Lxx *const lxx) {
    Control u = Control::Zero(GetInputSize());
    const size_t t_final = GetHorizon();
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint_vec_[t_final][i].AugLagGradientHessian(x, u, t_final, lx, &lu_,
                                                        lxx, &lxu_, &luu_);
    }
  }

  double UpateDynamics(StateVec *const x0, ControlVec *u0) {
    double cost = 0.0;
    const size_t t_final = GetHorizon();
    for (size_t t = 0; t < t_final; ++t) {
      cost += GetCost((*x0)[t], (*u0)[t], t);
      (*x0)[t + 1] =
          ilqr_model_ptr_->UpdateDynamicsOneStep((*x0)[t], (*u0)[t], t);
    }
    cost += GetTerminalCost((*x0)[t_final]);
    return cost;
  }

  void UpdateDuals(const State &x, const Control &u, const size_t &step) {
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint_vec_[step][i].UpdateDuals(x, u, step);
    }
  }

  void UpdatePenalties(const size_t &step) {
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint_vec_[step][i].UpdatePenalties();
    }
  }

  void ResetDuals(const size_t &step) {
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint_vec_[step][i].ResetDuals();
    }
  }

  void ResetPenalties(const size_t &step) {
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      constraint_vec_[step][i].ResetPenalties();
    }
  }

  void AddConstraint(const size_t &step,
                     const std::vector<BoundConstraint<M, N>> &constraint_vec) {
    constraint_vec_.resize(GetHorizon() + 1);
    constraint_vec_[step] = std::move(constraint_vec);
  }

  ConVec GetDuals(const size_t &step) {
    ConVec temp = ConVec::Zero();
    for (size_t i = 0; i < C; ++i) {
      temp(i) = constraint_vec_[step][i].GetDuals().cwiseAbs().maxCoeff();
    }
    return temp;
  }

  ConVec GetPenalties(const size_t &step) {
    ConVec temp = ConVec::Zero();
    for (size_t i = 0; i < C; ++i) {
      temp(i) = constraint_vec_[step][i].GetPenalty();
    }
    return temp;
  }

  bool CheckViolation(const size_t &step) {
    const size_t constraint_size = ConstraintSize();
    for (size_t i = 0; i < constraint_size; ++i) {
      if (!constraint_vec_[step][i].CheckViolation()) {
        return false;
      }
    }
    return true;
  }

  ConVec GetViolation(const size_t &step) {
    ConVec temp = ConVec::Zero();
    for (size_t i = 0; i < C; ++i) {
      temp(i) = constraint_vec_[step][i].MaxViolation();
    }
    return temp;
  }

  size_t ConstraintSize() { return constraint_size_; }

  void InitControlVar() {
    lu_.setZero();
    lxu_.setZero();
    luu_.setZero();
  }

  void UpdateWarmStart(const std::vector<ControlVec> &warm_start_u_list) {
    warm_start_uk_list_.clear();
    warm_start_uk_list_.reserve(warm_start_u_list.size());
    for (const auto &element : warm_start_u_list) {
      warm_start_uk_list_.push_back(element);
    }
  }

  void SetCostMapPtr(std::vector<std::vector<double>> *cost_map_ptr) {
    cost_map_ptr_ = cost_map_ptr;
    cost_map_ptr->clear();
    cost_map_ptr_->resize(cost_stack_.size());
    for (auto &each_cost : *cost_map_ptr_) {
      each_cost.resize(GetHorizon() + 1);
    }
  }

  std::vector<const CostBase<M, N> *> GetCostStackPtr() { return cost_stack_; }

  const ModelBase<M, N> *Model() const { return ilqr_model_ptr_; }

  size_t GetHorizon() const { return solver_config_ptr_->horizon; }

  bool IsDebugMode() const { return solver_config_ptr_->is_debug_mode; }

  size_t GetInputSize() const { return solver_config_ptr_->input_size; }

  size_t GetStateSize() const { return solver_config_ptr_->state_size; }

 protected:
  std::vector<ControlVec> warm_start_uk_list_;

  Lu lu_;
  Lxu lxu_;
  Luu luu_;

  const ModelBase<M, N> *ilqr_model_ptr_;

  std::vector<const CostBase<M, N> *> cost_stack_;

  std::size_t constraint_size_{C};
  std::vector<std::vector<BoundConstraint<M, N>>> constraint_vec_;

  const SolverConfig *solver_config_ptr_;

  std::vector<std::vector<double>> *cost_map_ptr_;

  CostBase<M, N> *common_term_;
};

}  // namespace spt
}  // namespace e2e_noa
