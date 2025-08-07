#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>

#include "bound_constraint.h"
#include "constraint.h"
#include "ilqr_cost.h"
#include "ilqr_data.pb.h"
#include "ilqr_math_utils.h"
#include "ilqr_model.h"

namespace e2e_noa {
namespace spt {

template <uint8_t X, uint8_t Y, uint8_t Z>
class ILqr {
 public:
  using ConVec = Eigen::Matrix<double, C, 1>;

  struct CilqrInfo {
    std::vector<CilqrElem> cilqr_info;
  };

  struct CilqrIteration {
    std::vector<CilqrInfo> cilqr_iteration;
    std::vector<double> max_violation;
  };

  struct CilqrSolve {
    std::vector<CilqrIteration> cilqr_solve;
    size_t iter_count = 0;
  };

 public:
  ILqr() = default;
  ~ILqr() = default;

  void Init(const ILqrModel<M, N> *const model_ptr,
            const SolverConfig *const solver_config_ptr,
            CostBase<M, N> *const common_term_calculator) {
    solver_config_ = solver_config_ptr;
    cost_manager_ = std::make_unique<CostManager<M, N, C>>(
        model_ptr, solver_config_ptr, common_term_calculator);
    solver_info_.cost_size = 0;
  }

  void Solve(const State &x0) {
    time_info_.Reset();
    time_info_.UpdateAllStart();
    InitIlqrSolverInfo(&solver_info_);

    x_list_[0] = x0;

    cost_manager_->SetCostMapPtr(
        &(solver_info_.iteration_info_vec[0].cost_map));

    const bool update_init_seed = true;
    cost_manager_->InitGuess(update_init_seed, &x_list_, &u_list_, &cost_,
                             &init_seed_index_);
    cost_manager_->UpdateCommonTerms(x_list_, u_list_);
    if (IsCilqrEnable()) {
      const auto horizon = GetHorizon();
      for (size_t i = 0; i < horizon; ++i) {
        cost_ += cost_manager_->GetAugLagCost(x_list_[i], u_list_[i], i);
      }
      cost_ += cost_manager_->GetTerminalConstraint(x_list_[horizon]);
    }

    time_info_.t_init_guess_ms +=
        time_info_.GetElapsed(time_info_.all_start, false);

    UpdateAdvancedInfo(0);
    lambda_hessian_ = lambda_hessian_init_;

    using ResetEigenVec;
    if (x_list_.empty() || u_list_.empty()) {
      solver_info_.solver_condition = iLqrSolveCondition::FAULT_INPUT_SIZE;
      return false;
    }
    lambda_hessian_ = lambda_hessian_init_;
    solver_info_.init_guess_cost = 0.0;
    solver_info_.init_aug_lag_cost = 0.0;
    solver_info_.init_terminal_constraint_cost = 0.0;
    if (IsCilqrEnable()) {
      const bool update_init_seed = false;
      cost_manager_->InitGuess(update_init_seed, &x_list_, &u_list_, &cost_,
                               &init_seed_index_);
      solver_info_.init_guess_cost = cost_;
      const auto horizon = GetHorizon();
      for (size_t i = 0; i < horizon; ++i) {
        solver_info_.init_aug_lag_cost +=
            cost_manager_->GetAugLagCost(x_list_[i], u_list_[i], i);
      }
      cost_ += solver_info_.init_aug_lag_cost;
      solver_info_.init_terminal_constraint_cost =
          cost_manager_->GetTerminalConstraint(x_list_[horizon]);
      cost_ += solver_info_.init_terminal_constraint_cost;
    }

    solver_info_.init_cost = cost_;

    solver_info_.solver_condition = iLqrSolveCondition::INIT;

    bool solver_success = false;
    bool update_success = true;
    for (size_t iter = 0; iter < GetMaxIter(); ++iter) {
      time_info_.UpdateStart();

      solver_info_.iter_count += 1;

      cost_manager_->SetCostMapPtr(
          &(solver_info_.iteration_info_vec[solver_info_.iter_count].cost_map));

      if (update_success) {
        UpdateDynamicsDerivatives();
      }

      time_info_.t_compute_deriv_ms +=
          time_info_.GetElapsed(time_info_.start, true);

      size_t backward_pass_count = 0;
      while (true) {
        backward_pass_count++;

        const bool is_converged = BackwardPass();
        if (is_converged) {
          break;
        }
        if (backward_pass_count >= kMaxBackwardPassCount) {
          solver_info_.solver_condition =
              iLqrSolveCondition::BACKWARD_PASS_FAIL;
          return false;
        }
        LambdaForBackwardPass(backward_pass_count);
      }

      time_info_.t_backward_pass_ms +=
          time_info_.GetElapsed(time_info_.start, true);
      double expected = 0.0;
      double new_cost = 0.0;
      const bool forward_pass_success =
          ForwardPass(solver_info_.iter_count, &new_cost, &expected);
      const double dcost = cost_ - new_cost;

      auto &iter_debug_info =
          solver_info_.iteration_info_vec[solver_info_.iter_count];
      iter_debug_info.linesearch_success = forward_pass_success;

      iter_debug_info.cost = new_cost;
      iter_debug_info.dcost = dcost;
      iter_debug_info.lambda = lambda_hessian_;
      iter_debug_info.expect = expected;

      time_info_.t_forward_pass_ms +=
          time_info_.GetElapsed(time_info_.start, true);

      iter_debug_info.t_compute_deriv_ms = time_info_.t_compute_deriv_ms;
      iter_debug_info.t_backward_pass_ms = time_info_.t_backward_pass_ms;
      iter_debug_info.t_forward_pass_ms = time_info_.t_forward_pass_ms;

      UpdateAdvancedInfo(solver_info_.iter_count);

      double ratio{0.0};
      if (expected >= solver_config_->ilqr_config().cost_tol()) {
        ratio = dcost / expected;
      }
      if (forward_pass_success) {
        DeLambda(ratio);

        const double prev_iter_cost = cost_;

        cost_ = new_cost;

        update_success = true;
        if (solver_info_.solver_condition ==
            iLqrSolveCondition::EXPECTED_TOLERANCE) {
          solver_success = true;
          break;
        }

        if (dcost < solver_config_->ilqr_config().cost_tol()) {
          solver_success = true;
          solver_info_.solver_condition =
              iLqrSolveCondition::NORMAL_COST_TOLERANCE;
          break;
        }

        double cost_percent_tol{
            solver_config_->ilqr_config().cost_percent_tol()};
        if (solver_config_->ilqr_config().variable_cost_percent_tol()) {
          cost_percent_tol = GetCostPercent(iter);
        }
        if (dcost / prev_iter_cost < cost_percent_tol) {
          solver_success = true;
          solver_info_.solver_condition =
              iLqrSolveCondition::NORMAL_COST_PERCENT_TOLERANCE;
          break;
        }
      } else {
        IncreaseLambda();

        if (lambda_hessian_ > kLambdaMax) {
          solver_success = false;
          solver_info_.solver_condition = iLqrSolveCondition::LINESEARCH_FAIL;
          break;
        } else if (solver_info_.solver_condition ==
                   iLqrSolveCondition::NON_POSITIVE_EXPECT) {
          solver_success = false;
        }
      }

      if ((iter + 1) == GetMaxIter()) {
        solver_info_.solver_condition = iLqrSolveCondition::MAX_ITER_TERMINATE;
        solver_success = true;
      }
    }

    cost_manager_->UpdateCommonTerms(x_list_, u_list_);
    Duals();
    UpdatePenalties();

    CilqrIteration cilqr_iteration_tmp;
    const auto constraint_size = GetConstraintSize();
    ConVec lambda_value = ConVec::Zero();
    ConVec penalty_value = ConVec::Zero();
    ConVec violation = ConVec::Zero();
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      CilqrInfo cilqr_info_tmp;
      lambda_value = cost_manager_->GetDuals(i);
      penalty_value = cost_manager_->GetPenalties(i);
      ConVec temp_violation = cost_manager_->GetViolation(i);

      for (size_t j = 0; j < constraint_size; ++j) {
        cilqr_info_tmp.cilqr_info.emplace_back(lambda_value(j),
                                               penalty_value(j));
        if (temp_violation(j) > violation(j)) {
          violation(j) = temp_violation(j);
        }
      }
      cilqr_iteration_tmp.cilqr_iteration.emplace_back(cilqr_info_tmp);
    }

    for (size_t i = 0; i < constraint_size; ++i) {
      cilqr_iteration_tmp.max_violation.emplace_back(violation(i));
    }
    cilqr_info_.cilqr_solve.emplace_back(cilqr_iteration_tmp);
  }

  void Duals() {
    const auto horizon = GetHorizon();
    size_t i = 0;
    for (; i < horizon; ++i) {
      cost_manager_->Duals(x_list_[i], u_list_[i], i);
    }
    cost_manager_->Duals(x_list_[i], u_term_init_, i);
  }

  void UpdatePenalties() {
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      cost_manager_->UpdatePenalties(i);
    }
  }

  void ResetCilqrInfo() {
    CilqrIteration cilqr_iteration_tmp;
    const auto constraint_size = GetConstraintSize();
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      CilqrInfo cilqr_info_tmp;
      for (size_t j = 0; j < constraint_size; ++j) {
        cilqr_info_tmp.cilqr_info.emplace_back(0.0, 0.0);
      }
      cilqr_iteration_tmp.cilqr_iteration.emplace_back(cilqr_info_tmp);
    }
    for (size_t i = 0; i < constraint_size; ++i) {
      cilqr_iteration_tmp.max_violation.emplace_back(0.0);
    }
    cilqr_info_.cilqr_solve.emplace_back(cilqr_iteration_tmp);
  }

  void UpdateWarmStart(const std::vector<ControlVec> &warm_start_u_list) {
    cost_manager_->UpdateWarmStart(warm_start_u_list);
  }

  const StateVec *GetStateResultPtr() const { return &x_list_; }

  const ControlVec *GetControlResultPtr() const { return &u_list_; }

  const ILqrSolverInfo *GetSolverInfoPtr() { return &solver_info_; }

  const CilqrSolve *GetCilqrInfoPtr() { return &cilqr_info_; }
  const TimeInfo *GetTimeInfoPtr() { return &time_info_; }

  void UpdateDynamicsDerivatives() {
    using ResetEigenVec;
    ResetEigenVec(lx_vec_);
    ResetEigenVec(lu_vec_);
    ResetEigenVec(lxx_vec_);
    ResetEigenVec(lxu_vec_);
    ResetEigenVec(luu_vec_);
    size_t i = 0;
    const auto horizon = GetHorizon();
    for (; i < horizon; ++i) {
      cost_manager_->Model()->GetDynamicsDerivatives(x_list_[i], u_list_[i], i,
                                                     &fx_vec_[i], &fu_vec_[i]);
      cost_manager_->GetGradientHessian(x_list_[i], u_list_[i], i, &lx_vec_[i],
                                        &lu_vec_[i], &lxx_vec_[i], &lxu_vec_[i],
                                        &luu_vec_[i]);
      if (IsCilqrEnable()) {
        cost_manager_->GetConstraintGradientHessian(
            x_list_[i], u_list_[i], i, &lx_vec_[i], &lu_vec_[i], &lxx_vec_[i],
            &lxu_vec_[i], &luu_vec_[i]);
      }
    }
    cost_manager_->GetTerminalGradientHessian(x_list_[i], &lx_vec_[i],
                                              &lxx_vec_[i]);
    if (IsCilqrEnable()) {
      cost_manager_->GetConstraintTerminalGradientHessian(
          x_list_[i], &lx_vec_[i], &lxx_vec_[i]);
    }
  }
  void AddCost(const CostBase<M, N> *cost_term) {
    cost_manager_->AddCost(cost_term);
    solver_info_.cost_size++;
  }

  void SetConstraintSize(const size_t &constraint_size) {
    cost_manager_->SetConstraintSize(constraint_size);
  }

  void AddConstraint(const size_t &horizon,
                     const std::vector<BoundConstraint<M, N>> &constraint_vec) {
    cost_manager_->AddConstraint(horizon, constraint_vec);
  }

  void SetCallback(CILQRCallback callback) { cilqr_callback_ = callback; }

  const SolverConfig &GetSolverConfig() { return *solver_config_; }

  void UpdateAdvancedInfo(const size_t &iter) {
    solver_info_.iteration_info_vec[iter].x_vec = x_list_;
    solver_info_.iteration_info_vec[iter].u_vec = u_list_;
  }

  void ResetU() { ResetEigenVec(u_list_); }

  void InitIlqrSolverInfo(ILqrSolverInfo *ilqr_solver_info) const {
    ilqr_solver_info->solver_condition = iLqrSolveCondition::INIT;
    ilqr_solver_info->iter_count = 0;
    ilqr_solver_info->iteration_info_vec.clear();
    ilqr_solver_info->iteration_info_vec.resize(
        GetMaxIter() * GetMaxOuterIter() + 1);
  }

  size_t GetHorizon() const { return solver_config_->ilqr_config().horizon(); }
  size_t GetMaxIter() const { return solver_config_->ilqr_config().max_iter(); }
  size_t GetMaxOuterIter() const {
    return solver_config_->cilqr_config().max_outer_iterations();
  }
  int GetInitSeedIndex() const { return init_seed_index_; }
  bool IsCilqrEnable() const { return solver_config_->enable_cilqr(); }
  bool IsDebugMode() const { return solver_config_->is_debug_mode(); }
  size_t GetInputSize() const {
    return solver_config_->ilqr_config().input_size();
  }
  size_t GetStateSize() const {
    return solver_config_->ilqr_config().state_size();
  }
  const size_t GetConstraintSize() const {
    return cost_manager_->ConstraintSize();
  }

  void ComputeTime() {
    time_info_.t_one_step_ms =
        time_info_.GetElapsed(time_info_.all_start, false);
    solver_info_.total_time_ms = time_info_.t_one_step_ms;
    for (size_t i = 1; i < solver_info_.iter_count + 1; ++i) {
      solver_info_.t_compute_deriv_ms +=
          solver_info_.iteration_info_vec[i].t_compute_deriv_ms;
      solver_info_.t_backward_pass_ms +=
          solver_info_.iteration_info_vec[i].t_backward_pass_ms;
      solver_info_.t_forward_pass_ms +=
          solver_info_.iteration_info_vec[i].t_forward_pass_ms;
    }
  }

 private:
  bool BackwardPass() {
    const auto horizon = GetHorizon();
    Vx_ = lx_vec_[horizon];
    Vxx_ = lxx_vec_[horizon];

    dV_.fill(0);

    for (int i = (static_cast<int>(horizon) - 1); i >= 0; i--) {
      fut_ = fu_vec_[i].transpose();
      fxt_.noalias() = fx_vec_[i].transpose();
      fu_Vxx_.noalias() = fut_ * Vxx_;

      Qx_ = lx_vec_[i];
      Qx_.noalias() += fxt_ * Vx_;

      Qu_ = lu_vec_[i];
      Qu_.noalias() += fut_ * Vx_;

      Qxx_ = lxx_vec_[i];
      Qxx_.noalias() += (fxt_ * Vxx_ * fx_vec_[i]);

      Quu_ = luu_vec_[i];
      Quu_.noalias() += fu_Vxx_ * fu_vec_[i];

      Qux_ = lxu_vec_[i].transpose();
      Qux_.noalias() += fu_Vxx_ * fx_vec_[i];

      Quuf_ = Quu_;
      Quuf_.noalias() += lambda_hessian_ * Quu_eye_;
      if (!PSDCheck(Quuf_)) {
        return false;
      }

      QuuF_inv_ = Quuf_.inverse();
      k_i_ = -QuuF_inv_ * Qu_;
      K_i_ = -QuuF_inv_ * Qux_;

      Kt_ = K_i_.transpose();
      Kt_Quu_Quxt_ = Qux_.transpose();
      Kt_Quu_Quxt_.noalias() += Kt_ * Quu_;

      dV_[0] += k_i_.transpose() * Qu_;
      dV_[1] += 0.5 * (k_i_.transpose() * Quu_ * k_i_).value();

      Vx_ = Qx_;
      Vx_.noalias() += Kt_Quu_Quxt_ * k_i_;
      Vx_.noalias() += Kt_ * Qu_;
      Vxx_ = Qxx_;
      Vxx_.noalias() += Kt_Quu_Quxt_ * K_i_;
      Vxx_.noalias() += Kt_ * Qux_;

      k_vec_[i] = k_i_;
      K_vec_[i] = K_i_;
    }
    return true;
  }

  double GetCostPercent(const size_t iter) {
    constexpr size_t kStrictTolIter{2};
    constexpr size_t kLooseTolIter{6};
    const double base_tol{solver_config_->ilqr_config().cost_percent_tol()};
    if (iter <= kStrictTolIter) {
      return base_tol;
    } else if (iter <= kLooseTolIter) {
      return base_tol + (iter - kStrictTolIter) * base_tol;
    }
    return base_tol + (kLooseTolIter - kStrictTolIter) * base_tol;
  }

  bool ForwardPass(const size_t &iter, double *new_cost, double *expected) {
    xk_new_vec_ = x_list_;
    uk_new_vec_ = u_list_;

    const size_t alpha_size = alpha_vec.size();
    const size_t horizon = GetHorizon();
    for (size_t i = 0; i < alpha_size; ++i) {
      const double alpha = alpha_vec[i];
      solver_info_.iteration_info_vec[iter].linesearch_count = i + 1;
      (*new_cost) = 0.0;
      for (size_t j = 0; j < horizon; ++j) {
        uk_new_vec_[j] = u_list_[j];
        uk_new_vec_[j].noalias() += alpha * k_vec_[j];
        uk_new_vec_[j].noalias() += K_vec_[j] * (xk_new_vec_[j] - x_list_[j]);

        (*new_cost) +=
            cost_manager_->GetCost(xk_new_vec_[j], uk_new_vec_[j], j);
        if (IsCilqrEnable()) {
          (*new_cost) +=
              cost_manager_->GetAugLagCost(xk_new_vec_[j], uk_new_vec_[j], j);
        }
        xk_new_vec_[j + 1] = cost_manager_->Model()->UpdateDynamicsOneStep(
            xk_new_vec_[j], uk_new_vec_[j], j);
      }
      if (IsCilqrEnable()) {
        *new_cost += cost_manager_->GetTerminalConstraint(xk_new_vec_[horizon]);
      }
      (*new_cost) += cost_manager_->GetTerminalCost(xk_new_vec_[horizon]);

      (*expected) = -alpha * (dV_[0] + alpha * dV_[1]);

      if ((*expected) < 0.0) {
        solver_info_.solver_condition = NON_POSITIVE_EXPECT;
        return false;
      }
      if ((cost_ - (*new_cost)) > kZMin * (*expected)) {
        x_list_ = xk_new_vec_;
        u_list_ = uk_new_vec_;
        return true;
      }
      if (*expected < solver_config_->ilqr_config().cost_tol()) {
        *new_cost = cost_;
        solver_info_.solver_condition = EXPECTED_TOLERANCE;
        return true;
      }
    }
    return false;
  }

  void IncreaseLambda() {
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ * kLambdaFactor);
  }

  void LambdaForBackwardPass(const size_t fail_count) {
    constexpr size_t kMaxFailMultiplier{5};
    const size_t fail_multiplier{std::min(kMaxFailMultiplier, fail_count)};
    const double factor{kLambdaFactor * fail_multiplier};
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ * factor);
  }

  void DeLambda() {
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ / kLambdaFactor);
  }
  void DeLambda(const double ratio) {
    constexpr double kBetterRatio{0.75};
    constexpr double kGoodRatio{0.05};
    constexpr double kRegularLambdaFactor{7.6};
    constexpr double kReducedLambdaFactor{2.0};

    const double a{(kRegularLambdaFactor - kReducedLambdaFactor) /
                   (kBetterRatio - kGoodRatio)};
    const double b{kReducedLambdaFactor - a * kGoodRatio};

    double factor{kReducedLambdaFactor};
    if (ratio > kBetterRatio) {
      factor = kRegularLambdaFactor;
    } else if (ratio > kGoodRatio) {
      factor = a * ratio + b;
    }
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ / factor);
  }

  bool PSDCheck(const LuuMT &Q) {
    if (GetInputSize() == 1) {
      return Q(0, 0) > kLambdaMin;
    }
    if (GetInputSize() == 2) {
      return (Q(0, 0) > kLambdaMin) &&
             ((Q(0, 0) * Q(1, 1) - Q(0, 1) * Q(1, 0)) > kLambdaMin);
    }
    Eigen::LDLT<LuuMT> ldltOfQ(Q);
    return ldltOfQ.isPositive();

    return true;
  }

  double cost_ = 0.0;

  StateVec x_list_;
  ControlVec u_list_;
  StateVec xk_new_vec_;
  ControlVec uk_new_vec_;
  Control u_term_init_;

  double lambda_hessian_ = 1.0;
  double lambda_hessian_init_ = 1.0;
  int init_seed_index_ = -1;

  std::unique_ptr<CostManager<M, N, C>> cost_manager_;
};
}  // namespace spt
}  // namespace e2e_noa
