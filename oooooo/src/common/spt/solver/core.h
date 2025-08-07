#pragma once
#include <Eigen/Dense>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
// #include "common/spt/solver/bound_constraint.h"
#include "common/spt/solver/constraint.h"
#include "common/spt/solver/cost_base.h"
// #include "common/spt/solver/utils.h"
// #include "common/spt/solver/model_base.h"
#include <chrono>

namespace e2e_noa {
namespace spt {
template <uint8_t M, uint8_t N, uint8_t C>
class ILqr {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SOLVER_TYPES(M, N)
  using ConVec = Eigen::Matrix<double, C, 1>;

  // solver result and reason
  enum SolverResult {
    INIT,
    NORMAL_COST_TOLERANCE,
    NORMAL_COST_PERCENT_TOLERANCE,
    MAX_LINESEARCH_TERMINATE,
    MAX_ITER_TERMINATE,
    INIT_TERMINATE,
    BACKWARD_PASS_FAIL,
    NON_POSITIVE_EXPECT,
    LINESEARCH_FAIL,
    FAULT_INPUT_SIZE,
    EXPECTED_TOLERANCE
  };

  // solver info for each iteration
  struct IterationInfo {
    bool linesearch_success = false;
    size_t linesearch_count = 0;
    double lambda = 0.0;
    double cost = 0.0;
    double dcost = 0.0;
    double expect = 0.0;
    StateVec x_vec;
    ControlVec u_vec;
    double deriv_time = 0.0;
    double backward_time = 0.0;
    double forward_time = 0.0;
    std::vector<std::vector<double>> cost_map;
  };

  // solver info for all iterations
  struct SolverInfo {
    uint8_t solver_res = NORMAL_COST_TOLERANCE;
    size_t cost_size = 0;
    size_t iter_count = 0;
    double init_cost = 0.0;
    double total_time_ms = 0.0;
    double deriv_time = 0.0;
    double backward_time = 0.0;
    double forward_time = 0.0;
    std::vector<IterationInfo> iteration_info_vec;
    double init_guess_cost = 0.0;
    double init_aug_lag_cost = 0.0;
    double init_terminal_constraint_cost = 0.0;
  };

  struct CilqrElem {
    CilqrElem(double lambda_, double penalty_)
        : lambda(lambda_), penalty(penalty_) {}
    double lambda = 0.0;
    double penalty = 0.0;
  };

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

  struct TimeInfo {
    void Reset() {
      init_guess_time = 0.0;
      deriv_time = 0.0;
      backward_time = 0.0;
      forward_time = 0.0;
      one_step_time = 0.0;
    }

    double GetDt(std::chrono::time_point<std::chrono::system_clock> &start,
                 const bool is_overlay) {
      auto now = std::chrono::system_clock::now();
      auto dt =
          std::chrono::duration_cast<std::chrono::nanoseconds>(now - start)
              .count();

      if (is_overlay) {
        start = now;
      }

      return dt * 1e-6;
    }

    void UpdateAllStart() { all_start = std::chrono::system_clock::now(); }

    void UpdateStart() { start = std::chrono::system_clock::now(); }

    std::chrono::time_point<std::chrono::system_clock> all_start;
    std::chrono::time_point<std::chrono::system_clock> start;
    double init_guess_time = 0.0;
    double deriv_time = 0.0;
    double backward_time = 0.0;
    double forward_time = 0.0;
    double one_step_time = 0.0;
  };

 public:
  ILqr() = default;
  ~ILqr() = default;

  void Init(const ModelBase<M, N> *const model_ptr,
            const SolverConfig *const solver_config_ptr,
            CostBase<M, N> *const cost_common) {
    solver_config_ptr_ = solver_config_ptr;
    cost_manager_ptr_ = std::make_unique<CostManager<M, N, C>>(
        model_ptr, solver_config_ptr, cost_common);
    solver_info_.cost_size = 0;
  }

  void Solve(const State &x0) {
    time_info_.Reset();
    time_info_.UpdateAllStart();
    InitIlqrSolverInfo(&solver_info_);

    xk_vec_[0] = x0;

    ResetPenalties();
    ResetDuals();
    ResetCilqrInfo();

    cost_manager_ptr_->SetCostMapPtr(
        &(solver_info_.iteration_info_vec[0].cost_map));

    const bool update_init_seed = true;
    cost_manager_ptr_->InitGuess(update_init_seed, &xk_vec_, &uk_vec_, &cost_,
                                 &init_seed_index_);
    cost_manager_ptr_->UpdateCommonTerms(xk_vec_, uk_vec_);

    const auto horizon = GetHorizon();
    for (size_t i = 0; i < horizon; ++i) {
      cost_ += cost_manager_ptr_->GetAugLagCost(xk_vec_[i], uk_vec_[i], i);
    }
    cost_ += cost_manager_ptr_->GetTerminalConstraint(xk_vec_[horizon]);

    time_info_.init_guess_time += time_info_.GetDt(time_info_.all_start, false);

    UpdateAdvancedInfo(0);

    lambda_hessian_ = lambda_hessian_init_;

    // ilqr main iteration

    cilqr_info_.cilqr_solve.clear();
    const int max_outer_iter = GetMaxOuterIter();
    for (int iter = 0; iter < max_outer_iter; ++iter) {
      ILqrIteration();
      UpdateDuals();
      UpdatePenalties();

      RecordCilqrInfo();

      if (CheckViolation()) {
        cilqr_info_.iter_count = iter + 1;
        break;
      }
    }

    // one_step_time
    ComputeTime();
  }

  void UpdateDuals() {
    const auto horizon = GetHorizon();
    size_t i = 0;
    for (; i < horizon; ++i) {
      cost_manager_ptr_->UpdateDuals(xk_vec_[i], uk_vec_[i], i);
    }
    cost_manager_ptr_->UpdateDuals(xk_vec_[i], u_term_init_, i);
  }

  void UpdatePenalties() {
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      cost_manager_ptr_->UpdatePenalties(i);
    }
  }

  void ResetPenalties() {
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      cost_manager_ptr_->ResetPenalties(i);
    }
  }

  void ResetDuals() {
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      cost_manager_ptr_->ResetDuals(i);
    }
  }

  bool CheckViolation() {
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      if (!cost_manager_ptr_->CheckViolation(i)) {
        return false;
      }
    }
    return true;
  }

  void RecordCilqrInfo() {
    CilqrIteration cilqr_iteration_tmp;
    const auto constraint_size = GetConstraintSize();
    ConVec lambda_value = ConVec::Zero();
    ConVec penalty_value = ConVec::Zero();
    ConVec violation = ConVec::Zero();
    const auto horizon = GetHorizon();
    for (size_t i = 0; i <= horizon; ++i) {
      CilqrInfo cilqr_info_tmp;
      lambda_value = cost_manager_ptr_->GetDuals(i);
      penalty_value = cost_manager_ptr_->GetPenalties(i);
      ConVec temp_violation = cost_manager_ptr_->GetViolation(i);

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
    cost_manager_ptr_->UpdateWarmStart(warm_start_u_list);
  }

  const StateVec *GetStateResultPtr() const { return &xk_vec_; }

  const ControlVec *GetControlResultPtr() const { return &uk_vec_; }

  const SolverInfo *GetSolverInfoPtr() { return &solver_info_; }

  const CilqrSolve *GetCilqrInfoPtr() { return &cilqr_info_; }
  const TimeInfo *GetTimeInfoPtr() { return &time_info_; }

  void PrintSolverInfo() {
    if (!IsDebugMode()) {
      return;
    }
    std::cout << "--------------------------------------------------- "
                 "solver info "
                 "--------------------------------------------------- "
              << std::endl;

    // iteration info
    std::cout << "cost size = " << solver_info_.cost_size
              << ", init cost = " << std::setprecision(10)
              << solver_info_.init_cost
              << ", iteration count = " << solver_info_.iter_count
              << ", cilqr iteration count = " << cilqr_info_.iter_count
              << ", solver condition = "
              << static_cast<size_t>(solver_info_.solver_res) << std::endl;

    printf("%-12s %-32s %-32s %-32s %-18s %-14s %-14s\n", "iteration", "cost",
           "reduction", "expect", "lambda", "LS_count", "LS_success");

    for (size_t iter = 1; iter < solver_info_.iter_count + 1; ++iter) {
      printf("%-12ld %-32.17g %-32.17f %-32.17f %-18.7f %-14ld %-14d\n", iter,
             solver_info_.iteration_info_vec[iter].cost,
             solver_info_.iteration_info_vec[iter].dcost,
             solver_info_.iteration_info_vec[iter].expect,
             solver_info_.iteration_info_vec[iter].lambda,
             solver_info_.iteration_info_vec[iter].linesearch_count,
             solver_info_.iteration_info_vec[iter].linesearch_success);
    }
  }

  void PrintCostInfo() {
    if (!IsDebugMode()) {
      return;
    }

    std::cout << "cilqr_iteration: " << cilqr_info_.cilqr_solve.size()
              << std::endl;
    std::cout << "cilqr_check_violation: " << CheckViolation() << std::endl;

    std::cout << "cost vec info:" << std::endl;
    const auto cost_size = solver_info_.cost_size;
    const size_t cost_vec_size = solver_info_.iter_count + 1;
    std::vector<std::vector<double>> cost_vec_vec(
        cost_vec_size, std::vector<double>(cost_size, 0.0));

    for (size_t i = 0; i < cost_vec_size; ++i) {
      const auto &cost_map = solver_info_.iteration_info_vec[i].cost_map;
      for (size_t j = 0; j < cost_size; ++j) {
        for (const double cost : cost_map[j]) {
          cost_vec_vec[i][j] += cost;
        }
      }
    }

    for (size_t iter = 0; iter < cost_vec_size; ++iter) {
      if (iter == 0) {
        printf("cost_vec[init] = [ ");
      } else {
        printf("cost_vec[%ld]    = [ ", iter - 1);
      }

      for (const auto cost : cost_vec_vec[iter]) {
        printf("%.5f ", cost);
      }
      printf("]\n");
    }
    std::cout << std::endl;
  }

  void PrintTimeInfo() {
    // time info
    std::cout << "\n-----time info:" << std::endl;
    std::cout << "Total time: " << time_info_.one_step_time << std::endl;
    std::cout << "compute_derivatives: " << time_info_.deriv_time << std::endl;
    std::cout << "backward pass: " << time_info_.backward_time << std::endl;
    std::cout << "forward pass: " << time_info_.forward_time << std::endl;
    std::cout << "other stuff: "
              << time_info_.one_step_time -
                     (time_info_.deriv_time + time_info_.backward_time +
                      time_info_.forward_time)
              << std::endl;
    // record time cost info
    solver_info_.total_time_ms = time_info_.one_step_time;
  }

  void UpdateDynamicsDerivatives() {
    using e2e_noa::spt::ResetEigenVec;
    ResetEigenVec(lx_vec_);
    ResetEigenVec(lu_vec_);
    ResetEigenVec(lxx_vec_);
    ResetEigenVec(lxu_vec_);
    ResetEigenVec(luu_vec_);
    size_t i = 0;
    const auto horizon = GetHorizon();
    for (; i < horizon; ++i) {
      cost_manager_ptr_->Model()->GetDynamicsDerivatives(
          xk_vec_[i], uk_vec_[i], i, &fx_vec_[i], &fu_vec_[i]);
      cost_manager_ptr_->GetGradientAndHessian(
          xk_vec_[i], uk_vec_[i], i, &lx_vec_[i], &lu_vec_[i], &lxx_vec_[i],
          &lxu_vec_[i], &luu_vec_[i]);

      cost_manager_ptr_->GetConstraintGradientHessian(
          xk_vec_[i], uk_vec_[i], i, &lx_vec_[i], &lu_vec_[i], &lxx_vec_[i],
          &lxu_vec_[i], &luu_vec_[i]);
    }
    cost_manager_ptr_->GetTerminalGradientHessian(xk_vec_[i], &lx_vec_[i],
                                                  &lxx_vec_[i]);

    cost_manager_ptr_->GetConstraintTerminalGradientHessian(
        xk_vec_[i], &lx_vec_[i], &lxx_vec_[i]);
  }
  void AddCost(const CostBase<M, N> *cost_term) {
    cost_manager_ptr_->AddCost(cost_term);
    solver_info_.cost_size++;
  }

  void SetConstraintSize(const size_t &constraint_size) {
    cost_manager_ptr_->SetConstraintSize(constraint_size);
  }

  void AddConstraint(const size_t &horizon,
                     const std::vector<BoundConstraint<M, N>> &constraint_vec) {
    cost_manager_ptr_->AddConstraint(horizon, constraint_vec);
  }

  const SolverConfig &GetSolverConfig() { return *solver_config_ptr_; }

  // update cost_vec
  void UpdateAdvancedInfo(const size_t &iter) {
    solver_info_.iteration_info_vec[iter].x_vec = xk_vec_;
    solver_info_.iteration_info_vec[iter].u_vec = uk_vec_;
  }

  // reset solver
  void Reset() {
    using e2e_noa::spt::ResizeVecAndResetEigen;
    const size_t horizon = GetHorizon();
    ResizeVecAndResetEigen(xk_vec_, horizon + 1);      // add terminate
    ResizeVecAndResetEigen(xk_new_vec_, horizon + 1);  // add terminate
    ResizeVecAndResetEigen(uk_vec_, horizon);
    ResizeVecAndResetEigen(uk_new_vec_, horizon);
    ResizeVecAndResetEigen(lx_vec_, horizon + 1);  // add terminate
    ResizeVecAndResetEigen(lu_vec_, horizon);
    ResizeVecAndResetEigen(lxx_vec_, horizon + 1);  // add terminate
    ResizeVecAndResetEigen(lxu_vec_, horizon);
    ResizeVecAndResetEigen(luu_vec_, horizon);
    ResizeVecAndResetEigen(fx_vec_, horizon);
    ResizeVecAndResetEigen(fu_vec_, horizon);
    ResizeVecAndResetEigen(k_vec_, horizon);
    ResizeVecAndResetEigen(K_vec_, horizon);
    Qu_.setZero();
    Qx_.setZero();
    coeff_d_.setZero();
    Vx_.setZero();
    u_term_init_.setZero();
    Quu_.setZero();
    Qxx_.setZero();
    Qux_.setZero();
    Quuf_.setZero();
    coeff_k_.setZero();
    Quu_eye_.setIdentity();
    Vxx_.setZero();
    fxt_.setZero();
    fut_.setZero();
    fu_Vxx_.setZero();
    QuuF_inv_.setZero();
    Kt_Quu_Quxt_.setZero();
    Quxt_.setZero();
    Kt_Quu_.setZero();
    Kt_.setZero();

    cost_manager_ptr_->Reset();
    cost_manager_ptr_->InitControlVar();
    std::fill(dV_.begin(), dV_.end(), 0);
    cost_ = 0.0;
    lambda_hessian_ = 1.0;
    lambda_hessian_init_ = 1.0;
    init_seed_index_ = 1.0;
    cilqr_info_.iter_count = 0;
  }

  void ResetU() { e2e_noa::spt::ResetEigenVec(uk_vec_); }

  void InitIlqrSolverInfo(SolverInfo *ilqr_solver_info) const {
    ilqr_solver_info->solver_res = SolverResult::INIT;
    ilqr_solver_info->iter_count = 0;
    ilqr_solver_info->iteration_info_vec.clear();
    ilqr_solver_info->iteration_info_vec.resize(
        GetMaxIter() * GetMaxOuterIter() + 1);
  }

  size_t GetHorizon() const { return solver_config_ptr_->horizon; }
  size_t GetMaxIter() const { return solver_config_ptr_->max_iter; }
  size_t GetMaxOuterIter() const {
    return solver_config_ptr_->max_outer_iterations;
  }
  int GetInitSeedIndex() const { return init_seed_index_; }
  bool IsDebugMode() const { return solver_config_ptr_->is_debug_mode; }
  size_t GetInputSize() const { return solver_config_ptr_->input_size; }
  size_t GetStateSize() const { return solver_config_ptr_->state_size; }
  const size_t GetConstraintSize() const {
    return cost_manager_ptr_->ConstraintSize();
  }

  void ComputeTime() {
    time_info_.one_step_time = time_info_.GetDt(time_info_.all_start, false);
    solver_info_.total_time_ms = time_info_.one_step_time;
    for (size_t i = 1; i < solver_info_.iter_count + 1; ++i) {
      solver_info_.deriv_time += solver_info_.iteration_info_vec[i].deriv_time;
      solver_info_.backward_time +=
          solver_info_.iteration_info_vec[i].backward_time;
      solver_info_.forward_time +=
          solver_info_.iteration_info_vec[i].forward_time;
    }
  }

 private:
  /*
  INPUTS
  cx: 2x(N+1)          cu: 2x(N+1)
  cuu: nxnx(N+1)        cxx: nxnx(N+1)  cuu: 2x2x(N+1)
  fx: nxnx(N+1)        fu: nx2x(N+1)    fxx: none
  fxu: None            fuu: none        u: 2xT
  OUTPUTS
  Vx: nx(N+1)      Vxx: nxnx(N+1)      k:mxT
  K: mxnxT         dV: 2x1
  */

  bool BackwardPass() {
    // cost-to-go at end
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
      coeff_d_ = -QuuF_inv_ * Qu_;
      coeff_k_ = -QuuF_inv_ * Qux_;

      Kt_ = coeff_k_.transpose();
      Kt_Quu_Quxt_ = Qux_.transpose();
      Kt_Quu_Quxt_.noalias() += Kt_ * Quu_;

      // cost-to-go approximation
      dV_[0] += coeff_d_.transpose() * Qu_;
      dV_[1] += 0.5 * (coeff_d_.transpose() * Quu_ * coeff_d_).value();

      Vx_ = Qx_;
      Vx_.noalias() += Kt_Quu_Quxt_ * coeff_d_;
      Vx_.noalias() += Kt_ * Qu_;
      Vxx_ = Qxx_;
      Vxx_.noalias() += Kt_Quu_Quxt_ * coeff_k_;
      Vxx_.noalias() += Kt_ * Qux_;

      // save controls/gains
      k_vec_[i] = coeff_d_;
      K_vec_[i] = coeff_k_;
    }
    return true;
  }

  bool ILqrIteration() {
    using e2e_noa::spt::ResetEigenVec;
    if (xk_vec_.empty() || uk_vec_.empty()) {
      solver_info_.solver_res = SolverResult::FAULT_INPUT_SIZE;
      return false;
    }
    lambda_hessian_ = lambda_hessian_init_;
    solver_info_.init_guess_cost = 0.0;
    solver_info_.init_aug_lag_cost = 0.0;
    solver_info_.init_terminal_constraint_cost = 0.0;

    // init guess
    const bool update_init_seed = false;
    cost_manager_ptr_->InitGuess(update_init_seed, &xk_vec_, &uk_vec_, &cost_,
                                 &init_seed_index_);
    solver_info_.init_guess_cost = cost_;
    const auto horizon = GetHorizon();
    for (size_t i = 0; i < horizon; ++i) {
      solver_info_.init_aug_lag_cost +=
          cost_manager_ptr_->GetAugLagCost(xk_vec_[i], uk_vec_[i], i);
    }
    cost_ += solver_info_.init_aug_lag_cost;
    solver_info_.init_terminal_constraint_cost =
        cost_manager_ptr_->GetTerminalConstraint(xk_vec_[horizon]);
    cost_ += solver_info_.init_terminal_constraint_cost;

    solver_info_.init_cost = cost_;

    solver_info_.solver_res = SolverResult::INIT;

    ResetEigenVec(lx_vec_);
    ResetEigenVec(lu_vec_);
    ResetEigenVec(lxx_vec_);
    ResetEigenVec(lxu_vec_);
    ResetEigenVec(luu_vec_);

    // iteration loop
    bool solver_success = false;
    bool update_success = true;
    for (size_t iter = 0; iter < GetMaxIter(); ++iter) {
      time_info_.UpdateStart();

      solver_info_.iter_count += 1;

      cost_manager_ptr_->SetCostMapPtr(
          &(solver_info_.iteration_info_vec[solver_info_.iter_count].cost_map));

      if (update_success) {
        UpdateDynamicsDerivatives();
      }

      time_info_.deriv_time += time_info_.GetDt(time_info_.start, true);

      // STEP 1: backward pass
      size_t backward_pass_count = 0;
      while (true) {
        backward_pass_count++;
        // Update Vx, Vxx, l, L, dV with back_pass
        const bool is_converged = BackwardPass();
        if (is_converged) {
          break;
        }
        if (backward_pass_count >= kMaxBackwardPassCount) {
          solver_info_.solver_res = SolverResult::BACKWARD_PASS_FAIL;
          return false;
        }
        IncreaseLambdaForBackwardPass(backward_pass_count);
      }

      time_info_.backward_time += time_info_.GetDt(time_info_.start, true);
      // STEP 2: forward pass
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

      // calculate forward_time

      time_info_.forward_time += time_info_.GetDt(time_info_.start, true);

      iter_debug_info.deriv_time = time_info_.deriv_time;
      iter_debug_info.backward_time = time_info_.backward_time;
      iter_debug_info.forward_time = time_info_.forward_time;

      UpdateAdvancedInfo(solver_info_.iter_count);

      // STEP 3: check if terminate
      double ratio{0.0};
      if (expected >= solver_config_ptr_->cost_tol) {
        ratio = dcost / expected;
      }
      if (forward_pass_success) {
        DecreaseLambda(ratio);

        const double prev_iter_cost = cost_;

        cost_ = new_cost;

        update_success = true;
        if (solver_info_.solver_res == SolverResult::EXPECTED_TOLERANCE) {
          solver_success = true;
          break;
        }
        // terminate check
        if (dcost < solver_config_ptr_->cost_tol) {
          solver_success = true;
          solver_info_.solver_res = SolverResult::NORMAL_COST_TOLERANCE;
          break;
        }

        double cost_percent_tol{solver_config_ptr_->cost_percent_tol};
        if (solver_config_ptr_->variable_cost_percent_tol) {
          cost_percent_tol = GetCurrCostPercentTol(iter);
        }
        if (dcost / prev_iter_cost < cost_percent_tol) {
          solver_success = true;
          solver_info_.solver_res = SolverResult::NORMAL_COST_PERCENT_TOLERANCE;
          break;
        }
      } else {
        IncreaseLambda();

        if (lambda_hessian_ > kLambdaMax) {
          solver_success = false;
          solver_info_.solver_res = SolverResult::LINESEARCH_FAIL;
          break;
        } else if (solver_info_.solver_res ==
                   SolverResult::NON_POSITIVE_EXPECT) {
          solver_success = false;
        }
      }

      if ((iter + 1) == GetMaxIter()) {
        solver_info_.solver_res = SolverResult::MAX_ITER_TERMINATE;
        solver_success = true;
      }
    }  // end of iteration loop

    cost_manager_ptr_->UpdateCommonTerms(xk_vec_, uk_vec_);

    return solver_success;
  }

  double GetCurrCostPercentTol(const size_t iter) {
    constexpr size_t kStrictTolIter{2};
    constexpr size_t kLooseTolIter{6};
    const double base_tol{solver_config_ptr_->cost_percent_tol};
    if (iter <= kStrictTolIter) {
      return base_tol;
    } else if (iter <= kLooseTolIter) {
      return base_tol + (iter - kStrictTolIter) * base_tol;
    }
    return base_tol + (kLooseTolIter - kStrictTolIter) * base_tol;
  }

  // input x, u, dV, k
  // output success_flag, new_cost
  bool ForwardPass(const size_t &iter, double *new_cost, double *expected) {
    xk_new_vec_ = xk_vec_;
    uk_new_vec_ = uk_vec_;

    const size_t alpha_size = alpha_vec.size();
    const size_t horizon = GetHorizon();
    for (size_t i = 0; i < alpha_size; ++i) {
      const double alpha = alpha_vec[i];
      solver_info_.iteration_info_vec[iter].linesearch_count = i + 1;
      (*new_cost) = 0.0;
      for (size_t j = 0; j < horizon; ++j) {
        uk_new_vec_[j] = uk_vec_[j];
        uk_new_vec_[j].noalias() += alpha * k_vec_[j];
        uk_new_vec_[j].noalias() += K_vec_[j] * (xk_new_vec_[j] - xk_vec_[j]);

        (*new_cost) +=
            cost_manager_ptr_->GetCost(xk_new_vec_[j], uk_new_vec_[j], j);

        (*new_cost) +=
            cost_manager_ptr_->GetAugLagCost(xk_new_vec_[j], uk_new_vec_[j], j);

        xk_new_vec_[j + 1] = cost_manager_ptr_->Model()->UpdateDynamicsOneStep(
            xk_new_vec_[j], uk_new_vec_[j], j);
      }

      *new_cost +=
          cost_manager_ptr_->GetTerminalConstraint(xk_new_vec_[horizon]);

      (*new_cost) += cost_manager_ptr_->GetTerminalCost(xk_new_vec_[horizon]);

      (*expected) = -alpha * (dV_[0] + alpha * dV_[1]);
      if ((*expected) < 0.0) {
        solver_info_.solver_res = NON_POSITIVE_EXPECT;
        return false;
      }
      if ((cost_ - (*new_cost)) > kZMin * (*expected)) {
        xk_vec_ = xk_new_vec_;
        uk_vec_ = uk_new_vec_;
        return true;
      }
      if (*expected < solver_config_ptr_->cost_tol) {
        *new_cost = cost_;
        solver_info_.solver_res = EXPECTED_TOLERANCE;
        return true;
      }
    }
    return false;
  }

  void IncreaseLambda() {
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ * kLambdaFactor);
  }

  void IncreaseLambdaForBackwardPass(const size_t fail_count) {
    constexpr size_t kMaxFailMultiplier{5};
    const size_t fail_multiplier{std::min(kMaxFailMultiplier, fail_count)};
    const double factor{kLambdaFactor * fail_multiplier};
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ * factor);
  }

  void DecreaseLambda() {
    lambda_hessian_ = std::max(kLambdaMin, lambda_hessian_ / kLambdaFactor);
  }
  void DecreaseLambda(const double ratio) {
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

  bool PSDCheck(const Luu &Q) {
    if (GetInputSize() == 1) {
      return Q(0, 0) > kLambdaMin;
    }
    if (GetInputSize() == 2) {
      return (Q(0, 0) > kLambdaMin) &&
             ((Q(0, 0) * Q(1, 1) - Q(0, 1) * Q(1, 0)) > kLambdaMin);
    }
    Eigen::LDLT<Luu> ldltOfQ(Q);
    return ldltOfQ.isPositive();

    return true;
  }

  double cost_ = 0.0;

  StateVec xk_vec_;
  ControlVec uk_vec_;
  StateVec xk_new_vec_;
  ControlVec uk_new_vec_;
  Control u_term_init_;

  LxVec lx_vec_;
  LuVec lu_vec_;
  LxxVec lxx_vec_;
  LxuVec lxu_vec_;
  LuuVec luu_vec_;

  FxVec fx_vec_;
  FuVec fu_vec_;

  std::array<double, 2> dV_{};
  CoeffDVec k_vec_;
  CoeffKVec K_vec_;

  Lu Qu_;
  Lx Qx_;
  CoeffD coeff_d_;
  Lx Vx_;

  Luu Quu_;
  Lxx Qxx_;
  Lux Qux_;
  Luu Quuf_;
  CoeffK coeff_k_;
  Luu Quu_eye_;
  Lxx Vxx_;
  Fx fxt_;
  FuT fut_;
  FuT fu_Vxx_;
  Luu QuuF_inv_;
  Lxu Kt_Quu_Quxt_;
  Lxu Quxt_;
  Lx Kt_Quu_;
  CoeffKT Kt_;

  double lambda_hessian_ = 1.0;
  double lambda_hessian_init_ = 1.0;
  int init_seed_index_ = -1;

  // model
  std::unique_ptr<CostManager<M, N, C>> cost_manager_ptr_;

  // solver config
  const SolverConfig *solver_config_ptr_;

  // solver info
  SolverInfo solver_info_;
  CilqrSolve cilqr_info_;

  // time debug info
  TimeInfo time_info_;
};
}  // namespace spt
}  // namespace e2e_noa
