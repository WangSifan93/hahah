//////////////////////////////////////////////////////////////////////////////
//
// Core LQ game solver from Basar and Olsder, "Preliminary Notation for
// Corollary 6.1" (pp. 279). All notation matches the text, though we
// shall assume that `c` (additive drift in dynamics) is always `0`, which
// holds because these dynamics are for delta x, delta us.
//
// Solve a time-varying, finite horizon LQ game (finds closed-loop Nash
// feedback strategies for both players).
//
// Assumes that dynamics are given by
//           ``` dx_{k+1} = A_k dx_k + \sum_i Bs[i]_k du[i]_k ```
//
// NOTE: Bs, Qs, ls, R1s, R2s are all lists of lists of matrices.
// NOTE: all indices of inner lists correspond to the "current time" k except
// for those of the Qs, which correspond to the "next time" k+1. That is,
// the kth entry of Qs[i] is the state cost corresponding to time step k+1. This
// makes sense because there is no point assigning any state cost to the
// initial state x_0.
//
// Returns strategies Ps, alphas.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/solver/lq_feedback_solver.h"

#include <glog/logging.h>

#include <vector>

#include "ilq_games/include/utils/linear_dynamics_approximation.h"
#include "ilq_games/include/utils/quadratic_cost_approximation.h"
#include "ilq_games/include/utils/strategy.h"

// 添加Eigen的LDLT分解头文件
#include <Eigen/Cholesky>

namespace e2e_noa::planning {

std::vector<Strategy> LQFeedbackSolver::Solve(
    const std::vector<LinearDynamicsApproximation> &linearization,
    const std::vector<std::vector<QuadraticCostApproximation>>
        &quadraticization,
    const VectorXf &x0, std::vector<VectorXf> *delta_xs,
    std::vector<std::vector<VectorXf>> *costates) {
  CHECK_EQ(linearization.size(), num_time_steps_);
  CHECK_EQ(quadraticization.size(), num_time_steps_);

  const PlayerIndex num_players = dynamics_->NumPlayers();
  const Dimension xdim = dynamics_->XDim();

  if (delta_xs) CHECK_NOTNULL(costates);
  if (costates) CHECK_NOTNULL(delta_xs);
  if (delta_xs) {
    delta_xs->resize(num_time_steps_);
    costates->resize(num_time_steps_);
    for (size_t kk = 0; kk < num_time_steps_; kk++) {
      (*delta_xs)[kk].resize(xdim);
      (*costates)[kk].resize(num_players);
      for (PlayerIndex ii = 0; ii < num_players; ii++) {
        (*costates)[kk][ii].resize(xdim);
      }
    }
  }
  std::vector<Strategy> strategies;
  strategies.reserve(num_players);
  for (PlayerIndex ii = 0; ii < num_players; ii++) {
    strategies.emplace_back(num_time_steps_, xdim, dynamics_->UDim(ii));
  }

  // 预分配中间变量
  std::vector<MatrixXf> BiZi_cache(num_players);

  for (PlayerIndex ii = 0; ii < num_players; ii++) {
    BiZi_cache[ii].resize(dynamics_->UDim(ii), xdim);
  }
  // 预计算控制维度起始索引
  std::vector<Dimension> udim_starts(num_players + 1);
  udim_starts[0] = 0;
  for (PlayerIndex ii = 0; ii < num_players; ii++) {
    udim_starts[ii + 1] = udim_starts[ii] + dynamics_->UDim(ii);
  }

  // 初始化终端
  const std::vector<QuadraticCostApproximation> &final_quad =
      quadraticization.back();
  for (PlayerIndex ii = 0; ii < num_players; ii++) {
    Zs_[num_time_steps_ - 1][ii] = final_quad[ii].state.hess;
    zetas_[num_time_steps_ - 1][ii] = final_quad[ii].state.grad;
  }
  Eigen::PartialPivLU<MatrixXf> solver;

  // 反向时间递推
  for (int kk = num_time_steps_ - 2; kk >= 0; kk--) {
    const LinearDynamicsApproximation &lin = linearization[kk];
    const std::vector<QuadraticCostApproximation> &quad = quadraticization[kk];

    // 预计算 Bi^T * Z_{k+1,ii}
    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      BiZi_cache[ii].noalias() = lin.Bs[ii].transpose() * Zs_[kk + 1][ii];
    }

    // 填充S矩阵和Y矩阵
    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      const Dimension row_start = udim_starts[ii];
      const Dimension u_dim_ii = dynamics_->UDim(ii);

      for (PlayerIndex jj = 0; jj < num_players; jj++) {
        const Dimension col_start = udim_starts[jj];
        const Dimension u_dim_jj = dynamics_->UDim(jj);

        if (ii == jj) {
          S_.block(row_start, col_start, u_dim_ii, u_dim_jj).noalias() =
              BiZi_cache[ii] * lin.Bs[jj] + quad[ii].control.at(ii).hess;
        } else {
          S_.block(row_start, col_start, u_dim_ii, u_dim_jj).noalias() =
              BiZi_cache[ii] * lin.Bs[jj];
        }
      }

      // 填充Y矩阵
      Y_.block(row_start, 0, u_dim_ii, xdim).noalias() = BiZi_cache[ii] * lin.A;
      Y_.col(xdim).segment(row_start, u_dim_ii).noalias() =
          lin.Bs[ii].transpose() * zetas_[kk + 1][ii] +
          quad[ii].control.at(ii).grad;
    }

    // 自适应正则化
    if (adaptive_regularization_) {
      for (size_t ii = 0; ii < S_.cols(); ii++) {
        const float radius = S_.col(ii).lpNorm<1>() - std::abs(S_(ii, ii));
        if (S_(ii, ii) - radius < 1e-3f) {
          S_(ii, ii) += radius + 1e-3f;
        }
      }
    }

    solver.compute(S_);
    X_ = solver.solve(Y_);
    // 设置策略
    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      strategies[ii].Ps[kk] = Ps_[ii];
      strategies[ii].alphas[kk] = alphas_[ii];
    }

    // 计算F和beta
    F_.noalias() = lin.A;
    beta_.setZero();
    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      F_.noalias() -= lin.Bs[ii] * Ps_[ii];
      beta_.noalias() -= lin.Bs[ii] * alphas_[ii];
    }

    // 更新Zs和zetas
    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      zetas_[kk][ii].noalias() =
          F_.transpose() * (zetas_[kk + 1][ii] + Zs_[kk + 1][ii] * beta_) +
          quad[ii].state.grad;
      Zs_[kk][ii].noalias() =
          F_.transpose() * Zs_[kk + 1][ii] * F_ + quad[ii].state.hess;

      for (const auto &Rij_entry : quad[ii].control) {
        const PlayerIndex jj = Rij_entry.first;
        const MatrixXf &Rij = Rij_entry.second.hess;
        const VectorXf &rij = Rij_entry.second.grad;

        zetas_[kk][ii].noalias() +=
            Ps_[jj].transpose() * (Rij * alphas_[jj] - rij);
        Zs_[kk][ii].noalias() += Ps_[jj].transpose() * Rij * Ps_[jj];
      }
    }
  }

  // 前向计算delta_xs和costates (优化终点处理)
  if (delta_xs) {
    VectorXf x_star = x0;
    // 处理非终点时间步
    for (size_t kk = 0; kk < num_time_steps_ - 1; kk++) {
      (*delta_xs)[kk] = x_star;

      for (PlayerIndex ii = 0; ii < num_players; ii++) {
        (*costates)[kk][ii].noalias() =
            -Zs_[kk + 1][ii] * x_star - zetas_[kk + 1][ii];
      }

      const auto &lin = linearization[kk];
      VectorXf next_x = lin.A * x_star;
      for (PlayerIndex ii = 0; ii < num_players; ii++) {
        next_x.noalias() -= lin.Bs[ii] * strategies[ii].alphas[kk];
      }
      x_star = std::move(next_x);
    }

    // 单独处理终点
    (*delta_xs)[num_time_steps_ - 1] = x_star;
    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      (*costates)[num_time_steps_ - 1][ii].setZero();
    }
  }
  return strategies;
}

}  // namespace e2e_noa::planning