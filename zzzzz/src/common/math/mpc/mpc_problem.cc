#include "apps/planning/src/common/math/mpc/mpc_problem.h"

#include "OsqpEigen/OsqpEigen.h"
#include "apps/planning/src/common/math/mpc/utility.h"
#include "lqr/linear_quadratic_regulator.h"

namespace zark {
namespace planning {

MPCProblem::MPCProblem(const int n_steps, const int n_x, const int n_u,
                       const int n_u_dot, const Eigen::VectorXd& Q,
                       const Eigen::VectorXd& R, const Eigen::VectorXd& R_dot,
                       const double dt, const MPCData::Names& names,
                       const bool use_linear_slack_cost,
                       const bool enable_non_negative_slack_constraints)
    : n_steps_(n_steps),
      n_nodes_(n_steps + 1),
      n_x_(n_x),
      n_u_(n_u),
      n_u_dot_(n_u_dot),
      Q_(Q),
      R_(R),
      R_dot_(R_dot),
      dt_(dt),
      names_(names),
      use_linear_slack_cost_(use_linear_slack_cost),
      enable_non_negative_slack_constraints_(
          enable_non_negative_slack_constraints) {}

bool MPCProblem::Solve(const Eigen::VectorXd& x_init,
                       const Eigen::MatrixXd& x_ref,
                       const Eigen::MatrixXd& u_ref, const Eigen::MatrixXd& u_2,
                       const MPCData::Constraints& constraints,
                       const Eigen::VectorXd& u_prev, const double dt_prev,
                       MPCData& mpc_data) {
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = ComputeStateSpaceModel();

  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = ConstructCostFunction(Q_, R_, R_dot_, A_d.back(), B_d.back(),
                                         x_ref, u_ref, dt_, u_prev, dt_prev);

  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A, b_eq);

  Eigen::VectorXd b_ineq;
  ConstructInequalityConstraints(constraints, dt_, u_prev, dt_prev, A, b_ineq);

  ConvertToSoftConstraints(dt_, dt_prev, constraints, use_linear_slack_cost_, H,
                           f, A, b_eq, b_ineq);

  Eigen::VectorXd z;
  if (!Optimize(H, f, A, b_eq, b_ineq, z)) {
    return false;
  }

  mpc_data = ConvertToMPCData(z, x_ref, u_ref, constraints, dt_prev, u_prev);

  return true;
}

std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd>
MPCProblem::ConstructCostFunction(
    const Eigen::VectorXd& Q, const Eigen::VectorXd& R,
    const Eigen::VectorXd& R_dot, const Eigen::MatrixXd& A_d_terminal,
    const Eigen::MatrixXd& B_d_terminal, const Eigen::MatrixXd& x_ref,
    const Eigen::MatrixXd& u_ref, const double dt,
    const Eigen::VectorXd& u_prev, const double dt_prev) {
  std::vector<Eigen::Triplet<double>> triplet_list_for_H;
  triplet_list_for_H.reserve(this->n_steps_ * (this->n_x_ + this->n_u_) +
                             this->n_x_ * this->n_x_);

  auto UpdateCostForStateAndControl =
      [this](const Eigen::VectorXd& Q, const Eigen::VectorXd& R,
             const Eigen::MatrixXd& Q_terminal, const Eigen::MatrixXd& x_ref,
             const Eigen::MatrixXd& u_ref,
             std::vector<Eigen::Triplet<double>>& triplet_list_for_H,
             Eigen::VectorXd& f) {
        for (int k = 0; k < this->n_steps_; ++k) {  // stage cost
          const int i_x_start = k * this->n_x_;
          for (int i = 0; i < this->n_x_; ++i) {
            const int i_x = i_x_start + i;
            triplet_list_for_H.emplace_back(i_x, i_x, Q(i));
          }
          const int i_u_start = this->n_nodes_ * this->n_x_ + k * this->n_u_;
          for (int i = 0; i < this->n_u_; ++i) {
            const int i_u = i_u_start + i;
            triplet_list_for_H.emplace_back(i_u, i_u, R(i));
          }
        }
        const int i_x_start = this->n_steps_ * this->n_x_;  // terminal cost
        for (int i = 0; i < this->n_x_; ++i) {
          const int i_x = i_x_start + i;
          for (int j = 0; j < this->n_x_; ++j) {
            const int j_x = i_x_start + j;
            triplet_list_for_H.emplace_back(i_x, j_x, Q_terminal(i, j));
          }
        }

        f = Eigen::VectorXd::Zero(this->n_x_ * this->n_nodes_ +
                                  this->n_u_ * this->n_steps_);
        f <<  //
            -(Q.asDiagonal() * x_ref.leftCols(this->n_steps_))
                 .reshaped(),                  // stage cost
            -Q_terminal * x_ref.rightCols(1),  // terminal cost
            -(R * u_ref).reshaped();
      };

  auto UpdateCostForControlRate =
      [this](const Eigen::VectorXd& R_dot, const double dt,
             const double dt_prev, const Eigen::VectorXd& u_prev,
             std::vector<Eigen::Triplet<double>>& triplet_list_for_H,
             Eigen::VectorXd& f) {
        const double one_over_dt_sqr = 1.0 / (dt * dt);
        const double one_over_dt_prev_dt = 1.0 / (dt_prev * dt);
        const Eigen::VectorXd R_dot_diag_0 =
            R_dot * one_over_dt_prev_dt + R_dot * one_over_dt_sqr;
        const Eigen::VectorXd R_dot_diag = R_dot * (2.0 * one_over_dt_sqr);
        const Eigen::VectorXd R_dot_diag_terminal = R_dot * one_over_dt_sqr;
        const Eigen::VectorXd R_dot_off_diag = -R_dot * one_over_dt_sqr;
        for (int k = 0; k < this->n_steps_; ++k) {
          const int i_u_dot_start =
              this->n_nodes_ * this->n_x_ + k * this->n_u_dot_;
          for (int i = 0; i < this->n_u_dot_; ++i) {
            const int i_u_dot = i_u_dot_start + i;
            if (k == 0) {
              triplet_list_for_H.emplace_back(i_u_dot, i_u_dot,
                                              R_dot_diag_0(i));
            } else {
              triplet_list_for_H.emplace_back(i_u_dot, i_u_dot,
                                              (k < this->n_steps_ - 1)
                                                  ? R_dot_diag(i)
                                                  : R_dot_diag_terminal(i));
              triplet_list_for_H.emplace_back(i_u_dot - this->n_u_dot_, i_u_dot,
                                              R_dot_off_diag(i));
              triplet_list_for_H.emplace_back(i_u_dot, i_u_dot - this->n_u_dot_,
                                              R_dot_off_diag(i));
            }
          }
        }

        f.segment(this->n_nodes_ * this->n_x_, this->n_u_dot_) +=
            -(R_dot.asDiagonal() * u_prev * one_over_dt_prev_dt);
      };

  const int n_cost = this->n_x_ * this->n_nodes_ + this->n_u_ * this->n_steps_;
  Eigen::SparseMatrix<double> H(n_cost, n_cost);
  Eigen::VectorXd f(n_cost);

  UpdateCostForStateAndControl(Q, R, Q_terminal_, x_ref, u_ref,
                               triplet_list_for_H, f);

  UpdateCostForControlRate(R_dot, dt, dt_prev, u_prev, triplet_list_for_H, f);

  H.setFromTriplets(triplet_list_for_H.begin(), triplet_list_for_H.end());

  return std::make_pair(H, f);
}

void MPCProblem::ConstructEqualityConstraints(
    const Eigen::VectorXd& x_init, const std::vector<Eigen::MatrixXd>& A_d,
    const std::vector<Eigen::MatrixXd>& B_d,
    const std::vector<Eigen::MatrixXd>& B_d2, const Eigen::MatrixXd& u_2,
    Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b_eq) {
  const int n_eq = n_nodes_ * n_x_;
  const int n_dec_var = n_nodes_ * n_x_ + n_steps_ * n_u_;
  const int is_lti =
      (A_d.size() == 1 && B_d.size() == 1);  // linear-time-invariant (LTI)
  const int is_u2_valid = !B_d2.empty() && (static_cast<size_t>(u_2.cols()) ==
                                            B_d2.size());  // valid u_2 term

  A.resize(n_eq, n_dec_var);
  b_eq = Eigen::VectorXd::Zero(n_eq);
  auto AssignMatrixBlock = [](const Eigen::MatrixXd& A_in, const int i_start,
                              const int j_start,
                              Eigen::SparseMatrix<double>& A) {
    for (int i = 0; i < A_in.rows(); ++i) {
      for (int j = 0; j < A_in.cols(); ++j) {
        A.insert(i_start + i, j_start + j) = A_in(i, j);
      }
    }
  };
  for (int i = 0; i < n_eq; ++i) {
    A.insert(i, i) = -1.0;
  }
  for (int k = 0; k < n_steps_; ++k) {
    const int i_x_start = k * n_x_;
    const int i_u_start = n_nodes_ * n_x_ + k * n_u_;
    if (is_lti) {
      AssignMatrixBlock(A_d[0], i_x_start + n_x_, i_x_start, A);
      AssignMatrixBlock(B_d[0], i_x_start + n_x_, i_u_start, A);
    } else {
      AssignMatrixBlock(A_d[k], i_x_start + n_x_, i_x_start, A);
      AssignMatrixBlock(B_d[k], i_x_start + n_x_, i_u_start, A);
    }
    if (is_u2_valid) {
      b_eq.segment(i_x_start + n_x_, n_x_) = -B_d2[k] * u_2.col(k);
    }
  }

  b_eq.head(n_x_) << -x_init;
}

void MPCProblem::ConstructInequalityConstraints(
    const MPCData::Constraints& constraints, const double dt,
    const Eigen::VectorXd& u_prev, const double dt_prev,
    Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b_ineq) {
  const int n_dec_var = n_nodes_ * n_x_ + n_steps_ * n_u_;
  const int n_eq = n_nodes_ * n_x_;

  A.conservativeResize(n_eq + constraints.n_ineq, n_dec_var);
  b_ineq = Eigen::VectorXd::Zero(constraints.n_ineq);

  auto UpdateInequalityConstraintsForASingleType =  // a type can be:
                                                    // soft_min, soft_max,
                                                    // stiff_min, stiff_max,
                                                    // hard_min, hard_max
      [&dt, &u_prev, &dt_prev, &n_eq, this](
          const MPCData::Constraints::Constraint& constraint, const bool is_min,
          int& i_ineq_start, Eigen::SparseMatrix<double>& A_ineq,
          Eigen::VectorXd& b_ineq) {
        auto UpdateInequalityConstraintsForAStateType =  // a unit can be: x,
                                                         // u, u_dot
            [n_eq, this](
                const MPCData::Constraints::Constraint::ConstraintInfo& info,
                const int n_rows, const int n_cols, const bool is_min,
                int& i_ineq_start, MPCProblem::StateType type,
                Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b_ineq) {
              const int sign = is_min ? -1 : 1;
              switch (type) {
                case X:
                  for (int k = 0; k < n_cols; ++k) {
                    int i_curr = i_ineq_start + k * info.n_enabled_row;
                    for (int i = 0; i < n_rows; ++i) {
                      if (info.idx_enabled[i]) {
                        const int j_curr = k * n_rows + i;
                        A.insert(n_eq + i_curr, j_curr) = sign;
                        i_curr++;
                      }
                    }
                  }
                  break;
                case U:
                  for (int k = 0; k < n_cols; ++k) {
                    int i_curr = i_ineq_start + k * info.n_enabled_row;
                    for (int i = 0; i < n_rows; ++i) {
                      if (info.idx_enabled[i]) {
                        const int j_curr = n_eq + k * n_rows + i;
                        A.insert(n_eq + i_curr, j_curr) = sign;
                        i_curr++;
                      }
                    }
                  }
                  break;
                case U_DOT:
                  for (int k = 0; k < n_cols; ++k) {
                    int i_curr = i_ineq_start + k * info.n_enabled_row;
                    for (int i = 0; i < n_rows; ++i) {
                      if (info.idx_enabled[i]) {
                        const int j_curr = n_eq + k * n_rows + i;
                        A.insert(n_eq + i_curr, j_curr) = sign;
                        if (i_curr > i_ineq_start) {
                          A.insert(n_eq + i_curr, j_curr - n_u_dot_) = -sign;
                        }
                        i_curr++;
                      }
                    }
                  }
                  break;
                default:
                  AERROR << "Unknown state type";
                  break;
              }

              const int n_enabled = info.n_enabled_row * info.n_enabled_col;
              if (n_enabled > 0) {
                b_ineq.segment(i_ineq_start, n_enabled) =
                    info.constraint.reshaped() * (is_min ? -1.0 : 1.0);
              }

              i_ineq_start += n_enabled;
            };

        auto ConvertControlRateConstraint =
            [&dt, &dt_prev, &u_prev](const Eigen::MatrixXd& u_dot) {
              Eigen::MatrixXd u_dot_converted;
              if (u_dot.size() == 0) {
                return u_dot;
              }
              u_dot_converted = u_dot * dt;
              u_dot_converted.col(0) = u_prev + u_dot.col(0) * dt_prev;
              return u_dot_converted;
            };

        UpdateInequalityConstraintsForAStateType(
            constraint.x, this->n_x_, this->n_nodes_, is_min, i_ineq_start, X,
            A_ineq, b_ineq);

        UpdateInequalityConstraintsForAStateType(
            constraint.u, this->n_u_, this->n_steps_, is_min, i_ineq_start, U,
            A_ineq, b_ineq);

        auto u_dot_converted = constraint.u_dot;
        u_dot_converted.constraint =
            ConvertControlRateConstraint(u_dot_converted.constraint);
        UpdateInequalityConstraintsForAStateType(
            u_dot_converted, this->n_u_dot_, this->n_steps_, is_min,
            i_ineq_start, U_DOT, A_ineq, b_ineq);
      };

  int i_ineq_start = 0;
  UpdateInequalityConstraintsForASingleType(constraints.soft_min, true,
                                            i_ineq_start, A, b_ineq);
  UpdateInequalityConstraintsForASingleType(constraints.soft_max, false,
                                            i_ineq_start, A, b_ineq);
  UpdateInequalityConstraintsForASingleType(constraints.stiff_min, true,
                                            i_ineq_start, A, b_ineq);
  UpdateInequalityConstraintsForASingleType(constraints.stiff_max, false,
                                            i_ineq_start, A, b_ineq);
  UpdateInequalityConstraintsForASingleType(constraints.hard_min, true,
                                            i_ineq_start, A, b_ineq);
  UpdateInequalityConstraintsForASingleType(constraints.hard_max, false,
                                            i_ineq_start, A, b_ineq);
}

void MPCProblem::ConvertToSoftConstraints(
    const double dt, const double dt_prev,
    const MPCData::Constraints& constraints, const bool use_linear_slack_cost,
    Eigen::SparseMatrix<double>& H, Eigen::VectorXd& f,
    Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b_eq,
    Eigen::VectorXd& b_ineq) {
  use_linear_slack_cost ? AugmentLinearSlackCosts(constraints, H, f)  //
                        : AugmentQuadraticSlackCosts(constraints, H, f);

  AugmentEqualityConstraints(constraints, A, b_eq);

  AugmentInequalityConstraints(dt, dt_prev, constraints, A, b_ineq);
}

bool MPCProblem::Optimize(const Eigen::SparseMatrix<double>& H,
                          const Eigen::VectorXd& f,
                          const Eigen::SparseMatrix<double>& A,
                          const Eigen::VectorXd& b_eq,
                          const Eigen::VectorXd& b_ineq, Eigen::VectorXd& z) {
  const int n_dec_var = f.size();
  const int n_constraint = b_eq.size() + b_ineq.size();

  Eigen::VectorXd l(n_constraint);
  Eigen::VectorXd u(n_constraint);
  l << b_eq, -OSQP_INFTY * Eigen::VectorXd::Ones(b_ineq.size());
  u << b_eq, b_ineq;

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.settings()->setPolish(true);

  solver.data()->setNumberOfVariables(n_dec_var);
  solver.data()->setNumberOfConstraints(n_constraint);
  if (!solver.data()->setHessianMatrix(H)) {
    AERROR << "Set hessian matrix failed";
    return false;
  }
  if (!solver.data()->setGradient(const_cast<Eigen::VectorXd&>(f))) {
    AERROR << "Set gradient failed";
    return false;
  }
  if (!solver.data()->setLinearConstraintsMatrix(A)) {
    AERROR << "Set linear constraints matrix failed";
    return false;
  }
  if (!solver.data()->setLowerBound(l)) {
    AERROR << "Set lowerBound failed";
    return false;
  }
  if (!solver.data()->setUpperBound(u)) {
    AERROR << "Set upperBound failed";
    return false;
  }
  if (!solver.initSolver() ||
      solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    AERROR << "OSQP solver failed";
    return false;
  }

  z = solver.getSolution();

  return true;
}

MPCData MPCProblem::ConvertToMPCData(Eigen::VectorXd& z,
                                     const Eigen::MatrixXd& x_ref,
                                     const Eigen::MatrixXd& u_ref,
                                     const MPCData::Constraints& constraints,
                                     const double& dt_prev,
                                     const Eigen::VectorXd& u_prev) {
  MPCData mpc_data;
  mpc_data.names = names_;
  mpc_data.t = util::TVector(dt_, n_steps_ + 1);
  mpc_data.x_ref = x_ref;
  mpc_data.u_ref = u_ref;
  mpc_data.u_prev = u_prev;
  mpc_data.constraints = constraints;
  std::tie(mpc_data.x, mpc_data.u, mpc_data.u_dot, mpc_data.slacks) =
      ConvertSolution(constraints, dt_, u_prev, dt_prev, z);

  mpc_data.dt = dt_;
  mpc_data.dt_prev = dt_prev;
  mpc_data.n_x = n_x_;
  mpc_data.n_u = n_u_;
  mpc_data.n_u_dot = n_u_dot_;
  mpc_data.n_steps = n_steps_;

  return mpc_data;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, MPCData::Slacks>
MPCProblem::ConvertSolution(const MPCData::Constraints& constraints,
                            const double dt, const Eigen::VectorXd& u_prev,
                            const double dt_prev, const Eigen::VectorXd& z) {
  Eigen::MatrixXd x(n_x_, n_nodes_);
  Eigen::MatrixXd u(n_u_, n_steps_);
  Eigen::MatrixXd u_dot(n_u_dot_, n_steps_);
  MPCData::Slacks slack;

  int i_start = 0;
  x = z.head(n_x_ * n_nodes_).reshaped(n_x_, n_nodes_);
  i_start += n_x_ * n_nodes_;

  u = z.segment(i_start, n_u_ * n_steps_).reshaped(n_u_, n_steps_);
  u_dot << (u.col(0) - u_prev) / dt_prev,
      (u.rightCols(n_steps_ - 1) - u.leftCols(n_steps_ - 1)) / dt;
  i_start += n_u_ * n_steps_;

  auto AssignSlackGroup =
      [&z](const MPCData::Constraints::Constraint& constraint, int& i_start,
           MPCData::Slacks::Slack& slack_group) {
        auto AssignSingleSlackGroup =
            [&z](const MPCData::Constraints::Constraint::ConstraintInfo& info,
                 int& i_start, Eigen::MatrixXd& slack) {
              slack = Eigen::MatrixXd::Ones(info.idx_enabled.size(),
                                            info.n_enabled_col) *  //
                      std::numeric_limits<double>::quiet_NaN();
              const int n_enabled = info.n_enabled_row * info.n_enabled_col;
              int i_enabled = 0;
              for (size_t i = 0; i < info.idx_enabled.size(); ++i) {
                if (info.idx_enabled[i]) {
                  for (int j = 0; j < info.n_enabled_col; ++j) {
                    slack(i, j) =
                        z(i_start + j * info.n_enabled_row + i_enabled);
                  }
                  i_enabled++;
                }
              }
              i_start += n_enabled;
            };

        AssignSingleSlackGroup(constraint.x, i_start, slack_group.x);
        AssignSingleSlackGroup(constraint.u, i_start, slack_group.u);
        AssignSingleSlackGroup(constraint.u_dot, i_start, slack_group.u_dot);
      };

  AssignSlackGroup(constraints.soft_min, i_start, slack.soft_min);
  AssignSlackGroup(constraints.soft_max, i_start, slack.soft_max);
  AssignSlackGroup(constraints.stiff_min, i_start, slack.stiff_min);
  AssignSlackGroup(constraints.stiff_max, i_start, slack.stiff_max);

  return std::make_tuple(x, u, u_dot, slack);
}

void MPCProblem::AugmentLinearSlackCosts(
    const MPCData::Constraints& constraints, Eigen::SparseMatrix<double>& H,
    Eigen::VectorXd& f) {
  const int n_dec_var = n_nodes_ * n_x_ + n_steps_ * n_u_;
  const int n_dec_var_with_slacks = n_dec_var + constraints.n_ineq_soft;
  H.conservativeResize(n_dec_var_with_slacks, n_dec_var_with_slacks);

  auto AugmentLinearCostVector =
      [](const MPCData::Constraints::Constraint& constraint, int& i_start,
         Eigen::VectorXd& f) {
        auto AugmentSingleUnit =
            [](const MPCData::Constraints::Constraint::ConstraintInfo& info,
               int& i_start, Eigen::VectorXd& f) {
              const int n_enabled = info.n_enabled_row * info.n_enabled_col;
              f.conservativeResize(f.size() + n_enabled);
              for (int i = 0; i < info.n_enabled_col; ++i) {
                for (int j = 0; j < info.n_enabled_row; ++j) {
                  const int i_curr = i_start + i * info.n_enabled_row + j;
                  f(i_curr) = info.weight(j);
                }
              }
              i_start += n_enabled;
            };

        AugmentSingleUnit(constraint.x, i_start, f);
        AugmentSingleUnit(constraint.u, i_start, f);
        AugmentSingleUnit(constraint.u_dot, i_start, f);
      };

  int i_start = n_dec_var;
  AugmentLinearCostVector(constraints.soft_min, i_start, f);
  AugmentLinearCostVector(constraints.soft_max, i_start, f);
  AugmentLinearCostVector(constraints.stiff_min, i_start, f);
  AugmentLinearCostVector(constraints.stiff_max, i_start, f);
}

void MPCProblem::AugmentQuadraticSlackCosts(
    const MPCData::Constraints& constraints, Eigen::SparseMatrix<double>& H,
    Eigen::VectorXd& f) {
  const int n_dec_var = n_nodes_ * n_x_ + n_steps_ * n_u_;
  auto AugmentQuadraticCostMatrix =
      [](const MPCData::Constraints::Constraint& constraint, int& i_start,
         Eigen::SparseMatrix<double>& H) {
        auto AugmentSingleUnit =
            [](const MPCData::Constraints::Constraint::ConstraintInfo& info,
               int& i_start, Eigen::SparseMatrix<double>& H) {
              const int n_enabled = info.n_enabled_row * info.n_enabled_col;
              H.conservativeResize(H.rows() + n_enabled, H.cols() + n_enabled);
              for (int i = 0; i < info.n_enabled_col; ++i) {
                for (int j = 0; j < info.n_enabled_row; ++j) {
                  const int i_curr = i_start + i * info.n_enabled_row + j;
                  H.insert(i_curr, i_curr) = info.weight(j);
                }
              }
              i_start += n_enabled;
            };

        AugmentSingleUnit(constraint.x, i_start, H);
        AugmentSingleUnit(constraint.u, i_start, H);
        AugmentSingleUnit(constraint.u_dot, i_start, H);
      };

  int i_start = n_dec_var;
  AugmentQuadraticCostMatrix(constraints.soft_min, i_start, H);
  AugmentQuadraticCostMatrix(constraints.soft_max, i_start, H);
  AugmentQuadraticCostMatrix(constraints.stiff_min, i_start, H);
  AugmentQuadraticCostMatrix(constraints.stiff_max, i_start, H);

  f.conservativeResize(n_dec_var + constraints.n_ineq_soft);
  f.tail(constraints.n_ineq_soft)
      << Eigen::VectorXd::Zero(constraints.n_ineq_soft);
}

void MPCProblem::AugmentEqualityConstraints(
    const MPCData::Constraints& constraints, Eigen::SparseMatrix<double>& A,
    Eigen::VectorXd& b_eq) {
  const int n_eq = n_nodes_ * n_x_;
  const int n_dec_var_with_slack =
      n_nodes_ * n_x_ + n_steps_ * n_u_ + constraints.n_ineq_soft;
  A.conservativeResize(n_eq + constraints.n_ineq, n_dec_var_with_slack);
}

void MPCProblem::AugmentInequalityConstraints(
    const double dt, const double dt_prev,
    const MPCData::Constraints& constraints, Eigen::SparseMatrix<double>& A,
    Eigen::VectorXd& b_ineq) {
  const int n_eq = n_nodes_ * n_x_;
  const int n_dec_var = n_nodes_ * n_x_ + n_steps_ * n_u_;
  const int n_dec_var_with_slack = n_dec_var + constraints.n_ineq_soft;
  int i_start = 0;
  A.conservativeResize(n_eq + constraints.n_ineq, n_dec_var_with_slack);
  AugmentAIneqMatrix(dt, dt_prev, constraints.soft_min, i_start, A);
  AugmentAIneqMatrix(dt, dt_prev, constraints.soft_max, i_start, A);
  AugmentAIneqMatrix(dt, dt_prev, constraints.stiff_min, i_start, A);
  AugmentAIneqMatrix(dt, dt_prev, constraints.stiff_max, i_start, A);

  if (enable_non_negative_slack_constraints_) {
    A.conservativeResize(n_eq + constraints.n_ineq + constraints.n_ineq_soft,
                         n_dec_var_with_slack);
    for (int i = 0; i < constraints.n_ineq_soft; ++i) {
      A.insert(n_eq + constraints.n_ineq + i, n_dec_var + i) = -1.0;
    }
    b_ineq.conservativeResize(constraints.n_ineq + constraints.n_ineq_soft);
    b_ineq.tail(constraints.n_ineq_soft)
        << Eigen::VectorXd::Zero(constraints.n_ineq_soft);
  }
}

void MPCProblem::AugmentAIneqMatrix(
    const double dt, const double dt_prev,
    const MPCData::Constraints::Constraint& constraint, int& i_start,
    Eigen::SparseMatrix<double>& A) {
  const int n_eq = n_nodes_ * n_x_;
  const int n_dec_var = n_nodes_ * n_x_ + n_steps_ * n_u_;
  const int n_xu_enabled =
      constraint.x.n_enabled_row * constraint.x.n_enabled_col +  //
      constraint.u.n_enabled_row * constraint.u.n_enabled_col;
  const int n_u_dot_enabled =
      constraint.u_dot.n_enabled_row * constraint.u_dot.n_enabled_col;

  for (int i = 0; i < n_xu_enabled; ++i) {
    const int i_curr = i_start + i;
    A.insert(n_eq + i_curr, n_dec_var + i_curr) = -1.0;
  }
  i_start += n_xu_enabled;
  for (int i = 0; i < n_u_dot_enabled; ++i) {
    const int i_curr = i_start + i;
    A.insert(n_eq + i_curr, n_dec_var + i_curr) = -(i == 0 ? dt_prev : dt);
  }
  i_start += n_u_dot_enabled;
}

}  // namespace planning
}  // namespace zark
