#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <tuple>
#include <utility>
#include <vector>

#include "apps/planning/src/common/mpc_data.h"

namespace zark {
namespace planning {

/**
 * @brief This class solves the model predictive control (MPC)
 * problem. See the link below for documentation.
 *
 * https://wx6lpdt35j.feishu.cn/wiki/CDQewBXNrivNETkKDCAcV9jYn0d
 */
class MPCProblem {
  enum StateType { X, U, U_DOT };

 public:
  explicit MPCProblem(const int n_steps, const int n_x, const int n_u,
                      const int n_u_dot, const Eigen::VectorXd& Q,
                      const Eigen::VectorXd& R, const Eigen::VectorXd& R_dot,
                      const double dt, const MPCData::Names& names,
                      const bool use_linear_slack_cost = false,
                      const bool enable_non_negative_slack_constraints = false);

  virtual ~MPCProblem() = default;

  /**
   * @brief This is the public API to call to solve the MPC problem.
   *
   * @param x_init The initial state.
   * @param x_ref The reference state trajectory.
   * @param u_ref The reference control input trajectory.
   * @param u_2 The uncontrollable input trajectory.
   * @param constraints The set of MPC inequality constraints.
   * @param u_prev The control input applied in the previous planning cycle.
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param x The output state trajectory.
   * @param u The output control trajectory.
   * @param u_dot The output control rate trajectory.
   * @param slacks The output slack trajectory for all soft/stiff inequality
   * constraints.
   * @return true Successfully solved the problem.
   * @return false Failed the solve the problem.
   */
  bool Solve(const Eigen::VectorXd& x_init, const Eigen::MatrixXd& x_ref,
             const Eigen::MatrixXd& u_ref, const Eigen::MatrixXd& u_2,
             const MPCData::Constraints& constraints,
             const Eigen::VectorXd& u_prev, const double dt_prev,
             MPCData& mpc_data);

 protected:
  /**
   * @brief This virtual function defines the following discrete-time
   * state-space model in the MPC problem.
   *
   * x(k+1) = A_d(k)x(k) + B_d(k)u(k), k = 0, ..., N-1
   * x(0) = x_init
   *
   * It should be defined in the derived class.
   *
   * @return std::pair<std::vector<Eigen::MatrixXd>,
   * std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>> the A_d,
   * B_d, B_d2 matrix tuples for all time steps. If the model is linear
   * time-invariant (LTI), then the size of A_d and B_d should both be one.
   */
  virtual std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>,
                     std::vector<Eigen::MatrixXd>>
  ComputeStateSpaceModel() = 0;

 private:
  /**
   * The following function converts the MPC problem into a quadratic program
   * (QP) of the form:
   *
   *    min 1/2 * z'Hz + f'z
   *    s.t.  A_eq * z == b_eq
   *          A_ineq * z <= b_ineq
   */

  /**
   * @brief This function computes the cost function matrices H and f in the QP.
   *
   * @param Q The Q matrix for state penalty weight.
   * @param R The R matrix for control input penalty weight.
   * @param R_dot The R_dot matrix for control input rate penalty weight.
   * @param A_d_terminal The discrete-time state-space model matrices A_d at the
   * terminal time.
   * @param B_d_terminal The discrete-time state-space model matrices B_d at the
   * terminal time.
   * @param x_ref The reference state trajectory.
   * @param u_ref The reference control input trajectory.
   * @param dt The sampling time between time steps [s].
   * @param u_prev The control input applied in the previous planning cycle.
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @return std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd>
   */
  std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd> ConstructCostFunction(
      const Eigen::VectorXd& Q, const Eigen::VectorXd& R,
      const Eigen::VectorXd& R_dot, const Eigen::MatrixXd& A_d_terminal,
      const Eigen::MatrixXd& B_d_terminal, const Eigen::MatrixXd& x_ref,
      const Eigen::MatrixXd& u_ref, const double dt,
      const Eigen::VectorXd& u_prev, const double dt_prev);

  /**
   * @brief This function computes the equality constraint matrices A_eq and
   * b_eq.
   *
   * @param x_init The initial state.
   * @param A_d The discrete-time state-space model matrices A_d.
   * @param B_d The discrete-time state-space model matrices B_d.
   * @param A The inequality constraint matrix in the QP problem.
   * @param b_eq The equality constraint vector b_eq in the QP.
   */
  void ConstructEqualityConstraints(const Eigen::VectorXd& x_init,
                                    const std::vector<Eigen::MatrixXd>& A_d,
                                    const std::vector<Eigen::MatrixXd>& B_d,
                                    const std::vector<Eigen::MatrixXd>& B_d2,
                                    const Eigen::MatrixXd& u_2,
                                    Eigen::SparseMatrix<double>& A,
                                    Eigen::VectorXd& b_eq);

  /**
   * @brief This function computes the inequality constraint matrices A_ineq and
   * b_ineq.
   *
   * @param constraints The set of MPC inequality constraints.
   * @param dt The sampling time between time steps [s].
   * @param u_prev The control input applied in the previous planning cycle.
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param A The inequality constraint matrix in the QP problem.
   * @param b_ineq The inequality constraint vector b_ineq in the QP.
   */
  void ConstructInequalityConstraints(const MPCData::Constraints& constraints,
                                      const double dt,
                                      const Eigen::VectorXd& u_prev,
                                      const double dt_prev,
                                      Eigen::SparseMatrix<double>& A,
                                      Eigen::VectorXd& b_ineq);

  /**
   * @brief This function augment slack variables to the soft/stiff inequality
   * constraints.
   *
   * @param dt The sampling time between time steps [s].
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param constraints The set of MPC inequality constraints.
   * @param use_linear_slack_cost a flag to indicate whether to use linear slack
   * cost.
   * @param H See above.
   * @param f See above.
   * @param A See above.
   * @param b_eq See above.
   * @param b_ineq See above.
   */
  void ConvertToSoftConstraints(const double dt, const double dt_prev,
                                const MPCData::Constraints& constraints,
                                const bool use_linear_slack_cost,
                                Eigen::SparseMatrix<double>& H,
                                Eigen::VectorXd& f,
                                Eigen::SparseMatrix<double>& A,
                                Eigen::VectorXd& b_eq, Eigen::VectorXd& b_ineq);

  /**
   * @brief This function solves the QP defined above.
   *
   * @param H See above.
   * @param f See above.
   * @param A See above.
   * @param b_eq See above.
   * @param b_ineq See above.
   * @param z The solution to the QP.
   * @return bool flag to indicate whether the optimization is successful or not
   */
  bool Optimize(const Eigen::SparseMatrix<double>& H, const Eigen::VectorXd& f,
                const Eigen::SparseMatrix<double>& A,
                const Eigen::VectorXd& b_eq, const Eigen::VectorXd& b_ineq,
                Eigen::VectorXd& z);

  /**
   * @brief This function assign values to mpc_data .
   *
   * @param z See above.
   * @param constraints See above.
   * @param x_ref See above.
   * @param u_ref See above.
   * @param constraints See above.
   * @param dt_prev See above.
   * @param u_prev See above.
   */
  MPCData ConvertToMPCData(Eigen::VectorXd& z, const Eigen::MatrixXd& x_ref,
                           const Eigen::MatrixXd& u_ref,
                           const MPCData::Constraints& constraints,
                           const double& dt_prev,
                           const Eigen::VectorXd& u_prev);

  /**
   * @brief This function converts the QP solution to MPC variables.
   *
   * @param constraints The set of MPC inequality constraints.
   * @param dt The sampling time between time steps [s].
   * @param u_prev The control input applied in the previous planning cycle.
   * @param dt_prev The time elapsed since the previous planning cycle [s].
   * @param z The solution to the QP.
   * @return std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Slacks>
   * - x: The state trajectory.
   * - u: The control input trajectory.
   * - u_dot: The control input rate trajectory.
   * - slacks: The inequality constraint violation trajectories.
   */
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, MPCData::Slacks>
  ConvertSolution(const MPCData::Constraints& constraints, const double dt,
                  const Eigen::VectorXd& u_prev, const double dt_prev,
                  const Eigen::VectorXd& z);

  /**
   * @brief This function augments linear slack costs to the cost function.
   *
   * @param constraints See above.
   * @param H See above.
   * @param f See above.
   */
  void AugmentLinearSlackCosts(const MPCData::Constraints& constraints,
                               Eigen::SparseMatrix<double>& H,
                               Eigen::VectorXd& f);

  /**
   * @brief This function augments quadratic slack costs to the cost function.
   *
   * @param constraints See above.
   * @param H See above.
   * @param f See above.
   */
  void AugmentQuadraticSlackCosts(const MPCData::Constraints& constraints,
                                  Eigen::SparseMatrix<double>& H,
                                  Eigen::VectorXd& f);

  /**
   * @brief This function augments the equality constraint matrices A_eq and
   * b_eq.
   *
   * @param constraints See above.
   * @param A_eq See above.
   * @param b_eq See above.
   */
  void AugmentEqualityConstraints(const MPCData::Constraints& constraints,
                                  Eigen::SparseMatrix<double>& A_eq,
                                  Eigen::VectorXd& b_eq);

  /**
   * @brief This function augments the inequality constraint matrices A_ineq and
   * b_ineq.
   *
   * @param dt See above.
   * @param dt_prev See above.
   * @param constraints See above.
   * @param A See above.
   * @param b_ineq See above.
   */
  void AugmentInequalityConstraints(const double dt, const double dt_prev,
                                    const MPCData::Constraints& constraints,
                                    Eigen::SparseMatrix<double>& A,
                                    Eigen::VectorXd& b_ineq);
  /**
   * @brief This function augments the inequality constraint matrices A_ineq.
   *
   * @param dt See above.
   * @param dt_prev See above.
   * @param constraint See above.
   * @param i_start See above.
   * @param A See above.
   */
  void AugmentAIneqMatrix(const double dt, const double dt_prev,
                          const MPCData::Constraints::Constraint& constraint,
                          int& i_start, Eigen::SparseMatrix<double>& A);

 protected:
  const int n_steps_;  // number of forward simulation steps in the MPC problem
  const int n_nodes_;  // number of time steps in the MPC problem
  const int n_x_;      // number of states
  const int n_u_;      // number of control inputs
  const int n_u_dot_;  // number of control input rates
  const Eigen::VectorXd Q_;      // Q matrix for state penalty
  const Eigen::VectorXd R_;      // R matrix for control input penalty
  Eigen::MatrixXd Q_terminal_;   // Q_terminal matrix for terminal state penalty
  const Eigen::VectorXd R_dot_;  // R_dot matrix for control input rate penalty
  const double dt_;              // sampling time between time steps [s]
  const MPCData::Names names_;   // names of states, control,
                                 // and control rates.
  const bool use_linear_slack_cost_;  // whether to use linear slack cost
  const bool enable_non_negative_slack_constraints_;  // whether to enforce
                                                      // non-negative slack
                                                      // variable constraints

 protected:
  FRIEND_TEST(MPCProblemTest, TestSolve);
  FRIEND_TEST(MPCProblemTest, TestConstructCostFunction);
  FRIEND_TEST(MPCProblemTest, TestConstructEqualityConstraints);
  FRIEND_TEST(MPCProblemTest, TestConstructInequalityConstraints);
  FRIEND_TEST(MPCProblemTest, TestConvertToSoftConstraints);
  FRIEND_TEST(MPCProblemTest, TestOptimize);
  FRIEND_TEST(MPCProblemTest, TestConvertToMPCData);
  FRIEND_TEST(MPCProblemTest, TestConvertSolution);
  FRIEND_TEST(MPCProblemTest, TestComputeTerminalCostQ);
  FRIEND_TEST(MPCProblemTest, TestAugmentQuadraticSlackCosts);
  FRIEND_TEST(MPCProblemTest, TestAugmentEqualityConstraints);
  FRIEND_TEST(MPCProblemTest, TestAugmentInequalityConstraints);
  FRIEND_TEST(MPCProblemTest, TestAugmentAIneqMatrix);
};

}  // namespace planning
}  // namespace zark
