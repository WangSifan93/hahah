#include "apps/planning/src/common/math/mpc/mpc_problem.h"

#include "apps/planning/src/common/math/mpc/longitudinal_mpc_problem.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

void ConstructXandURef(const Eigen::VectorXd &x_init, const double dt,
                       Eigen::MatrixXd &x_ref, Eigen::MatrixXd &u_ref) {
  const int n_steps = 25;
  const int n_nodes = n_steps + 1;
  const double a = -2.0;
  x_ref.resize(2, n_nodes);
  u_ref.resize(1, n_steps);
  for (int i = 0; i < n_nodes; ++i) {
    const double time = i * dt;
    x_ref(0, i) = x_init(0) + x_init(1) * time + 0.5 * a * time * time;
    x_ref(1, i) = x_init(1) + a * time;
    if (i < n_steps) {
      u_ref(0, i) = a;
    }
  }
}

void ConstructConstrains(zark::planning::MPCData::Constraints &constraints) {
  constraints.n_ineq = 382;
  constraints.n_ineq_hard = 26;
  constraints.n_ineq_soft = 356;
  // soft min
  constraints.soft_min.n_ineq = 76;
  // soft min x
  constraints.soft_min.x.idx_enabled = {1, 0};
  constraints.soft_min.x.n_enabled_row = 1;
  constraints.soft_min.x.n_enabled_col = 26;
  constraints.soft_min.x.weight.resize(1);
  constraints.soft_min.x.weight << 10.0;
  constraints.soft_min.x.constraint = Eigen::MatrixXd::Ones(1, 26) * -1.0e-5;
  // soft min u
  constraints.soft_min.u.idx_enabled = {1};
  constraints.soft_min.u.n_enabled_row = 1;
  constraints.soft_min.u.n_enabled_col = 25;
  constraints.soft_min.u.weight.resize(1);
  constraints.soft_min.u.weight << 10.0;
  constraints.soft_min.u.constraint = Eigen::MatrixXd::Ones(1, 25) * -2.0;
  // soft min u_dot
  constraints.soft_min.u_dot.idx_enabled = {1};
  constraints.soft_min.u_dot.n_enabled_row = 1;
  constraints.soft_min.u_dot.n_enabled_col = 25;
  constraints.soft_min.u_dot.weight.resize(1);
  constraints.soft_min.u_dot.weight << 10.0;
  constraints.soft_min.u_dot.constraint = Eigen::MatrixXd::Ones(1, 25) * -3.0;

  // soft  max
  constraints.soft_max.n_ineq = 102;
  // soft  max x
  constraints.soft_max.x.idx_enabled = {1, 1};
  constraints.soft_max.x.n_enabled_row = 2;
  constraints.soft_max.x.n_enabled_col = 26;
  constraints.soft_max.x.weight.resize(2);
  constraints.soft_max.x.weight << 10.0, 10.0;
  Eigen::VectorXd constrant_x(2);
  constrant_x << 25.0, 15.0;
  constraints.soft_max.x.constraint =
      constrant_x * Eigen::MatrixXd::Ones(1, 26);
  // soft max u
  constraints.soft_max.u.idx_enabled = {1};
  constraints.soft_max.u.n_enabled_row = 1;
  constraints.soft_max.u.n_enabled_col = 25;
  constraints.soft_max.u.weight.resize(1);
  constraints.soft_max.u.weight << 50.0;
  constraints.soft_max.u.constraint.resize(1, 25);
  constraints.soft_max.u.constraint << 0.5, 0.572, 0.644000000000000,
      0.716000000000000, 0.788000000000000, 0.860000000000000,
      0.982264150943396, 1.10452830188679, 1.22679245283019, 1.34905660377359,
      1.47132075471698, 1.59358490566038, 1.66111111111111, 1.63740740740741,
      1.61370370370370, 1.59000000000000, 1.57200000000000, 1.55400000000000,
      1.53600000000000, 1.51800000000000, 1.50000000000000, 1.48000000000000,
      1.46000000000000, 1.44000000000000, 1.42000000000000;
  // soft max u_dot
  constraints.soft_max.u_dot.idx_enabled = {1};
  constraints.soft_max.u_dot.n_enabled_row = 1;
  constraints.soft_max.u_dot.n_enabled_col = 25;
  constraints.soft_max.u_dot.weight.resize(1);
  constraints.soft_max.u_dot.weight << 50.0;
  constraints.soft_max.u_dot.constraint = Eigen::MatrixXd::Ones(1, 25) * 1.5;

  // stiff min
  constraints.stiff_min.n_ineq = 76;
  // stiff min x
  constraints.stiff_min.x.idx_enabled = {1, 0};
  constraints.stiff_min.x.n_enabled_row = 1;
  constraints.stiff_min.x.n_enabled_col = 26;
  constraints.stiff_min.x.weight.resize(1);
  constraints.stiff_min.x.weight << 500.0;
  constraints.stiff_min.x.constraint = Eigen::MatrixXd::Ones(1, 26) * -1.0e-5;
  // stiff min u
  constraints.stiff_min.u.idx_enabled = {1};
  constraints.stiff_min.u.n_enabled_row = 1;
  constraints.stiff_min.u.n_enabled_col = 25;
  constraints.stiff_min.u.weight.resize(1);
  constraints.stiff_min.u.weight << 500.0;
  constraints.stiff_min.u.constraint = Eigen::MatrixXd::Ones(1, 25) * -6.0;
  // stiff min u_dot
  constraints.stiff_min.u_dot.idx_enabled = {1};
  constraints.stiff_min.u_dot.n_enabled_row = 1;
  constraints.stiff_min.u_dot.n_enabled_col = 25;
  constraints.stiff_min.u_dot.weight.resize(1);
  constraints.stiff_min.u_dot.weight << 500.0;
  constraints.stiff_min.u_dot.constraint = Eigen::MatrixXd::Ones(1, 25) * -6.0;

  // stiff  max
  constraints.stiff_max.n_ineq = 102;
  // stiff  max x
  constraints.stiff_max.x.idx_enabled = {1, 1};
  constraints.stiff_max.x.n_enabled_row = 2;
  constraints.stiff_max.x.n_enabled_col = 26;
  constraints.stiff_max.x.weight.resize(2);
  constraints.stiff_max.x.weight << 500.0, 500.0;
  Eigen::VectorXd constrant_x_stiff_max(2);
  constrant_x_stiff_max << 26.0, 18.0;
  constraints.stiff_max.x.constraint =
      constrant_x_stiff_max * Eigen::MatrixXd::Ones(1, 26);
  // stiff max u
  constraints.stiff_max.u.idx_enabled = {1};
  constraints.stiff_max.u.n_enabled_row = 1;
  constraints.stiff_max.u.n_enabled_col = 25;
  constraints.stiff_max.u.weight.resize(1);
  constraints.stiff_max.u.weight << 500.0;
  constraints.stiff_max.u.constraint.resize(1, 25);
  constraints.stiff_max.u.constraint << 0.900000000000000, 0.990000000000000,
      1.08000000000000, 1.17000000000000, 1.26000000000000, 1.35000000000000,
      1.46320754716981, 1.57641509433962, 1.68962264150943, 1.80283018867925,
      1.91603773584906, 2.02924528301887, 2.09444444444444, 2.07962962962963,
      2.06481481481481, 2.05000000000000, 2.02000000000000, 1.99000000000000,
      1.96000000000000, 1.93000000000000, 1.90000000000000, 1.88000000000000,
      1.86000000000000, 1.84000000000000, 1.82000000000000;
  // stiff max u_dot
  constraints.stiff_max.u_dot.idx_enabled = {1};
  constraints.stiff_max.u_dot.n_enabled_row = 1;
  constraints.stiff_max.u_dot.n_enabled_col = 25;
  constraints.stiff_max.u_dot.weight.resize(1);
  constraints.stiff_max.u_dot.weight << 500.0;
  constraints.stiff_max.u_dot.constraint = Eigen::MatrixXd::Ones(1, 25) * 5.0;

  // hard min
  constraints.hard_min.n_ineq = 26;
  // hard min x
  constraints.hard_min.x.idx_enabled = {0, 1};
  constraints.hard_min.x.n_enabled_row = 1;
  constraints.hard_min.x.n_enabled_col = 26;
  constraints.hard_min.x.constraint = Eigen::MatrixXd::Ones(1, 26) * -1.0e-5;
  // hard min u
  constraints.hard_min.u.idx_enabled = {0};
  constraints.hard_min.u.n_enabled_row = 0;
  constraints.hard_min.u.n_enabled_col = 25;
  // hard min u_dot
  constraints.hard_min.u_dot.idx_enabled = {0};
  constraints.hard_min.u_dot.n_enabled_row = 0;
  constraints.hard_min.u_dot.n_enabled_col = 25;

  // hard max
  constraints.hard_max.n_ineq = 0;
  // hard max x
  constraints.hard_max.x.idx_enabled = {0, 0};
  constraints.hard_max.x.n_enabled_row = 0;
  constraints.hard_max.x.n_enabled_col = 26;
  // hard max u
  constraints.hard_max.u.idx_enabled = {0};
  constraints.hard_max.u.n_enabled_row = 0;
  constraints.hard_max.u.n_enabled_col = 25;
  // hard max u_dot
  constraints.hard_max.u_dot.idx_enabled = {0};
  constraints.hard_max.u_dot.n_enabled_row = 0;
  constraints.hard_max.u_dot.n_enabled_col = 25;
}

class MPCProblemTest : public ::testing::Test {
 public:
  MPCProblemTest() {
    int n_steps = 25;
    int n_x = 2;
    int n_u = 1;
    int n_u_dot = 1;
    Eigen::VectorXd Q(n_x);
    Q << 0.1, 1.0;
    Eigen::VectorXd R(n_u);
    R << 1.0;
    Eigen::VectorXd R_dot(n_u_dot);
    R_dot << 1.0;
    double dt = 0.2;
    mpc_problem_ = std::make_unique<LongitudinalMPCProblem>(
        n_steps, n_x, n_u, n_u_dot, Q, R, R_dot, dt);
  }

 protected:
  std::unique_ptr<LongitudinalMPCProblem> mpc_problem_;
};

TEST_F(MPCProblemTest, TestSolve) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  Eigen::MatrixXd u_2;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  MPCData mpc_data;
  mpc_problem_->Solve(x_init, x_ref, u_ref, u_2, constraints, u_prev, dt_prev,
                      mpc_data);
  EXPECT_EQ(mpc_data.x.rows(), 2);
  EXPECT_EQ(mpc_data.x.cols(), 26);
  EXPECT_EQ(mpc_data.u.rows(), 1);
  EXPECT_EQ(mpc_data.u.cols(), 25);
  EXPECT_EQ(mpc_data.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_min.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.soft_min.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.soft_min.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_min.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_min.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_min.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_max.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.soft_max.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.soft_max.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_max.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_max.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_max.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_min.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.stiff_min.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_max.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.stiff_max.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u_dot.cols(), 25);
}

TEST_F(MPCProblemTest, TestComputeStateSpaceModel) {
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  EXPECT_EQ(A_d.size(), 1);
  EXPECT_EQ(B_d.size(), 1);
  EXPECT_EQ(B_d2.size(), 0);
  EXPECT_EQ(A_d[0](0, 0), 1.0);
  EXPECT_EQ(A_d[0](0, 1), 0.2);
  EXPECT_EQ(A_d[0](1, 0), 0.0);
  EXPECT_EQ(A_d[0](1, 1), 1.0);
  EXPECT_EQ(B_d[0](0, 0), 0.0);
  EXPECT_EQ(B_d[0](1, 0), 0.2);
}

TEST_F(MPCProblemTest, TestConstructCostFunction) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = mpc_problem_->ConstructCostFunction(
      Q, R, R_dot, A_d.back(), B_d.back(), x_ref, u_ref, dt, u_prev, dt_prev);
  EXPECT_EQ(H.rows(), 77);
  EXPECT_EQ(H.cols(), 77);
  EXPECT_NEAR(H.coeff(0, 0), 0.1, 0.01);
  EXPECT_NEAR(H.coeff(0, 1), 0.0, 0.01);
  EXPECT_NEAR(H.coeff(1, 1), 1.0, 0.01);
  EXPECT_NEAR(H.coeff(2, 2), 0.1, 0.01);
  EXPECT_NEAR(H.coeff(3, 3), 1.0, 0.01);
  EXPECT_NEAR(H.coeff(48, 48), 0.1, 0.01);
  EXPECT_NEAR(H.coeff(49, 49), 1.0, 0.01);
  EXPECT_NEAR(H.coeff(50, 50), 2.07, 0.1);
  EXPECT_NEAR(H.coeff(50, 51), 1.73, 0.1);
  EXPECT_NEAR(H.coeff(51, 50), 1.73, 0.1);
  EXPECT_NEAR(H.coeff(51, 51), 7.19, 0.1);
  EXPECT_NEAR(H.coeff(52, 52), 76.0, 0.1);
  EXPECT_NEAR(H.coeff(53, 52), -25.0, 0.1);
  EXPECT_NEAR(H.coeff(52, 53), -25.0, 0.1);
  EXPECT_NEAR(H.coeff(53, 53), 51.0, 0.1);
  EXPECT_NEAR(H.coeff(76, 76), 26.0, 0.1);

  EXPECT_EQ(f.rows(), 77);
  EXPECT_EQ(f.cols(), 1);
  EXPECT_NEAR(f(0, 0), 0.0, 0.1);
  EXPECT_NEAR(f(1, 0), -10.0, 0.1);
  EXPECT_NEAR(f(2, 0), -0.196, 0.1);
  EXPECT_NEAR(f(3, 0), -9.6, 0.1);
  EXPECT_NEAR(f(49, 0), -0.4, 0.1);
  EXPECT_NEAR(f(50, 0), -51.75, 2.0);
  EXPECT_NEAR(f(51, 0), -43.36, 2.0);
  EXPECT_NEAR(f(52, 0), 2.0, 0.01);
  EXPECT_NEAR(f(76, 0), 2.0, 0.01);
}

TEST_F(MPCProblemTest, TestConstructEqualityConstraints) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  Eigen::MatrixXd u_2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  EXPECT_EQ(A.rows(), 52);
  EXPECT_EQ(A.cols(), 77);
  EXPECT_NEAR(A.coeff(0, 0), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(2, 0), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(1, 1), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(2, 1), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(3, 1), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(2, 2), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(4, 2), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(4, 3), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(50, 49), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(3, 52), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(51, 76), 0.2, 0.01);
  EXPECT_EQ(b_eq.rows(), 52);
  EXPECT_EQ(b_eq.cols(), 1);
  EXPECT_NEAR(b_eq(0, 0), 0.0, 0.01);
  EXPECT_NEAR(b_eq(1, 0), -10.0, 0.01);
  EXPECT_NEAR(b_eq(2, 0), 0.0, 0.01);
  EXPECT_NEAR(b_eq(51, 0), 0.0, 0.01);
}

TEST_F(MPCProblemTest, TestConstructInequalityConstraints) {
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  double dt = 0.2;
  int n_eq = 52;
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_ineq;
  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  EXPECT_EQ(A.rows(), 382 + n_eq);
  EXPECT_EQ(A.cols(), 77);
  EXPECT_NEAR(A.coeff(n_eq + 0, 0), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(n_eq + 76, 0), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(n_eq + 178, 0), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(n_eq + 254, 0), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(n_eq + 1, 2), -1.0, 0.01);  // X
  EXPECT_NEAR(A.coeff(n_eq + 2, 4), -1.0, 0.01);  // X
  EXPECT_NEAR(A.coeff(n_eq + 263, 9), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(n_eq + 18, 36), -1.0, 0.01);  // X
  EXPECT_NEAR(A.coeff(n_eq + 291, 37), 1.0, 0.01);  // X
  EXPECT_NEAR(A.coeff(n_eq + 355, 76), 1.0, 0.01);
  EXPECT_EQ(b_ineq.rows(), 382);
  EXPECT_EQ(b_ineq.cols(), 1);
  EXPECT_NEAR(b_ineq(0, 0), 0.0, 0.01);
  EXPECT_NEAR(b_ineq(25, 0), 0.0, 0.01);
  EXPECT_NEAR(b_ineq(26, 0), 2.0, 0.01);
  EXPECT_NEAR(b_ineq(50, 0), 2.0, 0.01);
  EXPECT_NEAR(b_ineq(51, 0), 0.3, 0.01);
  EXPECT_NEAR(b_ineq(52, 0), 0.6, 0.01);
}

TEST_F(MPCProblemTest, TestConvertToSoftConstraints) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  Eigen::MatrixXd u_2;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = mpc_problem_->ConstructCostFunction(
      Q, R, R_dot, A_d.back(), B_d.back(), x_ref, u_ref, dt, u_prev, dt_prev);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  Eigen::VectorXd b_ineq;
  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  mpc_problem_->ConvertToSoftConstraints(dt, dt_prev, constraints, false, H, f,
                                         A, b_eq, b_ineq);
  EXPECT_EQ(H.rows(), 433);
  EXPECT_EQ(H.cols(), 433);
  EXPECT_EQ(f.rows(), 433);
  EXPECT_EQ(f.cols(), 1);
  EXPECT_EQ(b_eq.rows(), 52);
  EXPECT_EQ(b_eq.cols(), 1);
  EXPECT_EQ(A.rows(), 434);
  EXPECT_EQ(A.cols(), 433);
}

TEST_F(MPCProblemTest, TestOptimize) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  Eigen::MatrixXd u_2;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = mpc_problem_->ConstructCostFunction(
      Q, R, R_dot, A_d.back(), B_d.back(), x_ref, u_ref, dt, u_prev, dt_prev);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  Eigen::VectorXd b_ineq;
  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  mpc_problem_->ConvertToSoftConstraints(dt, dt_prev, constraints, false, H, f,
                                         A, b_eq, b_ineq);
  Eigen::VectorXd z;
  bool status = mpc_problem_->Optimize(H, f, A, b_eq, b_ineq, z);
  EXPECT_EQ(status, true);
  EXPECT_EQ(z.rows(), 433);
  EXPECT_EQ(z.cols(), 1);
  EXPECT_NEAR(z(0, 0), 0.0, 0.01);
  EXPECT_NEAR(z(1, 0), 10.0, 0.01);
  EXPECT_NEAR(z(2, 0), 2.0, 0.01);
  EXPECT_NEAR(z(3, 0), 9.9334, 0.01);
  EXPECT_NEAR(z(4, 0), 3.9867, 0.01);
  EXPECT_NEAR(z(5, 0), 9.7291, 0.01);
  EXPECT_NEAR(z(6, 0), 5.9324, 0.01);
  EXPECT_NEAR(z(5, 0), 9.7291, 0.01);
  EXPECT_NEAR(z(6, 0), 5.9324, 0.01);
  EXPECT_NEAR(z(50, 0), 25.4894, 0.01);
  EXPECT_NEAR(z(51, 0), 0.0, 0.01);
  EXPECT_NEAR(z(52, 0), -0.3354, 0.01);
  EXPECT_NEAR(z(53, 0), -1.016, 0.01);
  EXPECT_NEAR(z(58, 0), -2.609, 0.01);
  EXPECT_NEAR(z(76, 0), -1.32, 0.01);
  EXPECT_NEAR(z(106, 0), 0.2197, 0.01);
  EXPECT_NEAR(z(107, 0), 0.4818, 0.01);
  EXPECT_NEAR(z(121, 0), 0.0, 0.01);
  EXPECT_NEAR(z(128, 0), 0.3499, 0.01);
  EXPECT_NEAR(z(432, 0), 0.0, 0.01);
}

TEST_F(MPCProblemTest, TestConvertToMPCData) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  Eigen::MatrixXd u_2;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = mpc_problem_->ConstructCostFunction(
      Q, R, R_dot, A_d.back(), B_d.back(), x_ref, u_ref, dt, u_prev, dt_prev);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  Eigen::VectorXd b_ineq;
  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  mpc_problem_->ConvertToSoftConstraints(dt, dt_prev, constraints, false, H, f,
                                         A, b_eq, b_ineq);
  Eigen::VectorXd z;
  mpc_problem_->Optimize(H, f, A, b_eq, b_ineq, z);
  MPCData mpc_data = mpc_problem_->ConvertToMPCData(
      z, x_ref, u_ref, constraints, dt_prev, u_prev);
  EXPECT_EQ(mpc_data.names.x[0], "s");
  EXPECT_EQ(mpc_data.names.x[1], "v");
  EXPECT_EQ(mpc_data.t.size(), 26);
  EXPECT_EQ(mpc_data.x.rows(), 2);
  EXPECT_EQ(mpc_data.x.cols(), 26);
  EXPECT_EQ(mpc_data.u.rows(), 1);
  EXPECT_EQ(mpc_data.u.cols(), 25);
  EXPECT_EQ(mpc_data.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_min.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.soft_min.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.soft_min.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_min.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_min.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_min.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_max.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.soft_max.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.soft_max.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_max.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.soft_max.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.soft_max.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_min.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.stiff_min.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_min.u_dot.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_max.x.rows(), 2);
  EXPECT_EQ(mpc_data.slacks.stiff_max.x.cols(), 26);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u.cols(), 25);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u_dot.rows(), 1);
  EXPECT_EQ(mpc_data.slacks.stiff_max.u_dot.cols(), 25);
}

TEST_F(MPCProblemTest, TestConvertSolution) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  Eigen::MatrixXd u_2;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = mpc_problem_->ConstructCostFunction(
      Q, R, R_dot, A_d.back(), B_d.back(), x_ref, u_ref, dt, u_prev, dt_prev);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  Eigen::VectorXd b_ineq;
  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  mpc_problem_->ConvertToSoftConstraints(dt, dt_prev, constraints, false, H, f,
                                         A, b_eq, b_ineq);
  Eigen::VectorXd z;
  mpc_problem_->Optimize(H, f, A, b_eq, b_ineq, z);
  Eigen::MatrixXd x;
  Eigen::MatrixXd u;
  Eigen::MatrixXd u_dot;
  zark::planning::MPCData::Slacks slacks;
  std::tie(x, u, u_dot, slacks) =
      mpc_problem_->ConvertSolution(constraints, dt, u_prev, dt_prev, z);
  EXPECT_EQ(x.rows(), 2);
  EXPECT_EQ(x.cols(), 26);
  EXPECT_NEAR(x(0, 0), 0.0, 0.01);
  EXPECT_NEAR(x(0, 1), 2.0, 0.01);
  EXPECT_NEAR(x(0, 2), 3.98, 0.01);
  EXPECT_NEAR(x(0, 3), 5.93, 0.01);
  EXPECT_NEAR(x(0, 25), 25.4869, 0.01);
  EXPECT_NEAR(x(1, 0), 10.0, 0.01);
  EXPECT_NEAR(x(1, 1), 9.9334, 0.01);
  EXPECT_NEAR(x(1, 2), 9.7301, 0.01);
  EXPECT_NEAR(x(1, 3), 9.3994, 0.01);
  EXPECT_NEAR(x(1, 25), 0.0, 0.01);
  EXPECT_EQ(u.rows(), 1);
  EXPECT_EQ(u.cols(), 25);
  EXPECT_NEAR(u(0, 0), -0.332, 0.01);
  EXPECT_NEAR(u(0, 1), -1.016, 0.01);
  EXPECT_NEAR(u(0, 2), -1.6537, 0.01);
  EXPECT_NEAR(u(0, 3), -2.2197, 0.01);
  EXPECT_NEAR(u(0, 24), -1.3248, 0.01);
  EXPECT_EQ(u_dot.rows(), 1);
  EXPECT_EQ(u_dot.cols(), 25);
  EXPECT_NEAR(u_dot(0, 0), -3.3518, 0.01);
  EXPECT_NEAR(u_dot(0, 1), -3.4156, 0.01);
  EXPECT_NEAR(u_dot(0, 2), -3.1827, 0.01);
  EXPECT_NEAR(u_dot(0, 3), -2.8234, 0.01);
  EXPECT_NEAR(u_dot(0, 24), 0.2375, 0.01);
  EXPECT_EQ(slacks.soft_min.x.rows(), 2);
  EXPECT_EQ(slacks.soft_min.x.cols(), 26);
  EXPECT_NEAR(slacks.soft_min.x(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_min.x(0, 25), 0.0, 0.01);
  EXPECT_EQ(slacks.soft_min.u.rows(), 1);
  EXPECT_EQ(slacks.soft_min.u.cols(), 25);
  EXPECT_NEAR(slacks.soft_min.u(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_min.u(0, 3), 0.2199, 0.01);
  EXPECT_NEAR(slacks.soft_min.u(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.soft_min.u_dot.rows(), 1);
  EXPECT_EQ(slacks.soft_min.u_dot.cols(), 25);
  EXPECT_NEAR(slacks.soft_min.u_dot(0, 0), 0.349995, 0.01);
  EXPECT_NEAR(slacks.soft_min.u_dot(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_min.u_dot(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.soft_max.x.rows(), 2);
  EXPECT_EQ(slacks.soft_max.x.cols(), 26);
  EXPECT_NEAR(slacks.soft_max.x(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_max.x(0, 22), 0.1614, 0.01);
  EXPECT_NEAR(slacks.soft_max.x(0, 25), 0.489, 0.01);
  EXPECT_EQ(slacks.soft_max.u.rows(), 1);
  EXPECT_EQ(slacks.soft_max.u.cols(), 25);
  EXPECT_NEAR(slacks.soft_max.u(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_max.u(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_max.u(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.soft_max.u_dot.rows(), 1);
  EXPECT_EQ(slacks.soft_max.u_dot.cols(), 25);
  EXPECT_NEAR(slacks.soft_max.u_dot(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_max.u_dot(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.soft_max.u_dot(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.stiff_min.x.rows(), 2);
  EXPECT_EQ(slacks.stiff_min.x.cols(), 26);
  EXPECT_NEAR(slacks.stiff_min.x(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_min.x(0, 25), 0.0, 0.01);
  EXPECT_EQ(slacks.stiff_min.u.rows(), 1);
  EXPECT_EQ(slacks.stiff_min.u.cols(), 25);
  EXPECT_NEAR(slacks.stiff_min.u(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_min.u(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_min.u(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.stiff_min.u_dot.rows(), 1);
  EXPECT_EQ(slacks.stiff_min.u_dot.cols(), 25);
  EXPECT_NEAR(slacks.stiff_min.u_dot(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_min.u_dot(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_min.u_dot(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.stiff_max.x.rows(), 2);
  EXPECT_EQ(slacks.stiff_max.x.cols(), 26);
  EXPECT_NEAR(slacks.stiff_max.x(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_max.x(0, 25), 0.0, 0.01);
  EXPECT_EQ(slacks.stiff_max.u.rows(), 1);
  EXPECT_EQ(slacks.stiff_max.u.cols(), 25);
  EXPECT_NEAR(slacks.stiff_max.u(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_max.u(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_max.u(0, 24), 0.0, 0.01);
  EXPECT_EQ(slacks.stiff_max.u_dot.rows(), 1);
  EXPECT_EQ(slacks.stiff_max.u_dot.cols(), 25);
  EXPECT_NEAR(slacks.stiff_max.u_dot(0, 0), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_max.u_dot(0, 3), 0.0, 0.01);
  EXPECT_NEAR(slacks.stiff_max.u_dot(0, 24), 0.0, 0.01);
}

TEST_F(MPCProblemTest, TestAugmentQuadraticSlackCosts) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  double dt = 0.2;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd u_ref;
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  ConstructXandURef(x_init, dt, x_ref, u_ref);
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd f;
  std::tie(H, f) = mpc_problem_->ConstructCostFunction(
      Q, R, R_dot, A_d.back(), B_d.back(), x_ref, u_ref, dt, u_prev, dt_prev);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  mpc_problem_->AugmentQuadraticSlackCosts(constraints, H, f);
  EXPECT_EQ(H.rows(), 433);
  EXPECT_EQ(H.cols(), 433);
  EXPECT_NEAR(H.coeff(76, 76), 26.0, 0.01);
  EXPECT_NEAR(H.coeff(77, 77), 10.0, 0.01);
  EXPECT_NEAR(H.coeff(204, 204), 10.0, 0.01);
  EXPECT_NEAR(H.coeff(205, 205), 50.0, 0.01);
  EXPECT_NEAR(H.coeff(254, 254), 50.0, 0.01);
  EXPECT_NEAR(H.coeff(255, 255), 500.0, 0.01);
  EXPECT_NEAR(H.coeff(432, 432), 500.0, 0.01);
  EXPECT_EQ(f.rows(), 433);
  EXPECT_EQ(f.cols(), 1);
  EXPECT_NEAR(f(0, 0), 0.0, 0.01);
  EXPECT_NEAR(f(1, 0), -10.0, 0.01);
  EXPECT_NEAR(f(2, 0), -0.196, 0.01);
  EXPECT_NEAR(f(3, 0), -9.6, 0.01);
  EXPECT_NEAR(f(49, 0), -0.4, 0.01);
  EXPECT_NEAR(f(50, 0), -51.75, 2.0);
  EXPECT_NEAR(f(51, 0), -43.36, 2.0);
  EXPECT_NEAR(f(52, 0), 2.0, 0.01);
  EXPECT_NEAR(f(76, 0), 2.0, 0.01);
  EXPECT_NEAR(f(77, 0), 0.0, 0.01);
  EXPECT_NEAR(f(177, 0), 0.0, 0.01);
  EXPECT_NEAR(f(432, 0), 0.0, 0.01);
}

TEST_F(MPCProblemTest, TestAugmentEqualityConstraints) {
  Eigen::VectorXd Q(2);
  Q << 0.1, 1.0;
  Eigen::VectorXd R(1);
  R << 1.0;
  Eigen::VectorXd R_dot(1);
  R_dot << 1.0;
  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  Eigen::MatrixXd u_2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  mpc_problem_->AugmentEqualityConstraints(constraints, A, b_eq);
  EXPECT_EQ(A.rows(), 434);
  EXPECT_EQ(A.cols(), 433);
  EXPECT_NEAR(A.coeff(0, 0), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(2, 0), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(1, 1), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(2, 1), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(3, 1), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(2, 2), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(4, 2), 1.0, 0.01);
  EXPECT_NEAR(A.coeff(4, 3), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(50, 49), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(3, 52), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(51, 76), 0.2, 0.01);
  EXPECT_NEAR(A.coeff(51, 432), 0.0, 0.01);
  EXPECT_EQ(b_eq.rows(), 52);
  EXPECT_EQ(b_eq.cols(), 1);
  EXPECT_NEAR(b_eq(0, 0), 0.0, 0.01);
  EXPECT_NEAR(b_eq(1, 0), -10.0, 0.01);
  EXPECT_NEAR(b_eq(2, 0), 0.0, 0.01);
  EXPECT_NEAR(b_eq(51, 0), 0.0, 0.01);
}

TEST_F(MPCProblemTest, TestAugmentInequalityConstraints) {
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  double dt = 0.2;
  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_ineq;

  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  mpc_problem_->AugmentInequalityConstraints(dt, dt_prev, constraints, A,
                                             b_ineq);
  EXPECT_EQ(A.rows(), 434);
  EXPECT_EQ(A.cols(), 433);
  EXPECT_NEAR(A.coeff(0, 0), 0.01, 0.01);
  EXPECT_NEAR(A.coeff(51, 128), 0.01, 0.01);
  EXPECT_NEAR(A.coeff(53, 130), 0.01, 0.01);
  EXPECT_NEAR(A.coeff(75, 152), 0.01, 0.01);
  EXPECT_NEAR(A.coeff(331, 408), 0.01, 0.01);
  EXPECT_NEAR(A.coeff(332, 409), 0.01, 0.01);
  EXPECT_NEAR(A.coeff(355, 432), 0.01, 0.01);
}

TEST_F(MPCProblemTest, TestAugmentAIneqMatrix) {
  Eigen::VectorXd u_prev(1);
  u_prev << 0.0;
  double dt_prev = 0.1;
  double dt = 0.2;
  int n_steps = 25;
  const int n_nodes = n_steps + 1;
  const int n_x = 2;

  Eigen::VectorXd x_init(2);
  x_init << 0.0, 10.0;

  zark::planning::MPCData::Constraints constraints;
  ConstructConstrains(constraints);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b_ineq;
  std::vector<Eigen::MatrixXd> A_d;
  std::vector<Eigen::MatrixXd> B_d;
  std::vector<Eigen::MatrixXd> B_d2;
  Eigen::MatrixXd u_2;
  std::tie(A_d, B_d, B_d2) = mpc_problem_->ComputeStateSpaceModel();

  int i_start = 0;
  Eigen::VectorXd b_eq;
  mpc_problem_->ConstructEqualityConstraints(x_init, A_d, B_d, B_d2, u_2, A,
                                             b_eq);
  mpc_problem_->ConstructInequalityConstraints(constraints, dt, u_prev, dt_prev,
                                               A, b_ineq);
  A.conservativeResize(A.rows(), A.cols() + constraints.soft_min.n_ineq);
  mpc_problem_->AugmentAIneqMatrix(dt, dt_prev, constraints.soft_min, i_start,
                                   A);

  EXPECT_EQ(A.rows(), 382 + n_nodes * n_x);
  EXPECT_EQ(A.cols(), 153);
  EXPECT_NEAR(A.coeff(0, 0), -1.0, 0.01);
  EXPECT_NEAR(A.coeff(51, 128), 0.0, 0.01);
  EXPECT_NEAR(A.coeff(53, 130), 0.0, 0.01);
  EXPECT_NEAR(A.coeff(75, 152), 0.0, 0.01);
  EXPECT_NEAR(A.coeff(433, 152), 0.0, 0.01);
}

}  // namespace planning
}  // namespace zark
