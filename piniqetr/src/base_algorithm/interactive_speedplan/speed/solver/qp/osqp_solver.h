#ifndef ST_PLANNING_MATH_QP_OSQP_SOLVER
#define ST_PLANNING_MATH_QP_OSQP_SOLVER

#include <osqp.h>

#include "absl/status/status.h"
#include "math/eigen.h"
#include "speed/solver/qp/sparse_qp_solver.h"

namespace e2e_noa {

class OsqpSolver : public SparseQpSolver {
 public:
  OsqpSolver(const SMatXd& A, const VecXd& b, const SMatXd& G, const VecXd& g,
             const SMatXd& H, const VecXd& h);
  virtual ~OsqpSolver() = default;

  absl::Status Solve() override;
  absl::Status Solve(const OSQPSettings& settings);
};

}  // namespace e2e_noa

#endif
