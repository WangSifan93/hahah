
#ifndef ONBOARD_PLANNER_MATH_QUINTIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_
#define ONBOARD_PLANNER_MATH_QUINTIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_
#include <vector>

#include "math/eigen.h"
#include "math/spiral.h"

namespace e2e_noa::planning {

struct QuinticSpiralBoundaryValueProblemParams {
  double wx;
  double wy;
  double wtheta;
  int num_steps;
};

class QuinticSpiralBoundaryValueProblem {
 public:
  QuinticSpiralBoundaryValueProblem() = delete;
  QuinticSpiralBoundaryValueProblem(double k0, double x1, double y1,
                                    double theta1, double k1, double dk0,
                                    double ddk0)
      : k0_(k0),
        x1_(x1),
        y1_(y1),
        theta1_(theta1),
        k1_(k1),
        dk0_(dk0),
        ddk0_(ddk0) {}

  double operator()(const VecXd& vecx, VecXd& grad) const {
    return LeastSquare(vecx, &grad);
  }
  double LeastSquare(const VecXd& vecx, VecXd* ptr_grad) const;

  std::vector<double> getParams(double p0, double p1, double p2, double p3,
                                double p4, double p5, double sg) const;

 private:
  double k0_;
  double x1_;
  double y1_;
  double theta1_;
  double k1_;
  double dk0_;
  double ddk0_;

  const QuinticSpiralBoundaryValueProblemParams kParams_ = {1.0, 1.0, 1.0, 8};
};

class QuinticSpiralBoundaryValueSolver {
 public:
  QuinticSpiralBoundaryValueSolver() = default;
  static std::vector<Spiral> solve(const SpiralPoint& start,
                                   const SpiralPoint& end);
};

}  // namespace e2e_noa::planning

#endif
