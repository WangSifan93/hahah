#ifndef ONBOARD_PLANNER_MATH_CUBIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_
#define ONBOARD_PLANNER_MATH_CUBIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_
#include <utility>
#include <vector>

#include "math/eigen.h"
#include "math/spiral.h"
#include "math/vec.h"

namespace e2e_noa::planning {
struct CubicSpiralBoundaryValueProblemParams {
  double wx;
  double wy;
  double wtheta;
  int num_steps;
};
class CubicSpiralBoundaryValueProblem {
 public:
  CubicSpiralBoundaryValueProblem() = delete;
  CubicSpiralBoundaryValueProblem(double k0, double x1, double y1,
                                  double theta1, double k1)
      : k0_(k0), x1_(x1), y1_(y1), theta1_(theta1), k1_(k1) {}

  double operator()(const VecXd& vecx, VecXd& grad) const {
    return LeastSquare(vecx, &grad);
  }
  double LeastSquare(const VecXd& vecx, VecXd* ptr_grad) const;
  std::vector<double> getPrams(double p0, double p1, double p2, double p3,
                               double sg) const;

 private:
  std::pair<double, double> XY(double p0, double p1, double p2, double p3,
                               double sg, Vec3d* ptr_xgrad,
                               Vec3d* ptr_ygrad) const;
  double Theta(double p0, double p1, double p2, double p3, double sg,
               double ratio) const;
  Vec3d ThetaGrad(double p0, double p1, double p2, double p3, double sg,
                  double ratio) const;
  double CosTheta(double theta) const;
  Vec3d CosThetaGrad(double theta, const Vec3d& theta_grad) const;
  double SinTheta(double theta) const;
  Vec3d SinThetaGrad(double theta, const Vec3d& theta_grad) const;

  double k0_;
  double x1_;
  double y1_;
  double theta1_;
  double k1_;
  const CubicSpiralBoundaryValueProblemParams kParams_ = {1.0, 1.0, 1.0, 8};
};

class CubicSpiralBoundaryValueSolver {
 public:
  CubicSpiralBoundaryValueSolver() = default;
  static std::vector<Spiral> solve(const SpiralPoint& start,
                                   const SpiralPoint& end);
};
}  // namespace e2e_noa::planning

#endif
