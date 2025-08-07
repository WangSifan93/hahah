#ifndef QUINTIC_POLYNOMIAL_CURVE1D_H_
#define QUINTIC_POLYNOMIAL_CURVE1D_H_

#include <array>

#include "math/polynomial_curve1d.h"
namespace e2e_noa {
namespace planning {

class QuinticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuinticPolynomialCurve1d() = default;

  QuinticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 3>& end,
                           const double param);

  QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);

  ~QuinticPolynomialCurve1d() override = default;

  void SetParam(const double x0, const double dx0, const double ddx0,
                const double x1, const double dx1, const double ddx1,
                const double param);

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }
  std::string ToString() const override;

  double Coef(const size_t order) const override;

  size_t Order() const override { return 5; }

  bool GetParams(double& s1, double& s2, double& s3, double& p) const;

 protected:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
  std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
};

}  // namespace planning
}  // namespace e2e_noa
#endif
