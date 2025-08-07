#pragma once
#ifndef AD_E2E_PLANNING_MATH_CARTESIAN_FRENET_CONVERSION_H_
#define AD_E2E_PLANNING_MATH_CARTESIAN_FRENET_CONVERSION_H_
#include <array>

#include "math/vec2d.h"

namespace ad_e2e {
namespace planning {
namespace math {

class CartesianFrenetConverter {
 public:
  CartesianFrenetConverter() = delete;

  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double v, const double a,
                                  const double theta, const double kappa,
                                  std::array<double, 3>* const ptr_s_condition,
                                  std::array<double, 3>* const ptr_d_condition);

  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double x, const double y, double* ptr_s,
                                  double* ptr_d);

  static void frenet_to_cartesian(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const std::array<double, 3>& s_condition,
                                  const std::array<double, 3>& d_condition,
                                  double* const ptr_x, double* const ptr_y,
                                  double* const ptr_theta,
                                  double* const ptr_kappa, double* const ptr_v,
                                  double* const ptr_a);

  static double CalculateTheta(const double rtheta, const double rkappa,
                               const double l, const double dl);

  static double CalculateKappa(const double rkappa, const double rdkappa,
                               const double l, const double dl,
                               const double ddl);

  static Vec2d CalculateCartesianPoint(const double rtheta, const Vec2d& rpoint,
                                       const double l);

  static double CalculateLateralDerivative(const double theta_ref,
                                           const double theta, const double l,
                                           const double kappa_ref);

  static double CalculateSecondOrderLateralDerivative(
      const double theta_ref, const double theta, const double kappa_ref,
      const double kappa, const double dkappa_ref, const double l);
};

}  // namespace math
}  // namespace planning
}  // namespace ad_e2e
#endif
