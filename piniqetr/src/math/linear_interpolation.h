#ifndef AD_E2E_PLANNING_MATH_LINEAR_INTERPOLATION_H
#define AD_E2E_PLANNING_MATH_LINEAR_INTERPOLATION_H
#include <cmath>

#include "common/type_def.h"
#include "math/math_utils.h"

namespace ad_e2e {
namespace planning {
namespace math {

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t, bool clamp = false) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return x0;
  }
  if (clamp) {
    if (t0 <= t1) {
      if (t <= t0) {
        return x0;
      }
      if (t >= t1) {
        return x1;
      }
    } else {
      if (t >= t0) {
        return x0;
      }
      if (t <= t1) {
        return x1;
      }
    }
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

template <typename T>
T lerp(const T &x0, const T &x1, const double w) {
  const T x = x0 + w * (x1 - x0);
  return x;
}

double lerp_angle(const double a0, const double a1, const double w);

double lerp_angle(const double a0, const double t0, const double a1,
                  const double t1, const double t);

Vec2d InterpolateUsingLinearApproximation(const Vec2d &p0, const Vec2d &p1,
                                          const double w);

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w);

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double w);

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double w);

SpeedPoint InterpolateUsingLinearApproximation(const SpeedPoint &p0,
                                               const SpeedPoint &p1,
                                               const double w);
FrenetPoint InterpolateUsingLinearApproximation(const FrenetPoint &p0,
                                                const FrenetPoint &p1,
                                                const double w);

template <typename T>
T InterpolateUsingLinearApproximation(const T &p0, const double t0, const T &p1,
                                      const double t1, const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return p0;
  }
  const double r = (t - t0) / (t1 - t0);
  return InterpolateUsingLinearApproximation(p0, p1, r);
}
}  // namespace math
}  // namespace planning
}  // namespace ad_e2e
#endif
