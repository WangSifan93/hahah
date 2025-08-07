#ifndef AD_E2E_PLANNING_MATH_VEC2D_H
#define AD_E2E_PLANNING_MATH_VEC2D_H

#include "math/vec.h"
namespace ad_e2e {
namespace planning {
namespace math {
using Vec2d = e2e_noa::Vec2<double>;
constexpr double kMathEpsilon = e2e_noa::kMathEpsilon;
}  // namespace math
}  // namespace planning
}  // namespace ad_e2e
#endif
