#ifndef AD_E2E_PLANNING_MATH_VEC3D_H
#define AD_E2E_PLANNING_MATH_VEC3D_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>

#include "math/vec.h"

namespace ad_e2e {
namespace planning {
namespace math {

using Vec3d = e2e_noa::Vec3<double>;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Quaterniond Quaterniond;
typedef Eigen::AngleAxisd AngleAxisd;
typedef Eigen::Isometry3d Isometry3d;

}  // namespace math
}  // namespace planning
}  // namespace ad_e2e

#endif
