#ifndef ONBOARD_MATH_EIGEN_H_
#define ONBOARD_MATH_EIGEN_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/LU"
#include "Eigen/SVD"
#include "Eigen/Sparse"

namespace e2e_noa {

using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4d = Eigen::Matrix<double, 4, 4>;

using Mat2f = Eigen::Matrix<float, 2, 2>;
using Mat3f = Eigen::Matrix<float, 3, 3>;
using Mat4f = Eigen::Matrix<float, 4, 4>;

using Vec4d = Eigen::Matrix<double, 4, 1>;
using Vec4i = Eigen::Matrix<int, 4, 1>;

using SMatXd = Eigen::SparseMatrix<double>;

using Quaternion = Eigen::Quaternion<double>;
using AngleAxis = Eigen::AngleAxis<double>;

static Eigen::IOFormat CommaInit(Eigen::FullPrecision, Eigen::DontAlignCols,
                                 ", ", ", ", "", "", " << ", ";");
static Eigen::IOFormat Comma(Eigen::FullPrecision, 0, ", ", "\n", "", "");
static Eigen::IOFormat Numpy(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]",
                             "[", "]");
static Eigen::IOFormat NumpyShort(5, 0, ", ", ",\n", "[", "]", "[", "]");

}  // namespace e2e_noa

#endif
