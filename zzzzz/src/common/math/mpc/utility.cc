/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file utility.cc
 **/

#include "apps/planning/src/common/math/mpc/utility.h"

namespace zark {
namespace planning {
namespace util {
Eigen::RowVectorXd TVector(const double& dt, const int& n) {
  Eigen::RowVectorXd t(n);
  for (int k = 0; k < n; ++k) {
    t(k) = k * dt;
  }
  return t;
}

std::string ConvertToTimeString(const double& t) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(1) << "t=" << t << "s";
  return ss.str();
}

}  // namespace util
}  // namespace planning
}  // namespace zark
