/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file utility.h
 **/

#pragma once

#include <Eigen/Core>
#include <iomanip>
#include <sstream>

namespace zark {
namespace planning {
namespace util {
Eigen::RowVectorXd TVector(const double& dt, const int& n);

std::string ConvertToTimeString(const double& t);

};  // namespace util

}  // namespace planning
}  // namespace zark
