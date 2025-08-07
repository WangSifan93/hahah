/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

#ifndef _APPS_PLANNING_SRC_EVALUATION_CONST_VALUES_H_
#define _APPS_PLANNING_SRC_EVALUATION_CONST_VALUES_H_

namespace zark {
namespace planning {

// the cost which value is smaller then this will be ignored
static constexpr double kCostEpsilon = 1e-3;
static constexpr double kSlackEpsilon = 1e-3;

}  // namespace planning
}  // namespace zark

#endif /* _APPS_PLANNING_SRC_EVALUATION_CONST_VALUES_H_ */
