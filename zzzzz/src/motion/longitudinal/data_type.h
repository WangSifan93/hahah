/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file data_type.h
 **/

#pragma once

#include <limits>
#include <vector>

#include "apps/planning/src/common/obstacle.h"

namespace zark {
namespace planning {

const int kIdxS = 0;
const int kIdxV = 1;
const int kIdxA = 0;
const int kIdxJ = 0;
const double kInf = std::numeric_limits<double>::infinity();

struct Blocker {
  Blocker(const int n_nodes) {
    s = std::vector<double>(n_nodes, std::numeric_limits<double>::quiet_NaN());
    v = std::vector<double>(n_nodes, std::numeric_limits<double>::quiet_NaN());
  }

  std::vector<double> s;
  std::vector<double> v;
  bool is_front = true;
  bool is_filtered = false;
  int k_start = 40;
  int k_end = 0;
  const Obstacle* obs;
};

enum TimeGapLevel : int {
  LEVEL_1 = 1,
  LEVEL_2 = 2,
  LEVEL_3 = 3,
  LEVEL_4 = 4,
  LEVEL_5 = 5
};

}  // namespace planning
}  // namespace zark
