/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file local_route_point.h
 **/

#pragma once

#include "pnc_point.h"
#include "apps/planning/src/reference_line_provider/pnc_map/path.h"

namespace zark {
namespace planning {

constexpr double kDuplicatedEpsilon = 1e-7;

class LocalRoutePoint : public hdmap::MapPathPoint {
 public:
  LocalRoutePoint() = default;

  LocalRoutePoint(const hdmap::MapPathPoint& map_path_point, const double kappa,
                  const double dkappa)
      : MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}

  const double kappa() const { return kappa_; }

  const double dkappa() const { return dkappa_; }

  ::common::PathPoint ToPathPoint(double s) const;

  static void RemoveDuplicates(std::vector<LocalRoutePoint>* points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace zark
