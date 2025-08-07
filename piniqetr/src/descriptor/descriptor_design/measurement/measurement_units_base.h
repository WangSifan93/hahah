
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

namespace e2e_noa {
namespace planning {

// Forward declaration for SPT types
namespace spt_opt {
struct state {
  double x, y, theta, v, phi, kappa, a;
};
struct control {
  double j, chi;
};
}  // namespace spt_opt

/**
 * Enum for different types of measurement units
 */
enum class MeasurementUnitType {
  DISTANCE_TO_POINT,
  DISTANCE_TO_LINE,
  SPEED_MEASUREMENT,
  POLYGON_PENETRATION,
  TIME_TO_COLLISION,
  LATERAL_OFFSET,
  LONGITUDINAL_OFFSET
};

}  // namespace planning
}  // namespace e2e_noa