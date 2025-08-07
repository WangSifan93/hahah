#include "speed/decision/speed_limit_decision.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"
#include "math/frenet_common.h"
#include "math/geometry/box2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "speed_planning.pb.h"
#include "util/map_util.h"
#include "util/path_util.h"

namespace e2e_noa {
namespace planning {

namespace {}

void LaneChangeSpeedDecider(const double& av_speed, const double& max_acc,
                            OpenLoopSpeedLimit* open_loop_speed_limit) {
  if (av_speed < Kph2Mps(30.0)) {
    if (av_speed < Kph2Mps(20.0)) {
      double a_limit = Lerp(0.5, Kph2Mps(10.0), std::fmin(max_acc, 1.0),
                            Kph2Mps(20.0), av_speed, true);
      if (av_speed < 1e-2) {
        a_limit = std::fmax(a_limit, 1.0);
      }
      open_loop_speed_limit->AddALimit(a_limit, std::nullopt,
                                       "20.0 low speed lane change");
    } else {
      double a_limit = Lerp(std::fmin(max_acc, 1.0), Kph2Mps(20.0), max_acc,
                            Kph2Mps(30.0), av_speed, true);
      open_loop_speed_limit->AddALimit(a_limit, std::nullopt,
                                       "30.0 low speed lane change");
    }
  }
  return;
}

}  // namespace planning
}  // namespace e2e_noa
