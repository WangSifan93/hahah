#include "speed/empty_road_speed.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "plan/planner_defs.h"
#include "plan/speed_profile.h"
#include "planner_params.pb.h"

namespace e2e_noa {
namespace planning {

SpeedProfile CreateEmptyRoadSpeedProfile(
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const DiscretizedPath& path, const SpeedLimit& speed_limit, double v_now,
    int traj_steps) {
  const double max_accel = spacetime_constraint_params.max_acceleration();
  constexpr double kComfortDecel = -1.0;
  std::vector<double> v_s(path.size());

  v_s[0] = v_now;
  for (int i = 1; i < path.size(); ++i) {
    const double ds = path[i].s() - path[i - 1].s();
    v_s[i] = std::sqrt(Sqr(v_s[i - 1]) + 2.0 * max_accel * ds);
  }

  const auto compute_v_constraint_sqr =
      [max_accel](double s_start, double s_end, double v_max, double s) {
        if (s < s_start) {
          return Sqr(v_max) - 2.0 * kComfortDecel * (s_start - s);
        }
        if (s > s_end) {
          return Sqr(v_max) + 2.0 * max_accel * (s - s_end);
        }
        return Sqr(v_max);
      };

  for (const auto& range : speed_limit.speed_limit_ranges()) {
    for (int i = 0; i < path.size(); ++i) {
      const double v_constraint_sqr = compute_v_constraint_sqr(
          range.start_s, range.end_s, range.speed_limit, path[i].s());
      if (v_constraint_sqr < Sqr(v_s[i])) {
        v_s[i] = std::sqrt(v_constraint_sqr);
      }
    }
  }

  std::vector<double> path_s;
  path_s.reserve(path.size());
  for (int i = 0; i < path.size(); ++i) {
    path_s.push_back(path[i].s());
  }

  const PiecewiseSqrtFunction<double, double> v_s_plf(path_s, v_s);
  std::vector<double> t(traj_steps);
  for (int i = 0; i < t.size(); ++i) {
    t[i] = i * kTrajectoryTimeStep;
  }

  constexpr double kMinSpeed = 1e-6;
  std::vector<double> s(t.size());
  double current_s = 0.0;
  for (int i = 0; i < s.size(); ++i) {
    s[i] = current_s;
    const double current_v = v_s_plf(current_s);
    current_s += std::max(current_v, kMinSpeed) * kTrajectoryTimeStep;
    current_s = std::min(path_s.back(), current_s);
  }

  return SpeedProfile(PiecewiseLinearFunction(std::move(t), std::move(s)));
}

}  // namespace planning
}  // namespace e2e_noa
