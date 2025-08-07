#include "speed/path_speed_combiner.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <ostream>
#include <string>

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "math/util.h"
#include "plan/planner_defs.h"
#include "speed/speed_point.h"
#include "util/status_macros.h"

namespace e2e_noa::planning {
namespace {
constexpr double kEpsilon = 1e-6;
}

absl::Status CombinePathAndSpeed(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory) {
  CHECK_NOTNULL(trajectory);

  for (int i = 0; i < speed_data.size(); ++i) {
  }
  CHECK_GT(path_data.size(), 1);
  CHECK_GT(speed_data.size(), 1);

  trajectory->clear();
  double t = 0.0;
  while (t < speed_data.TotalTime()) {
    const auto speed_point = speed_data.EvaluateByTime(t);
    if (!speed_point.has_value()) {
      const auto error_msg =
          absl::StrFormat("Fail to evaluate speed vector at time %.2f", t);
      LOG(WARNING) << error_msg;
      return absl::InternalError(error_msg);
    }

    PathPoint path_point;
    if (path_data.length() < kEpsilon) {
      path_point = path_data.front();
    } else {
      path_point = path_data.Evaluate(speed_point->s());
    }

    ApolloTrajectoryPointProto& traj_point = trajectory->emplace_back();
    if (!forward) {
      path_point.set_s(-path_point.s());
      path_point.set_theta(NormalizeAngle(path_point.theta() + M_PI));
      path_point.set_kappa(-path_point.kappa());
    }
    *(traj_point.mutable_path_point()) = path_point;
    traj_point.set_v(forward ? speed_point->v() : -speed_point->v());
    traj_point.set_a(forward ? speed_point->a() : -speed_point->a());
    traj_point.set_j(forward ? speed_point->j() : -speed_point->j());
    const double yaw_rate = speed_point->v() * path_point.kappa();
    traj_point.set_yaw_rate(forward ? yaw_rate : -yaw_rate);
    traj_point.set_relative_time(t);
    traj_point.set_is_extend(false);

    t += kTrajectoryTimeStep;
  }

  RETURN_IF_ERROR(
      ExtendTrajectoryLength(path_data, forward, speed_data, trajectory));

  return absl::OkStatus();
}

absl::Status ExtendTrajectoryLength(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory) {
  if (trajectory->empty()) {
    const auto error_msg =
        absl::StrFormat("Fail to extend trajectory because of empty");
    LOG(WARNING) << error_msg;
    return absl::InternalError(error_msg);
  }
  if (speed_data.empty()) {
    const auto error_msg =
        absl::StrFormat("Fail to evaluate speed vector because of empty");
    LOG(WARNING) << error_msg;
    return absl::InternalError(error_msg);
  }
  const double kTrajMinLength = 5.0;
  const double kStopVThreshold = 0.1;
  const double kExtendDistStep = 0.1;
  auto speed_point = speed_data.back();
  double t = trajectory->back().relative_time();
  double s = speed_point.s();
  while (trajectory->back().path_point().s() <
             trajectory->front().path_point().s() + kTrajMinLength &&
         trajectory->back().v() < kStopVThreshold) {
    t += kTrajectoryTimeStep;
    s += kExtendDistStep;
    speed_point.set_s(s);
    PathPoint path_point;
    if (path_data.length() < kEpsilon) {
      path_point = path_data.front();
    } else {
      path_point = path_data.Evaluate(speed_point.s());
    }
    ApolloTrajectoryPointProto& extend_point = trajectory->emplace_back();
    if (!forward) {
      path_point.set_s(-path_point.s());
      path_point.set_theta(NormalizeAngle(path_point.theta() + M_PI));
      path_point.set_kappa(-path_point.kappa());
    }
    *(extend_point.mutable_path_point()) = path_point;
    extend_point.set_v(forward ? speed_point.v() : -speed_point.v());
    extend_point.set_a(forward ? speed_point.a() : -speed_point.a());
    extend_point.set_j(forward ? speed_point.j() : -speed_point.j());
    const double yaw_rate = speed_point.v() * path_point.kappa();
    extend_point.set_yaw_rate(forward ? yaw_rate : -yaw_rate);
    extend_point.set_relative_time(t);
    extend_point.set_is_extend(true);
  }
  return absl::OkStatus();
}

}  // namespace e2e_noa::planning
