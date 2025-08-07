#include "prediction/perception_util.h"

#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "math/geometry/util.h"
#include "math/vec.h"

namespace e2e_noa {
namespace prediction {
namespace {

std::vector<Vec2d> TransformPoints(
    const ::google::protobuf::RepeatedPtrField<Vec2dProto>& proto_points,
    const Vec2d& origin_center, const Vec2d& shift, double cos_angle,
    double sin_angle) {
  std::vector<Vec2d> points;
  points.reserve(proto_points.size());
  const Vec2d new_center = origin_center + shift;
  for (const auto& p : proto_points) {
    const Vec2d point = Vec2dFromProto(p) - origin_center;
    points.emplace_back(point.Rotate(cos_angle, sin_angle) + new_center);
  }
  return points;
}
}  // namespace

absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto* object) {
  double time_diff = current_time - object->timestamp();
  constexpr double kMaxTimeDiff = 5.0;
  if (time_diff < 0.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Provided time[%f] is less than perception's time[%f]",
                        current_time, object->timestamp()));
  }

  if (time_diff > kMaxTimeDiff) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Current time[%f] is too different with perception "
                        "time[%f], larger than threshold[%f]",
                        current_time, object->timestamp(), kMaxTimeDiff));
  }

  object->set_timestamp(current_time);
  const Vec2d vel = Vec2dFromProto(object->vel());
  const Vec2d pos = Vec2dFromProto(object->pos());
  const Vec2d accel = Vec2dFromProto(object->accel());

  Vec2d new_vel = vel + accel * time_diff;
  if (new_vel.Dot(vel) < 0) {
    new_vel = Vec2d(0.0, 0.0);
    time_diff = std::fabs(vel.Length() / accel.Length());
  }
  const Vec2d pos_shift = (vel + 0.5 * accel * time_diff) * time_diff;
  const double yaw_diff = NormalizeAngle(object->yaw_rate() * time_diff);
  const double new_yaw = NormalizeAngle(yaw_diff + object->yaw());
  const Vec2d new_pos = pos_shift + pos;

  new_pos.ToProto(object->mutable_pos());

  new_vel.ToProto(object->mutable_vel());

  const Vec2d rotation = Vec2d::FastUnitFromAngle(yaw_diff);
  const auto contour_points = TransformPoints(object->contour(), pos, pos_shift,
                                              rotation.x(), rotation.y());
  object->clear_contour();
  for (const auto& pt : contour_points) {
    pt.ToProto(object->add_contour());
  }

  object->mutable_bounding_box()->set_x(new_pos.x());
  object->mutable_bounding_box()->set_y(new_pos.y());
  object->mutable_bounding_box()->set_heading(new_yaw);

  object->set_yaw(NormalizeAngle(object->yaw() + yaw_diff));

  return absl::OkStatus();
}

}  // namespace prediction
}  // namespace e2e_noa
