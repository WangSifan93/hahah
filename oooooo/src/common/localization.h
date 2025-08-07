
#ifndef COMMON_LOCALIZATION
#define COMMON_LOCALIZATION
#include "common/type_def.h"
#include "math/vec3d.h"

namespace ad_e2e::planning {
class Localization {
 public:
  Localization() = delete;
  explicit Localization(const LocalizationInfo &info);
  Localization(const math::Vec2d &position, const double &yaw);
  ~Localization() = default;

  double timestamp() const { return timestamp_; }
  void set_timestamp(const double &timestamp) { timestamp_ = timestamp; }

  const std::string &frame_id() const { return frame_id_; }
  void set_frame_id(const std::string &frame_id) { frame_id_ = frame_id; }

  CoordinateType type() const { return type_; }
  void set_type(const CoordinateType &type) { type_ = type; }

  const math::Vector3d &position() const { return position_; }
  void set_position(const math::Vector3d &position) { position_ = position; }

  const math::Quaterniond &quaternion() const { return q_; };
  void set_quaternion(const math::Quaterniond &q) { q_ = q; }

  const math::Vector3d &v() const { return v_; }
  void set_v(const math::Vector3d &v) { v_ = v; }

  const math::Vector3d &a() const { return a_; }
  void set_a(const math::Vector3d &a) { a_ = a; }

  const math::Vector3d &angular_v() const { return angular_v_; }
  void set_angular_v(const math::Vector3d &angular_v) {
    angular_v_ = angular_v;
  }

  const std::vector<double> &position_cov() const { return position_cov_; }
  void set_position_cov(const std::vector<double> &position_cov) {
    position_cov_ = position_cov;
  }

  const std::vector<double> &quaternion_cov() const { return quaternion_cov_; }
  void set_quaternion_cov(const std::vector<double> &quaternion_cov) {
    quaternion_cov_ = quaternion_cov;
  }

  Point2d point() const { return {position_.x(), position_.y()}; }

  double yaw() const { return ToEulerAngles(q_).z(); }

  double yaw_rate() const { return angular_v_.z(); }

  double linear_v() const { return v_.norm(); }

  double lon_v() const { return (q_.inverse() * v_).x(); }

  double lat_v() const { return (q_.inverse() * v_).y(); }

  double theta() const { return std::atan2(v_.y(), v_.x()); }

  double linear_a() const {
    return std::hypot((q_.inverse() * a_).x(), (q_.inverse() * a_).y());
  }

  double lon_a() const { return (q_.inverse() * a_).x(); }

  double lat_a() const { return (q_.inverse() * a_).y(); }

  math::Vector3d WorldToVehicle(const math::Vector3d &world_p) const {
    return q_.inverse() * (world_p - position_);
  }
  TrajectoryPoint WorldToVehicle(const TrajectoryPoint &world_p) const;
  std::shared_ptr<Localization> WorldToVehicle(
      const std::shared_ptr<Localization> &world_p) const;
  math::Vec2d WorldToVehicle(const math::Vec2d &world_p) const;

  math::Vector3d VehicleToWorld(const math::Vector3d &vehicle_p) const {
    return q_ * vehicle_p + position_;
  }
  TrajectoryPoint VehicleToWorld(const TrajectoryPoint &vehicle_p) const;
  std::shared_ptr<Localization> VehicleToWorld(
      const std::shared_ptr<Localization> &vehicle_p) const;
  math::Vec2d VehicleToWorld(const math::Vec2d &vehicle_p) const;

  static math::Vector3d ToEulerAngles(const math::Quaterniond &q);

 private:
  double timestamp_ = 0.0;
  std::string frame_id_;
  CoordinateType type_ = CoordinateType::COORDINATE_ODOMETRY;
  math::Vector3d position_;
  math::Quaterniond q_;
  math::Vector3d v_;
  math::Vector3d a_;
  math::Vector3d angular_v_;
  std::vector<double> position_cov_;
  std::vector<double> quaternion_cov_;
};
typedef std::shared_ptr<Localization> LocalizationPtr;
}  // namespace ad_e2e::planning

#endif
