#pragma once
#include <cmath>
#include <string>

#include "error_code.h"
#include "geometry.h"
#include "header.h"
#include "pnc_point.h"

namespace zark {
namespace planning {
namespace perception {

enum Type {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,  // Pedestrian, usually determined by moving behavior.
  BICYCLE = 4,     // bike, motor bike
  VEHICLE = 5      // Passenger car or truck.
};

enum ConfidenceType {
  CONFIDENCE_UNKNOWN = 0,
  CONFIDENCE_CNN = 1,
  CONFIDENCE_RADAR = 2
};

enum SubType {
  ST_UNKNOWN = 0,
  ST_UNKNOWN_MOVABLE = 1,
  ST_UNKNOWN_UNMOVABLE = 2,
  ST_CAR = 3,
  ST_VAN = 4,
  ST_LIGHT_TRUCK = 5,
  ST_SMALL_BUS = 6,
  ST_CYCLIST = 7,
  ST_MOTORCYCLIST = 8,
  ST_TRICYCLIST = 9,
  ST_PEDESTRIAN = 10,
  ST_TRAFFICCONE = 11,
  ST_BIG_BUS = 12,
  ST_HEAVY_TRUCK = 13,
  ST_CONE = 14,
  ST_POLE = 15,
  ST_OTHER_VEHICLE = 16
};

class BBox2D {
 public:
  // Default constructor
  BBox2D() = default;

  // Parameterized constructor
  BBox2D(const double& xmin, const double& ymin, const double& xmax,
         const double& ymax)
      : xmin_(xmin), ymin_(ymin), xmax_(xmax), ymax_(ymax) {}

  // Getter functions
  const double& xmin() const { return xmin_; }
  const double& ymin() const { return ymin_; }
  const double& xmax() const { return xmax_; }
  const double& ymax() const { return ymax_; }

  // Setter functions
  void set_xmin(const double& xmin) { xmin_ = xmin; }
  void set_ymin(const double& ymin) { ymin_ = ymin; }
  void set_xmax(const double& xmax) { xmax_ = xmax; }
  void set_ymax(const double& ymax) { ymax_ = ymax; }

 private:
  double xmin_;  // in pixels.
  double ymin_;  // in pixels.
  double xmax_;  // in pixels.
  double ymax_;  // in pixels.
};

class PerceptionObstacle {
 public:
  enum Source { HOST_VEHICLE = 0, V2X = 1 };

  //  Default constructor
  PerceptionObstacle() {}

  // Parameterized constructor
  PerceptionObstacle(
      const int& id, const ::common::Point3D& position, const double& theta,
      const double& length, const double& width, const double& height,
      const ::common::Point3D& velocity,
      const std::vector<::common::Point3D>& polygon_point,
      const double& tracking_time, const Type& type, const double& timestamp,
      const std::vector<double>& point_cloud, const double& confidence,
      const ConfidenceType& confidence_type,
      const std::vector<::common::Point3D>& drops,
      const ::common::Point3D& acceleration,
      const ::common::Point3D& anchor_point, const BBox2D& bbox2d,
      const SubType& sub_type, const double& height_above_ground,
      const std::vector<double>& position_covariance,
      const std::vector<double>& velocity_covariance,
      const std::vector<double>& acceleration_covariance, const Source& source)
      : id_(id),
        position_(position),
        theta_(theta),
        velocity_(velocity),
        length_(length),
        width_(width),
        height_(height),
        polygon_point_(polygon_point),
        tracking_time_(tracking_time),
        type_(type),
        timestamp_(timestamp),
        point_cloud_(point_cloud),
        confidence_(confidence),
        confidence_type_(confidence_type),
        drops_(drops),
        acceleration_(acceleration),
        anchor_point_(anchor_point),
        bbox2d_(bbox2d),
        sub_type_(sub_type),
        height_above_ground_(height_above_ground),
        position_covariance_(position_covariance),
        velocity_covariance_(velocity_covariance),
        acceleration_covariance_(acceleration_covariance),
        source_(source) {}

  // Getter functions
  const int& id() const { return id_; }
  const ::common::Point3D& position() const { return position_; }
  const double& theta() const { return theta_; }
  const double& length() const { return length_; }
  const double& width() const { return width_; }
  const double& height() const { return height_; }
  const ::common::Point3D& velocity() const { return velocity_; }
  const std::vector<::common::Point3D>& polygon_point() const {
    return polygon_point_;
  }
  const double& tracking_time() const { return tracking_time_; }
  const Type& type() const { return type_; }
  const double& timestamp() const { return timestamp_; }
  const std::vector<double>& point_cloud() const { return point_cloud_; }
  const double& confidence() const { return confidence_; }
  const ConfidenceType& confidence_type() const { return confidence_type_; }
  const std::vector<::common::Point3D>& drops() const { return drops_; }
  const ::common::Point3D& acceleration() const { return acceleration_; }
  const ::common::Point3D& anchor_point() const { return anchor_point_; }
  const BBox2D& bbox2d() const { return bbox2d_; }
  const SubType& sub_type() const { return sub_type_; }
  const double& height_above_ground() const { return height_above_ground_; }
  const std::vector<double>& position_covariance() const {
    return position_covariance_;
  }
  const std::vector<double>& velocity_covariance() const {
    return velocity_covariance_;
  }
  const std::vector<double>& acceleration_covariance() const {
    return acceleration_covariance_;
  }
  const Source& source() const { return source_; }

  // Setter functions
  void set_id(const int& id) { id_ = id; }
  void set_position(const ::common::Point3D& position) { position_ = position; }
  void set_theta(const double& theta) { theta_ = theta; }
  void set_length(const double& length) { length_ = length; }
  void set_width(const double& width) { width_ = width; }
  void set_height(const double& height) { height_ = height; }
  void set_velocity(const ::common::Point3D& velocity) { velocity_ = velocity; }
  void set_polygon_point(const std::vector<::common::Point3D>& polygon_point) {
    polygon_point_ = polygon_point;
  }
  void set_tracking_time(const double& tracking_time) {
    tracking_time_ = tracking_time;
  }
  void set_type(const Type& type) { type_ = type; }
  void set_timestamp(const double& timestamp) { timestamp_ = timestamp; }
  void set_point_cloud(const std::vector<double>& point_cloud) {
    point_cloud_ = point_cloud;
  }
  void set_confidence(const double& confidence) { confidence_ = confidence; }
  void set_confidence_type(const ConfidenceType& confidence_type) {
    confidence_type_ = confidence_type;
  }
  void set_drops(const std::vector<::common::Point3D>& drops) {
    drops_ = drops;
  }
  void set_acceleration(const ::common::Point3D& acceleration) {
    acceleration_ = acceleration;
  }
  void set_anchor_point(const ::common::Point3D& anchor_point) {
    anchor_point_ = anchor_point;
  }
  void set_bbox2d(const BBox2D& bbox2d) { bbox2d_ = bbox2d; }
  void set_sub_type(const SubType& sub_type) { sub_type_ = sub_type; }
  void set_height_above_ground(const double& height_above_ground) {
    height_above_ground_ = height_above_ground;
  }
  void set_position_covariance(const std::vector<double>& position_covariance) {
    position_covariance_ = position_covariance;
  }
  void set_velocity_covariance(const std::vector<double>& velocity_covariance) {
    velocity_covariance_ = velocity_covariance;
  }
  void set_acceleration_covariance(
      const std::vector<double>& acceleration_covariance) {
    acceleration_covariance_ = acceleration_covariance;
  }
  void set_source(const Source& source) { source_ = source; }
  std::string DebugString() const { return "DebugString!"; }
  ::common::Point3D* mutable_position() { return &position_; }
  ::common::Point3D* mutable_velocity() { return &velocity_; }
  ::common::Point3D* add_polygon_point() {
    ::common::Point3D point;
    polygon_point_.emplace_back(point);
    return &polygon_point_.back();
  }

  const bool has_velocity() const {
    return std::fabs(velocity_.x()) > 0.0 && std::fabs(velocity_.y()) > 0.0;
  }

  std::string PerceptionObstacle_Type_Name(const Type& type) const {
    if (type == UNKNOWN)
      return "UNKNOWN";
    else if (type == UNKNOWN_MOVABLE)
      return "UNKNOWN_MOVABLE";
    else if (type == UNKNOWN_UNMOVABLE)
      return "UNKNOWN_UNMOVABLE";
    else if (type == PEDESTRIAN)
      return "PEDESTRIAN";
    else if (type == BICYCLE)
      return "BICYCLE";
    else if (type == VEHICLE)
      return "VEHICLE";
    else
      return " ";
  }

  // The last const is necessary because the object is usually const object
  const char* get_type_enum(const Type& value) const {
    switch (value) {
      case UNKNOWN:
        return "UNKNOWN";
      case UNKNOWN_MOVABLE:
        return "UNKNOWN_MOVABLE";
      case UNKNOWN_UNMOVABLE:
        return "UNKNOWN_UNMOVABLE";
      case PEDESTRIAN:
        return "PEDESTRIAN";
      case BICYCLE:
        return "BICYCLE";
      case VEHICLE:
        return "VEHICLE";
      default:
        return "";
    }
  }

 private:
  int id_;  // obstacle ID.

  // obstacle position in the world coordinate system.
  ::common::Point3D position_;

  double theta_;                // heading in the world coordinate system.
  ::common::Point3D velocity_;  // obstacle velocity.

  // Size of obstacle bounding box.
  double length_;  // obstacle length.
  double width_;   // obstacle width.
  double height_;  // obstacle height.

  std::vector<::common::Point3D> polygon_point_;  // obstacle corner points.

  // duration of an obstacle since detection in s.
  double tracking_time_;

  Type type_;         // obstacle type
  double timestamp_;  // GPS time in seconds.

  // Just for offline debugging, will not fill this field on board.
  // Format: [x0, y0, z0, x1, y1, z1...]
  std::vector<double> point_cloud_;  // [packed = true];

  double confidence_;  // [deprecated = true];

  ConfidenceType confidence_type_;  // [deprecated = true];
  // trajectory of object.
  std::vector<::common::Point3D> drops_;  // [deprecated = true];

  // The following fields are new added in zpilot 4.0
  ::common::Point3D acceleration_;  // obstacle acceleration

  // a stable obstacle point in the world coordinate system
  // position defined above is the obstacle bounding box ground center
  ::common::Point3D anchor_point_;
  BBox2D bbox2d_;

  SubType sub_type_;  // obstacle sub_type

  // orthogonal distance between obstacle lowest point and ground plane
  double height_above_ground_;  // [default = nan];

  // position covariance which is a row-majored 3x3 matrix
  std::vector<double> position_covariance_;  // [packed = true];
  // velocity covariance which is a row-majored 3x3 matrix
  std::vector<double> velocity_covariance_;  // [packed = true];
  // acceleration covariance which is a row-majored 3x3 matrix
  std::vector<double> acceleration_covariance_;  // [packed = true];

  Source source_;  // [default = HOST_VEHICLE];
};

class CIPVInfo {
 public:
  // Default constructor
  CIPVInfo() = default;

  // Parameterized constructor
  CIPVInfo(const int& cipv_id, const std::vector<int>& potential_cipv_id)
      : cipv_id_(cipv_id), potential_cipv_id_(potential_cipv_id) {}

  // Getter functions
  const int& cipv_id() const { return cipv_id_; }
  const std::vector<int>& potential_cipv_id() const {
    return potential_cipv_id_;
  }

  // Setter functions
  void set_cipv_id(const int& cipv_id) { cipv_id_ = cipv_id; }
  void set_potential_cipv_id(const std::vector<int>& potential_cipv_id) {
    potential_cipv_id_ = potential_cipv_id;
  }

 private:
  int cipv_id_;
  std::vector<int> potential_cipv_id_;
};

class PerceptionObstacles {
 public:
  // Default constructor
  PerceptionObstacles() = default;

  // Parameterized constructor
  PerceptionObstacles(
      const std::vector<PerceptionObstacle>& perception_obstacle,
      const ::common::Header& header, const ::common::ErrorCode& error_code,
      const CIPVInfo cipv_info)
      : perception_obstacle_(perception_obstacle),
        header_(header),
        error_code_(error_code),
        cipv_info_(cipv_info) {}

  // Getter functions
  const std::vector<PerceptionObstacle>& perception_obstacle() const {
    return perception_obstacle_;
  }
  const ::common::Header& header() const { return header_; }
  const ::common::ErrorCode& error_code() const { return error_code_; }
  const CIPVInfo cipv_info() const { return cipv_info_; }

  // Setter functions
  void set_perception_obstacle(
      const std::vector<PerceptionObstacle>& perception_obstacle) {
    perception_obstacle_ = perception_obstacle;
  }
  void set_header(const ::common::Header& header) { header_ = header; }
  void set_error_code(const ::common::ErrorCode& error_code) {
    error_code_ = error_code;
  }
  void set_cipv_info(const CIPVInfo cipv_info) { cipv_info_ = cipv_info; }

 private:
  std::vector<PerceptionObstacle>
      perception_obstacle_;         // An array of obstacles
  ::common::Header header_;         // Header
  ::common::ErrorCode error_code_;  // [default = OK];
  CIPVInfo cipv_info_;              // Closest In Path Vehicle (CIPV)
};

}  // namespace perception
}  // namespace planning
}  // namespace zark
