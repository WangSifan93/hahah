#ifndef AD_E2E_PLANNING_COMMON_OBSTACLE_H
#define AD_E2E_PLANNING_COMMON_OBSTACLE_H

#include "common/gflags.h"
#include "common/log.h"
#include "common/object_situation.h"
#include "common/obstacle_frame.h"
#include "common/trajectory.h"
#include "common/transformer.h"
#include "common/type_def.h"
#include "math/polygon2d.h"
#include "prediction.pb.h"
#include "util/circular_queue.h"

namespace ad_e2e {
namespace planning {
bool IsStatic(uint8_t type);
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const int32_t& history_num);
  ~Obstacle() = default;
  const std::string& id() const { return id_; }

  void InsertFrame(const DynamicObstacleInfo& dyn_obs,
                   const ObstacleFrame* const adc_frame);
  void InsertFrame(const StaticObstacleInfo& stc_obs);
  void SetId(const std::string& id) { id_ = id; }

  ObstacleFrame* GetLatestFrame();

  ObstacleFrame* GetFrameAt(int32_t idx);

  const CircularQueue<ObstacleFrame>& GetObsHistoryFrames() const;

  CircularQueue<ObstacleFrame>* GetObsHistoryFramesPtr();

  void ConvertToFloats(const Vec2d& anchor_point, const double& rotate_heading,
                       const int32_t& obs_idx, const std::size_t& start_idx,
                       std::vector<float>* const fp) const;
  void ConvertDynamicToFloats(const Vec2d& anchor_point,
                              const double& rotate_heading,
                              const int32_t& obs_idx,
                              const std::size_t& start_idx,
                              std::vector<float>* const fp) const;
  void ConvertStaticToFloats(const Vec2d& anchor_point,
                             const double& rotate_heading,
                             const int32_t& obs_idx,
                             const std::size_t& start_idx,
                             std::vector<float>* const fp) const;

 private:
  CircularQueue<ObstacleFrame> obs_history_frames_;

  bool is_static_ = false;
  std::string id_;
  std::size_t max_vector_size_ = FLAGS_ad_e2e_planning_max_obstacle_vector_size;
  std::size_t dynamic_vector_dim_ =
      FLAGS_ad_e2e_planning_dynamic_obstacle_vector_dim;
  std::size_t static_vector_dim_ =
      FLAGS_ad_e2e_planning_static_obstacle_vector_dim;
};

class StationaryObstacle {
 public:
  StationaryObstacle() = delete;
  explicit StationaryObstacle(const std::string& id,
                              const e2e_noa::ObjectType& type,
                              const math::Polygon2d& polygon,
                              const Vec2d& obs_pos, const double& theta,
                              const double& length, const double& width)
      : id_(id),
        type_(type),
        polygon_(polygon),
        position_(obs_pos),
        theta_(theta),
        length_(length),
        width_(width) {}
  ~StationaryObstacle() = default;

  const std::string& id() const { return id_; }
  e2e_noa::ObjectType type() const { return type_; }
  double x() const { return position_.x(); }
  double y() const { return position_.y(); }
  Vec2d pos() const { return position_; }
  double theta() const { return theta_; }
  double length() const { return length_; }
  double width() const { return width_; }
  math::Polygon2d polygon() const { return polygon_; }
  math::Box2d GetBoundingBox() const {
    return {position_, theta_, length_, width_};
  }
  double ds() const { return situation_.lon_distance(); }
  double dl() const { return situation_.lat_distance(); }
  double s() const { return sl_boundary_.center().s; }
  double l() const { return sl_boundary_.center().l; }
  double s_max() const { return situation_.lon_max(); }
  double s_min() const { return situation_.lon_min(); }

  const SLBoundary& sl_boundary() const { return sl_boundary_; }
  void set_sl_boundary(const SLBoundary& sl_boundary) {
    sl_boundary_ = sl_boundary;
  }

  const ObjectSituation& situation() const { return situation_; }
  ObjectSituation& mutable_situation() { return situation_; }

  std::vector<Vec2d> GetCornerPoints(const TrajectoryPoint& point) const;

 private:
  std::string id_;
  e2e_noa::ObjectType type_;
  math::Polygon2d polygon_;
  Vec2d position_;
  double theta_;
  double length_;
  double width_;
  ObjectSituation situation_;
  SLBoundary sl_boundary_;
};
using StationaryObstaclePtr = std::shared_ptr<StationaryObstacle>;
using ObstaclePtr = std::shared_ptr<Obstacle>;
using ObstaclePtr = std::shared_ptr<Obstacle>;

}  // namespace planning
}  // namespace ad_e2e

#endif
