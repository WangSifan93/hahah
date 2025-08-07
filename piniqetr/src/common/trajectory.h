#ifndef AD_E2E_PLANNING_COMMON_TRAJECTORY_H_
#define AD_E2E_PLANNING_COMMON_TRAJECTORY_H_
#include <memory>

#include "common/type_def.h"

namespace ad_e2e {
namespace planning {
class Trajectory {
 public:
  Trajectory() = default;
  explicit Trajectory(const std::vector<TrajectoryPoint> &points);
  explicit Trajectory(std::vector<TrajectoryPoint> &&points);
  ~Trajectory() = default;

  void Reset();

  double timestamp() const { return timestamp_; }
  void set_timestamp(const double &timestamp) { timestamp_ = timestamp; }

  const std::vector<TrajectoryPoint> &points() const { return points_; }
  void set_points(const std::vector<TrajectoryPoint> &points);
  void set_points(std::vector<TrajectoryPoint> &&points);

  void set_sl_bounds(std::vector<SLBoundary> &&sl_bounds) {
    sl_bounds_ = std::move(sl_bounds);
  }

  void set_ref_points(std::vector<PathPoint> &&ref_points) {
    ref_points_ = std::move(ref_points);
  }

  double probability() const { return probability_; }
  void set_probability(const double &probability) {
    probability_ = probability;
  }

  size_t point_size() const { return points_.size(); }

  const SLBoundary &GetSLBoundAtIdx(const size_t &idx) const;

  const PathPoint &GetRefPointAtIdx(const size_t &idx) const;

  TrajectoryPoint GetPointAtS(const double &accu_s) const;

  size_t GetNearestIndexAtS(const double &accu_s) const;

  TrajectoryPoint GetPointAtTime(const double &t) const;

  SLPoint GetSLPointAtTime(const double &t) const;

  size_t GetNearestIndexAtTime(const double &t) const;

  double GetDistance(const Point2d &point, TrajectoryPoint *nearest_pt) const;

  size_t GetNearestIndex(const Point2d &point) const;

  bool IsValid() const;

  void set_intention(const ObstacleIntention &intention) {
    intention_ = intention;
  }
  const ObstacleIntention &intention() const { return intention_; }

  double length() const {
    return points_.size() < 2 ? 0.0 : points_.back().accum_s;
  }

 private:
  void ComputeAccumulatedS();

 private:
  double timestamp_ = 0.0;
  double probability_ = 1.0;
  TrajectoryPoint empty_traj_pt_;
  SLBoundary empty_sl_bound_;
  PathPoint empty_path_pt_;
  std::vector<TrajectoryPoint> points_;
  std::vector<SLBoundary> sl_bounds_;
  std::vector<PathPoint> ref_points_;

  ObstacleIntention intention_ = INTENTION_UNKNOWN;
};
using TrajectoryPtr = std::shared_ptr<Trajectory>;
}  // namespace planning
}  // namespace ad_e2e
#endif
