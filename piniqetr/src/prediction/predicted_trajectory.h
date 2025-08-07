#ifndef ST_PLANNING_PREDICTION_PREDICTED_TRAJECTORY
#define ST_PLANNING_PREDICTION_PREDICTED_TRAJECTORY

#include <algorithm>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction.pb.h"
#include "prediction_common.pb.h"

namespace e2e_noa {
namespace prediction {
class alignas(64) PredictedTrajectoryPoint
    : public planning::TrajectoryPointWithAcceleration {
 public:
  PredictedTrajectoryPoint() = default;
  explicit PredictedTrajectoryPoint(
      const PredictedTrajectoryPointProto& proto) {
    FromProto(proto);
  }

  explicit PredictedTrajectoryPoint(
      const planning::TrajectoryPointWithAcceleration& point)
      : planning::TrajectoryPointWithAcceleration(point) {}

  void FromProto(const PredictedTrajectoryPointProto& proto);
  void ToProto(PredictedTrajectoryPointProto* proto) const;
};

class PredictedTrajectory {
 public:
  PredictedTrajectory() {}
  explicit PredictedTrajectory(const PredictedTrajectoryProto& proto) {
    FromProto(0.0, proto);
  }

  explicit PredictedTrajectory(double shift_time,
                               const PredictedTrajectoryProto& proto) {
    FromProto(shift_time, proto);
  }

  double probability() const { return probability_; }
  PredictionType type() const { return type_; }
  bool is_reversed() const { return is_reversed_; }
  int index() const { return index_; }
  const std::vector<PredictedTrajectoryPoint>& points() const {
    return points_;
  }
  void set_probability(double probability) { probability_ = probability; }
  void set_index(int index) { index_ = index; }
  void set_type(PredictionType type) { type_ = type; }

  std::vector<PredictedTrajectoryPoint>* mutable_points() { return &points_; }
  TrajectoryIntention intention() const { return intention_; }

  std::optional<PredictedTrajectoryPoint> EvaluateByTime(double t) const;

  std::string DebugString() const;

  void FromProto(const PredictedTrajectoryProto& proto);
  void ToProto(PredictedTrajectoryProto* proto, bool compress_traj) const;

 private:
  void FromProto(double shift_time, const PredictedTrajectoryProto& proto);

  double probability_ = 0.0;
  PredictionType type_;
  int index_;
  std::vector<PredictedTrajectoryPoint> points_;

  bool is_reversed_ = false;
  TrajectoryIntention intention_ = TrajectoryIntention::INTENTION_UNKNOWN;
};

}  // namespace prediction
}  // namespace e2e_noa

#endif
