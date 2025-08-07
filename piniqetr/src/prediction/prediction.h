#ifndef PREDICTION_PREDICTION_H_
#define PREDICTION_PREDICTION_H_

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

#include "math/geometry/polygon2d.h"
#include "perception.pb.h"
#include "prediction.pb.h"
#include "prediction/predicted_trajectory.h"
#include "prediction_common.pb.h"

namespace e2e_noa {
namespace prediction {

struct ObjectLongTermBehavior {
  double avg_speed;
  double obs_duration = 0.0;
  std::vector<double> accel_history;

  void FromProto(const ObjectLongTermBehaviorProto& proto) {
    avg_speed = proto.average_speed();
    obs_duration = proto.observation_duration();

    accel_history.reserve(proto.accel_history_size());
    for (double accel : proto.accel_history()) {
      accel_history.push_back(accel);
    }
  }

  void ToProto(ObjectLongTermBehaviorProto* proto) const {
    proto->set_average_speed(avg_speed);
    proto->set_observation_duration(obs_duration);

    auto* accel_hist = proto->mutable_accel_history();
    accel_hist->Reserve(accel_history.size());
    for (double accel : accel_history) accel_hist->Add(accel);
  }
};

class ObjectPrediction {
 public:
  ObjectPrediction() = default;

  explicit ObjectPrediction(const ObjectProto& object_proto)
      : perception_object_(object_proto) {}

  ObjectPrediction(std::vector<PredictedTrajectory> trajectories,
                   const ObjectProto& object);

  explicit ObjectPrediction(const ObjectPredictionProto& proto);

  ObjectPrediction(const ObjectPredictionProto& proto, double shift_time,
                   const ObjectProto& object);

  const ObjectProto& perception_object() const { return perception_object_; }

  double timestamp() const { return perception_object_.timestamp(); }

  const std::string& id() const { return perception_object_.id(); }
  const std::vector<PredictedTrajectory>& trajectories() const {
    return trajectories_;
  }
  double trajectory_prob_sum() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 0.0,
                           [](const double sum, const auto& traj) {
                             return sum + traj.probability();
                           });
  }
  double trajectory_max_prob() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 0.0,
                           [](const double max, const auto& traj) {
                             return std::max(max, traj.probability());
                           });
  }
  double trajectory_min_prob() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 1.0,
                           [](const double min, const auto& traj) {
                             return std::min(min, traj.probability());
                           });
  }

  const Polygon2d& contour() const { return contour_; }

  void set_id(std::string id) { perception_object_.set_id(id); }

  std::vector<PredictedTrajectory>* mutable_trajectories() {
    return &trajectories_;
  }

  std::vector<std::string> DebugStringList() const;

  const ObjectStopTimeProto& stop_time() const { return stop_time_; }
  const ObjectLongTermBehavior& long_term_behavior() const {
    return long_term_behavior_;
  }

  void FromProto(const ObjectPredictionProto& proto);
  void ToProto(ObjectPredictionProto* proto) const;

 private:
  void FromProto(const ObjectPredictionProto& proto, double shift_time,
                 const ObjectProto& object);

  std::vector<PredictedTrajectory> trajectories_;
  Polygon2d contour_;

  ObjectProto perception_object_;

  ObjectStopTimeProto stop_time_;

  ObjectLongTermBehavior long_term_behavior_;
};

}  // namespace prediction
}  // namespace e2e_noa

#endif
