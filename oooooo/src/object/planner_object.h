#ifndef ST_PLANNING_OBJECT_PLANNER_OBJECT
#define ST_PLANNING_OBJECT_PLANNER_OBJECT

#include <optional>
#include <string>
#include <vector>

#include "math/geometry/aabox2d.h"
#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"
#include "perception.pb.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "planner_object.pb.h"
#include "prediction.pb.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"

namespace e2e_noa {
namespace planning {

class PlannerObject {
 public:
  PlannerObject() = default;

  explicit PlannerObject(prediction::ObjectPrediction object_prediction);

  void ToPlannerObjectProto(PlannerObjectProto* planner_object_proto) const;

  const std::string& id() const { return object_proto_.id(); }

  int num_trajs() const { return prediction_.trajectories().size(); }

  const prediction::PredictedTrajectory& traj(int i) const {
    return prediction_.trajectories()[i];
  }

  const ObjectStopTimeProto& stop_time_info() const {
    return prediction_.stop_time();
  }

  const prediction::ObjectLongTermBehavior& long_term_behavior() const {
    return prediction_.long_term_behavior();
  }

  std::optional<int> MostLikelyTrajectory() const;

  ObjectType type() const { return object_proto_.type(); }
  void set_type(ObjectType otype) { object_proto_.set_type(otype); }

  const ObjectProto& object_proto() const { return object_proto_; }

  const TrajectoryPointWithAcceleration& pose() const { return pose_; }

  Vec2d velocity() const { return velocity_; }

  const Polygon2d& contour() const { return contour_; }

  const AABox2d& aabox() const { return aabox_; }

  void set_stationary(bool stationary) { is_stationary_ = stationary; }

  bool is_stationary() const { return is_stationary_; }

  const prediction::ObjectPrediction& prediction() const { return prediction_; }
  prediction::ObjectPrediction* mutable_prediction() { return &prediction_; }

  const Box2d& bounding_box() const { return bounding_box_; }

  const Box2d& perception_bbox() const { return perception_bbox_; }

  double proto_timestamp() const { return object_proto_.timestamp(); }

  bool is_sim_agent() const { return is_sim_agent_; }

  bool is_large_vehicle() const { return is_large_vehicle_; }

  const std::string& base_id() const { return base_id_; }

 private:
  void FromPrediction(const prediction::ObjectPrediction& prediction);

  bool is_stationary_ = false;
  TrajectoryPointWithAcceleration pose_;
  Vec2d velocity_;
  Polygon2d contour_;
  Box2d bounding_box_;
  Box2d perception_bbox_;
  AABox2d aabox_;
  prediction::ObjectPrediction prediction_;

  ObjectProto object_proto_;

  bool is_sim_agent_ = false;
  bool is_large_vehicle_ = false;
  std::string base_id_;

  friend class PlannerObjectBuilder;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
