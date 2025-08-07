#ifndef EGO_STATE_H_
#define EGO_STATE_H_
#include "math/geometry/box2d.h"
#include "nodes/planner/city/msg_proxy.h"
#include "plan/planner_world.h"
#include "vehicle.pb.h"
namespace e2e_noa::planning {
class EgoState {
 public:
  EgoState();

  ~EgoState() = default;

  void update(const PlannerWorldInput& input);

  Vec2d ego_pos() const { return ego_pos_; }

  double ego_theta() const { return ego_theta_; }

  Box2d ego_box() const { return ego_box_; }

  VehicleGeometryParamsProto vehicle_geometry() const {
    return vehicle_geometry_;
  }

  ApolloTrajectoryPointProto plan_start_point() const { return start_point_; }

  double start_point_velocity() const { return start_point_velocity_; }

 private:
  Vec2d ego_pos_;
  double ego_theta_;
  Box2d ego_box_;
  VehicleGeometryParamsProto vehicle_geometry_;
  ApolloTrajectoryPointProto start_point_;
  double start_point_velocity_;
};
}  // namespace e2e_noa::planning
#endif