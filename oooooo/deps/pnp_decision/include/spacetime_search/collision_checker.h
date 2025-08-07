#ifndef SPACETIME_SEARCH_COLLISION_CHECKER_H_
#define SPACETIME_SEARCH_COLLISION_CHECKER_H_

#include <vector>

#include "absl/types/span.h"
#include "math/geometry/aabox2d.h"
#include "math/geometry/box2d.h"
#include "object/object_vector.h"
#include "object/spacetime_object_state.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "spacetime_search/dp_spacetime_searcher_defs.h"
#include "spacetime_search/spacetime_state.h"
#include "spacetime_search/spatio_graph/spatio_state.h"
#include "vehicle.pb.h"
namespace e2e_noa {
namespace planning {

struct CollisionInfo {
  struct ObjectCollision {
    int time;
    const SpacetimeObjectTrajectory* traj;
    std::optional<CollisionConfiguration> collision_configuration =
        std::nullopt;
  };

  std::vector<ObjectCollision> collision_objects;
  void Clear() { collision_objects.clear(); }
};

CollisionConfiguration DetermineConfiguration(const Box2d& av_box,
                                              const Box2d& obj_box);

class CollisionChecker {
 public:
  virtual void UpdateStationaryObjectBuffer(double stationary_obj_buffer) = 0;

  virtual void UpdateMovingObjectBuffer(double moving_obj_buffer) = 0;

  virtual void CheckCollisionWithTrajectories(
      double init_t, absl::Span<const SpacetimeState> states,
      const IgnoreTrajMap& ignored_trajs, CollisionInfo* info) const = 0;

  virtual void CheckCollisionWithStationaryObjects(
      absl::Span<const SpatioState> states, CollisionInfo* info) const = 0;

  virtual ~CollisionChecker() {}
};

struct SampledTrajectory;

class BoxGroupCollisionChecker : public CollisionChecker {
 public:
  struct ObjectBoxGroup {
    const SampledTrajectory* sampled_traj = nullptr;

    std::vector<AABox2d> aaboxes;
  };

  BoxGroupCollisionChecker(
      const SpacetimePlannerObjectTrajectories* st_planner_obj_traj,
      const VehicleGeometryParamsProto* vehicle_geometry, int sample_step,
      double stationary_obj_buffer, double moving_obj_buffer,
      const ApolloTrajectoryPointProto& start_point,
      bool delete_reverse = false);

  void UpdateStationaryObjectBuffer(double stationary_obj_buffer) override;

  void UpdateMovingObjectBuffer(double moving_obj_buffer) override;

  void CheckCollisionWithTrajectories(double init_t,
                                      absl::Span<const SpacetimeState> states,
                                      const IgnoreTrajMap& ignored_trajs,
                                      CollisionInfo* info) const override;

  void CheckCollisionWithStationaryObjects(absl::Span<const SpatioState> states,
                                           CollisionInfo* info) const override;

  int sample_step() const { return sample_step_; }

 private:
  static constexpr int kGroupSize = 16;

  int sample_step_;

  double angle_sample_step_;

  double inv_angle_sample_step_;

  std::vector<Box2d> boxes_by_angles_;

  ObjectVector<ObjectBoxGroup> traj_box_groups_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;

  std::vector<SampledTrajectory> sampled_trajs_;
  std::vector<SpacetimeObjectState> stationary_obj_states_;
};

struct SampledTrajectory {
  std::vector<SpacetimeObjectState> states;

  const SpacetimeObjectTrajectory* traj;
};
std::vector<SampledTrajectory> SampleTrajectory(
    absl::Span<const SpacetimeObjectTrajectory* const> trajs, int sample_step);

}  
}  

#endif
