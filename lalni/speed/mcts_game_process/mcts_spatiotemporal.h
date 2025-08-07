
#ifndef PLANNER_SPEED_MCTS_SPATIOTEMPORAL_H_
#define PLANNER_SPEED_MCTS_SPATIOTEMPORAL_H_

#include <cmath>
#include <vector>

#include "math/double.h"
#include "mcts.h"
#include "plan/discretized_path.h"
#include "vehicle_status_update_model/vehicle_simulation_model.h"
#include "vru_status_update_model/vru_status_update_model.h"
namespace e2e_noa::planning {

class MCTSSpatiotemporal : public MCTS {
 public:
  MCTSSpatiotemporal() = delete;
  MCTSSpatiotemporal(const SpeedPlanningParamsProto::MCTSVruParamsProto *params)
      : params_(params) {}

  ~MCTSSpatiotemporal() = default;

  void InitMCTSSpatiotemporal(
      const double min_v, const double max_v, const double time_step,
      const double jerk, const double leader_v, const double follower_v,
      const DiscretizedPath *leader_path, const double goal_v,
      const double goal_pos_x, const double goal_pos_y,
      const double goal_heading, const double leader_x, const double leader_y,
      const double follower_x, const double follower_y,
      const double follower_length, const double follower_width,
      const double heading_diff,
      const StOverlapMetaProto::OverlapSource &overlap_source,
      const StBoundaryProto::ObjectType &vru_type,
      const std::vector<const SpacetimeObjectTrajectory *> &other_agents);

  void SetInteractionZone(const InteractionZone interaction_zone) {
    interaction_zone_ = interaction_zone;
  }

  void SetEgoPath(const DiscretizedPath &ego_path) { ego_path_ = ego_path; }

 private:
  bool PrePruning(const MCTSNodePtr &node, const double leader_action) override;

  void CreateChildNode(const MCTSNodePtr &parent, const double leader_action,
                       MCTSNodePtr &child) override;

  void UpdateChildNode(const MCTSNodeConstPtr &parent,
                       const MCTSNodePtr &child) override;

  void UpdateNodeStepReward(const MCTSNodePtr &node) override;

  double CalculateEgoReward(const MCTSNodePtr &node);
  double CalculateOtherReward(const MCTSNodePtr &node);

  double CalculateSafetyReward(const MCTSNodeConstPtr &node,
                               const IntersectionResult &intersectionresult);

  double CalculateEfficiencyReward(
      const MCTSNodeConstPtr &node,
      const IntersectionResult &intersectionresult);

  double CalculateAccelerationReward(const MCTSNodeConstPtr &node);

  double CalculateJerkReward(const MCTSNodeConstPtr &node);

  double CalculateActionSmoothReward(const MCTSNodeConstPtr &node);

  double CalculateEndReward(const MCTSNodeConstPtr &node);

  void UpdateLeaderState(const MCTSNodeConstPtr &parent,
                         const double leader_action, NodeState &state);

  bool IsLeaf(const MCTSNodePtr &node);

  void FindIntersectionAndCalculate(
      const double follower_x, const double follower_y,
      const double follower_heading, const double follower_speed,
      const double leader_x, const double leader_y, const double leader_speed,
      IntersectionResult &intersection_result);

 private:
  const SpeedPlanningParamsProto::MCTSVruParamsProto *params_ = nullptr;
  double time_step_ = 0.5;
  double leader_max_v_ = 16.6;
  double leader_cur_v_ = 16.6;
  double follower_cur_v_ = 5.0;
  double leader_start_x_ = 0.0;
  double leader_start_y_ = 0.0;
  double follower_start_x_ = 0.0;
  double follower_start_y_ = 0.0;
  double follower_length_ = 2.0;
  double follower_width_ = 1.0;
  double goal_heading_ = 0.0;
  double heading_diff_ = 0.0;
  StOverlapMetaProto::OverlapSource overlap_source_ =
      StOverlapMetaProto::UNKNOWN_SOURCE;
  StBoundaryProto::ObjectType vru_type_ = StBoundaryProto::UNKNOWN_OBJECT;
  VehicleSimulationModel leader_model_;
  const DiscretizedPath *leader_path_ = nullptr;
  std::vector<const SpacetimeObjectTrajectory *> other_agents_;
  InteractionZone interaction_zone_ = InteractionZone::Unknown;
  DiscretizedPath ego_path_;

  double goal_v_ = 3.0;
  Vec2d goal_position_;
};
}  // namespace e2e_noa::planning

#endif
