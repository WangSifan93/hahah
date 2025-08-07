#ifndef SPACETIME_SEARCH_COST_PROVIDER_H_
#define SPACETIME_SEARCH_COST_PROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "common/path_sl_boundary.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "object/spacetime_trajectory_manager.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "spacetime_search/collision_checker.h"
#include "spacetime_search/cost_feature.h"
#include "spacetime_search/ref_speed_table.h"
#include "spacetime_search/spacetime_form.h"
#include "spacetime_search/spatio_graph/spatio_form.h"
#include "vehicle.pb.h"
namespace e2e_noa::planning {

class CostProviderBase {
 public:
  absl::Span<const std::string> cost_names() const { return cost_names_; }

  absl::Span<const double> weights() const { return weights_; }

  void ComputeDpCost(double start_t, const SpacetimeForm* spacetime_form,
                     absl::Span<double> cost) const;
  IgnoreTrajMap ComputeInteractiveDpCost(double start_t,
                                         const SpacetimeForm* spacetime_form,
                                         const IgnoreTrajMap& ignored_trajs,
                                         absl::Span<double> cost) const;
  void ComputeDpLeadingObjCost(double start_t,
                               const SpacetimeForm* spacetime_form,
                               absl::Span<double> cost) const;
  void ComputeRefLineCost(const SpatioForm* spatio_form, bool terminating,
                          absl::Span<double> cost) const;

 protected:
  template <typename Config>
  void BuildWeightTable(const Config& cost_config);

  std::vector<std::unique_ptr<FeatureCost>> features_;

 private:
  std::vector<std::string> cost_names_;

  std::vector<double> weights_;

  std::vector<int> feature_size_;
};

class CostProvider : public CostProviderBase {
 public:
  CostProvider(
      const PlanPassage& plan_passage,
      const InitializationConfig& initialization_params,
      const SpacetimeConstraintParamsProto& spacetime_constraint_params,
      const std::vector<double>& stop_s_vec,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const std::vector<std::string>& leading_trajs,
      const VehicleGeometryParamsProto& vehicle_geom,
      const CollisionChecker* collision_checker, const PathSlBoundary* path_sl,
      const RefSpeedTable* ref_speed_table,
      const InitializationSceneType init_scene_type, bool is_lane_change,
      double max_accumulated_s, bool is_post_evaluation = false);
};

}  

#endif
