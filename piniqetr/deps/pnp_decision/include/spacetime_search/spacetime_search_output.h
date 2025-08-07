#ifndef SPACETIME_SEARCH_OUTPUT_H_
#define SPACETIME_SEARCH_OUTPUT_H_

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "constraint.pb.h"
#include "trajectory_initialization.pb.h"
#include "spacetime_search/cost_provider.h"
#include "spacetime_search/dp_spacetime_searcher_defs.h"
#include "spacetime_search/select_nudge_object.h"
#include "spacetime_search/spacetime_graph.h"
#include "spacetime_search/spacetime_state.h"
#include "util/hmi_content_util.h"
namespace e2e_noa::planning {

struct SpacetimeSearchOutput {
  absl::Status result_status = absl::OkStatus();

  NudgeInfos nudge_info;

  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> leader_set;
  double follower_max_decel = 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;

  bool is_lc_pause = false;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  SpacetimeEdgeIndex best_last_edge_index;

  struct SearchCost {
    std::vector<double> feature_cost;
    double cost_to_come = 0.0;
    double TotalCost() const { return cost_to_come; }
  };

  SpacetimeEdgeVector<SearchCost> search_costs;
  double min_cost;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;

  std::unique_ptr<SpacetimeGraph> spacetime_graph;

  std::unique_ptr<RefSpeedTable> ref_speed_table;

  std::unique_ptr<CostProvider> cost_provider;

  std::vector<SpacetimeEdgeIndex> terminated_edge_idxes;
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
  std::vector<double> top_k_total_costs;
  std::vector<SpacetimeEdgeIndex> top_k_edges;

  struct MultiTrajCandidate {
    std::vector<ApolloTrajectoryPointProto> trajectory;
    std::vector<std::string> leading_traj_ids;
    double total_cost;
    SpacetimeEdgeIndex last_edge_index;
    std::vector<double> feature_costs;
    double final_cost;
    IgnoreTrajMap ignored_trajs;
  };
  std::vector<MultiTrajCandidate> multi_traj_candidates;

  struct IsFilteredReasons {
    bool is_out_of_bound = false;
    bool is_violating_stop_constraint = false;
    bool is_dynamic_collision = false;
    bool is_violating_leading_objects = false;
  };
  struct TrajectoryEvaluationDumping {
    double weighted_total_cost;
    std::vector<double> dumped_weights;
    std::vector<double> feature_costs;
    std::vector<ApolloTrajectoryPointProto> traj;
    IsFilteredReasons is_filtered_reasons;
  };
  TrajectoryEvaluationDumping expert_evaluation;
  std::vector<TrajectoryEvaluationDumping> candidates_evaluation;
  SpeedResponseStyle speed_response_style;
  PlannerStatusProto::PlannerStatusCode lc_status_code = PlannerStatusProto::OK;
};

struct TrajectoryInitializationOutput {
  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> leader_set;
  double follower_max_decel = 0.0;
  bool is_lc_pause = false;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  InitializationStateProto initialization_state;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
  NudgeInfos nudge_info;
  SpeedResponseStyle speed_response_style;
  PlannerStatusProto::PlannerStatusCode lc_status_code = PlannerStatusProto::OK;
  bool is_init_follow_scene = false;
  std::string lc_lead_obj_id = "none";
};

}  

#endif
