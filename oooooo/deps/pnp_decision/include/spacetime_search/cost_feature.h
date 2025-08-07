#ifndef SPACETIME_SEARCH_COST_FEATURE_H_
#define SPACETIME_SEARCH_COST_FEATURE_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "descriptor/constraint_manager.h"
#include "trajectory_initialization.pb.h"
#include "spacetime_search/ref_speed_table.h"
#include "spacetime_search/spacetime_graph.h"
#include "spacetime_search/spacetime_search_input.h"
#include "spacetime_search/spatio_graph/spatio_graph.h"

namespace e2e_noa::planning {

struct SpatioEdgeInfo {
  const SpatioForm* spatio_form = nullptr;
  std::vector<SpatioState> states;
  bool terminating = false;
};

struct SpacetimeEdgeInfo {
  double start_t = 0.0;
  const SpacetimeForm* spacetime_form = nullptr;
  std::vector<SpacetimeState> const_interval_states;
  std::vector<SpacetimeState> equal_interval_states;
};

class FeatureCost {
 public:
  explicit FeatureCost(std::string name) : name_(std::move(name)) {}

  virtual void ComputeCost(const SpacetimeEdgeInfo& edge_info,
                           absl::Span<double> cost) const {}

  virtual void ComputeCost(const SpatioEdgeInfo& edge_info,
                           absl::Span<double> cost) const {}

  std::string name() const { return name_; }

  virtual ~FeatureCost() {}

 private:
  std::string name_;
};

}  
#endif
