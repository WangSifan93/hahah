#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_

#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "container/strong_vector.h"
#include "glog/logging.h"
#include "trajectory_initialization.pb.h"
#include "spacetime_search/spacetime_form.h"
#include "spacetime_search/spacetime_state.h"
#include "spacetime_search/spatio_graph/spatio_graph.h"

namespace e2e_noa::planning {
DECLARE_STRONG_VECTOR(SpacetimeNode);
DECLARE_STRONG_VECTOR(SpacetimeEdge);

struct SpacetimeNode {
  SpacetimeNodeIndex index;
  SpacetimeState state;
  SpatioNodeIndex geom_index;
};

struct SpacetimeEdge {
  SpacetimeNodeIndex start;
  SpacetimeNodeIndex end;
  const SpacetimeForm* spacetime;
  SpacetimeEdgeIndex prev_edge;
};

class SpacetimeGraph {
 public:
  explicit SpacetimeGraph(const SpatioGraph* geom_graph)
      : spatio_graph_(geom_graph) {}
  virtual int node_size() const = 0;
  virtual int edge_size() const = 0;
  virtual const SpacetimeNode& GetSpacetimeNode(SpacetimeNodeIndex i) const = 0;
  virtual const SpacetimeEdge& GetSpacetimeEdge(SpacetimeEdgeIndex i) const = 0;
  virtual SpacetimeEdge* GetMutableSpacetimeEdge(SpacetimeEdgeIndex i) = 0;
  virtual absl::Span<const SpacetimeEdgeIndex> GetOutgoingEdges(
      SpacetimeNodeIndex i) const = 0;
  virtual SpacetimeNodeIndex AddSpacetimeNode(SpacetimeState node,
                                              SpatioNodeIndex geom_index) = 0;
  virtual SpacetimeEdgeIndex AddSpacetimeEdge(
      SpacetimeNodeIndex start_node_index, SpacetimeNodeIndex end_node_index,
      const SpacetimeForm* spacetime_form, SpatioNodeIndex end_geom_index,
      SpacetimeEdgeIndex prev_edge) = 0;

  virtual const SpatioGraph* spatio_graph() const { return spatio_graph_; }

  virtual ~SpacetimeGraph() {}

  virtual void ToProto(SpacetimeGraphProto* proto) const = 0;

 protected:
  const SpatioGraph* spatio_graph_;
};

class XYTSpacetimeGraph : public SpacetimeGraph {
 public:
  explicit XYTSpacetimeGraph(const SpatioGraph* geom_graph)
      : SpacetimeGraph(geom_graph) {}

  int node_size() const override { return nodes_.size(); }
  int edge_size() const override { return edges_.size(); }

  const SpacetimeNode& GetSpacetimeNode(SpacetimeNodeIndex i) const override {
    return nodes_[i];
  }
  const SpacetimeEdge& GetSpacetimeEdge(SpacetimeEdgeIndex i) const override {
    return edges_[i];
  }
  SpacetimeEdge* GetMutableSpacetimeEdge(SpacetimeEdgeIndex i) override {
    return &edges_[i];
  }

  absl::Span<const SpacetimeEdgeIndex> GetOutgoingEdges(
      SpacetimeNodeIndex i) const override {
    CHECK(outgoing_edges_.valid_index(i));
    return outgoing_edges_[i];
  }
  SpacetimeNodeIndex AddSpacetimeNode(SpacetimeState node,
                                      SpatioNodeIndex geom_index) override;
  SpacetimeEdgeIndex AddSpacetimeEdge(SpacetimeNodeIndex start_node_index,
                                      SpacetimeNodeIndex end_node_index,
                                      const SpacetimeForm* spacetime_form,
                                      SpatioNodeIndex end_geom_index,
                                      SpacetimeEdgeIndex prev_edge) override;

  void ToProto(SpacetimeGraphProto* proto) const override;

 private:
  std::vector<SpacetimeEdgeIndex>* GetOrCreateOutgoingEdge(
      SpacetimeNodeIndex i);

  SpacetimeNodeVector<SpacetimeNode> nodes_;
  SpacetimeNodeVector<std::vector<SpacetimeEdgeIndex>> outgoing_edges_;
  SpacetimeEdgeVector<SpacetimeEdge> edges_;
};

}  
#endif
