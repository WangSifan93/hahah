#ifndef SPATIO_GRAPH_GEOMETRY_GRAPH_H_
#define SPATIO_GRAPH_GEOMETRY_GRAPH_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "common/log_data.h"
#include "container/strong_vector.h"
#include "glog/logging.h"
#include "trajectory_initialization.pb.h"
#include "math/vec.h"
#include "spacetime_search/spatio_graph/spatio_form.h"

namespace e2e_noa::planning {

DECLARE_STRONG_VECTOR(SpatioNode);
DECLARE_STRONG_VECTOR(SpatioEdge);

struct SpatioNode {
  SpatioNodeIndex index;
  Vec2d xy;
  double k;
  int station_index = 0;
  double lateral_offset = 0.0;
  double accumulated_s = 0.0;

  bool reachable = false;
  bool resampled = false;

  bool active = true;

  std::string DebugString() const {
    return absl::StrFormat(
        "SpatioNode %d(x: %.2f, y: %.2f, k: %.2f, station_index: %d, "
        "accumulated_s: %.2f, reachable: %d, resampled: %d, active: %d)",
        index.value(), xy.x(), xy.y(), k, station_index, accumulated_s,
        reachable, resampled, active);
  }
};

struct SpatioEdge {
  SpatioEdgeIndex index;
  SpatioNodeIndex start;
  SpatioNodeIndex end;
  const SpatioForm* spatio;
  bool truncated = false;

  bool active = true;

  bool isTruncated() const { return truncated; }
};

class SpatioGraph {
 public:
  virtual const SpatioNodeVector<SpatioNode>& nodes() const = 0;
  virtual absl::Span<const std::vector<SpatioNodeIndex>> nodes_layers()
      const = 0;
  virtual const SpatioNode& GetNode(SpatioNodeIndex index) const = 0;

  virtual const SpatioNode& GetStartNode() const = 0;
  virtual const SpatioEdgeVector<SpatioEdge>& edges() const = 0;
  virtual const SpatioEdge& GetEdge(SpatioEdgeIndex index) const = 0;

  virtual absl::Span<const SpatioEdgeIndex> GetOutgoingEdges(
      SpatioNodeIndex index) const = 0;
  virtual double GetMaxAccumulatedS() const = 0;

  virtual const SpatioGraphProto::EndInfo& GetSpatioGraphEndInfo() const = 0;

  virtual void DeactivateNode(SpatioNodeIndex) = 0;
  virtual void DeactivateEdge(SpatioEdgeIndex) = 0;

  virtual bool IsActive(SpatioNodeIndex) const = 0;
  virtual bool IsActive(SpatioEdgeIndex) const = 0;

  virtual ~SpatioGraph() {}

  virtual void ToProto(SpatioGraphProto* proto) const = 0;
};

class XYSpatioGraph : public SpatioGraph {
 public:
  XYSpatioGraph() = default;
  XYSpatioGraph(SpatioNodeVector<SpatioNode> nodes,
                std::vector<std::vector<SpatioNodeIndex>> nodes_layers,
                SpatioEdgeVector<SpatioEdge> edges,
                SpatioNodeVector<std::vector<SpatioEdgeIndex>> outgoing_edges,
                SpatioGraphProto::EndInfo end_info)
      : nodes_(std::move(nodes)),
        nodes_layers_(std::move(nodes_layers)),
        edges_(std::move(edges)),
        nodes_outgoing_edges_(std::move(outgoing_edges)),
        end_info_(std::move(end_info)) {}

  void ToProto(SpatioGraphProto* proto) const override;

  void ToProto(
      zark::e2e_noa::debug::
          NOADebugInfo_PlanningDebugInfo_SpatioSearchDebugInfo_SpatioGraphProto*
              proto) const;

  const SpatioNodeVector<SpatioNode>& nodes() const override { return nodes_; }
  const SpatioNode& GetNode(SpatioNodeIndex index) const override {
    return nodes_[index];
  }
  const SpatioNode& GetStartNode() const override;
  const SpatioEdgeVector<SpatioEdge>& edges() const override { return edges_; }
  const SpatioEdge& GetEdge(SpatioEdgeIndex index) const override {
    return edges_[index];
  }

  absl::Span<const std::vector<SpatioNodeIndex>> nodes_layers() const override {
    return nodes_layers_;
  }

  absl::Span<const SpatioEdgeIndex> GetOutgoingEdges(
      SpatioNodeIndex index) const override {
    return nodes_outgoing_edges_[index];
  }

  double GetMaxAccumulatedS() const override;

  const SpatioGraphProto::EndInfo& GetSpatioGraphEndInfo() const override {
    return end_info_;
  }

  inline void DeactivateNode(SpatioNodeIndex index) override {
    nodes_[index].active = false;
  }

  inline void DeactivateEdge(SpatioEdgeIndex index) override {
    edges_[index].active = false;
  }

  bool IsActive(SpatioNodeIndex index) const override {
    return nodes_[index].active;
  }
  bool IsActive(SpatioEdgeIndex index) const override {
    return edges_[index].active;
  }

 private:
  SpatioNodeVector<SpatioNode> nodes_;
  std::vector<std::vector<SpatioNodeIndex>> nodes_layers_;
  SpatioEdgeVector<SpatioEdge> edges_;

  SpatioNodeVector<std::vector<SpatioEdgeIndex>> nodes_outgoing_edges_;

  SpatioGraphProto::EndInfo end_info_;
};

}  
#endif
