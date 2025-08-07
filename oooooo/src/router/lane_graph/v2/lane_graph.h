#ifndef ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_
#define ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "container/flat_map.h"
#include "maps/semantic_map_defs.h"
#include "util/map_util.h"

namespace std {
inline std::string to_string(const std::pair<int, std::string>& vertex_id) {
  return absl::StrCat(vertex_id.first, "-", vertex_id.second);
}
}  // namespace std
namespace e2e_noa::planning {

static constexpr double kInfiniteCost = std::numeric_limits<double>::max();

template <typename VertexId>
struct TravelPath {
  std::vector<VertexId> vertices;
  double total_cost = kInfiniteCost;
};

template <typename VertexId>
struct VertexGraph {
 public:
  using VertexEdgeMap = FlatMap<VertexId, double>;
  using VertexSet = absl::flat_hash_set<VertexId>;

 public:
  void AddVertex(const VertexId& vert_id) {
    if (vertices_.contains(vert_id)) {
      LOG(WARNING) << std::to_string(vert_id) << " already exists!";
      return;
    }
    vertices_.insert(vert_id);
    edges_.emplace(vert_id, VertexEdgeMap());
  }

  void AddEdge(const VertexId& from_id, const VertexId& to_id, double cost) {
    if (!vertices_.contains(from_id) || !vertices_.contains(to_id)) {
      LOG(WARNING) << "please insert vertices first," << std::to_string(from_id)
                   << " -> " << std::to_string(to_id);
      return;
    }
    if (cost == kInfiniteCost) return;

    auto& edge_map = edges_[from_id];
    if (const auto res = edge_map.emplace(to_id, cost); !res.second) {
      res.first->second = std::min(res.first->second, cost);
    }
  }

  void RemoveEdge(const VertexId& from_id, const VertexId& to_id) {
    if (!IsEdgeValid(from_id, to_id)) {
      LOG(WARNING) << "Invalid edge " << std::to_string(from_id) << "->"
                   << std::to_string(to_id);
      return;
    }
    edges_[from_id].erase(to_id);
  }

  void RemoveEdgesFrom(const VertexId& from_id) {
    if (!vertices_.contains(from_id)) {
      LOG(WARNING) << std::to_string(from_id) << " does not exist!";
      return;
    }
    edges_[from_id].clear();
  }

  void ModifyEdge(const VertexId& from_id, const VertexId& to_id, double cost) {
    if (!IsEdgeValid(from_id, to_id)) {
      LOG(WARNING) << "Invalid edge ( " << std::to_string(from_id) << "->"
                   << std::to_string(to_id) << ") !";
      return;
    }
    edges_[from_id][to_id] = cost;
  }

  bool has_vertex(const VertexId& vert_id) const {
    return vertices_.contains(vert_id);
  }
  double edge_cost(const VertexId& from_id, const VertexId& to_id) const {
    if (const auto* edge = FindOrNull(edges_, from_id)) {
      if (const auto* v = FindOrNull(*edge, to_id)) {
        return *v;
      }
    }
    return kInfiniteCost;
  }
  const VertexSet& vertices() const { return vertices_; }
  const VertexEdgeMap& edges_from(const VertexId& from_id) const {
    DCHECK(vertices_.contains(from_id))
        << "Invalid vertex id " << std::to_string(from_id);
    return FindOrDieNoPrint(edges_, from_id);
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << vertices_.size() << " vertices in total\n";
    for (const auto& vertex : vertices_) {
      ss << "  " << vertex << ":\n";
      for (const auto& [to_id, cost] : FindOrDieNoPrint(edges_, vertex)) {
        ss << "    edge to " << to_id << " with cost " << cost << std::endl;
      }
    }
    return ss.str();
  }

 private:
  bool IsEdgeValid(const VertexId& from_id, const VertexId& to_id) const {
    return vertices_.contains(from_id) && vertices_.contains(to_id) &&
           ContainsKey(FindOrDieNoPrint(edges_, from_id), to_id);
  }

  VertexSet vertices_;
  absl::flat_hash_map<VertexId, VertexEdgeMap> edges_;
};

inline std::pair<int, std::string> ToVertId(int layer,
                                            const std::string& lane_id) {
  return {layer, lane_id};
}

inline std::pair<int, mapping::ElementId> FromVertId(
    std::pair<int, std::string> vertex_id) {
  return {vertex_id.first, mapping::ElementId(vertex_id.second)};
}

struct LaneGraph {
  using VertexId = std::pair<int, std::string>;
  using EdgeType = std::pair<VertexId, VertexId>;
  using LaneGraphLayers = std::vector<std::pair<int, double>>;

  static constexpr double kInfiniteCost =
      std::numeric_limits<double>::infinity();
  static constexpr double kDeadEndToTargetCost = 1e10;
  static const VertexId kInvalidVertexId;
  static const VertexId kTargetVertex;

  LaneGraphLayers layers;
  VertexGraph<VertexId> graph;
  absl::flat_hash_set<VertexId> fork_lc_verts;
  absl::flat_hash_set<EdgeType> fork_lk_edges;
};

const LaneGraph::VertexId& InvalidVertexId(LaneGraph::VertexId);

}  // namespace e2e_noa::planning

#endif
