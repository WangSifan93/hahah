#include <router/lane_graph/v2/lane_graph.h>
namespace e2e_noa::planning {
const LaneGraph::VertexId LaneGraph::kInvalidVertexId = ToVertId(-1, "");
const LaneGraph::VertexId LaneGraph::kTargetVertex = ToVertId(-1, "");

const LaneGraph::VertexId& InvalidVertexId(LaneGraph::VertexId) {
  return LaneGraph::kInvalidVertexId;
}

}  // namespace e2e_noa::planning
