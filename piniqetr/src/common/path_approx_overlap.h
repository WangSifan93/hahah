#ifndef ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_
#define ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_

#include <optional>
#include <vector>

#include "common/path_approx.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"

namespace e2e_noa {
namespace planning {

struct AgentOverlap {
  double first_ra_s = 0.0;
  double last_ra_s = 0.0;

  double ra_heading = 0.0;

  double lat_dist = 0.0;
};

std::vector<AgentOverlap> ComputeAgentOverlapsWithBuffer(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius);

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff, std::pair<int, int>& min_max_index_res);

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff);

bool HasPathApproximationOverlapWithPolygon(const PathApprox& path_approx,
                                            double step_length, int first_index,
                                            int last_index,
                                            const Polygon2d& polygon,
                                            double search_radius);

}  // namespace planning
}  // namespace e2e_noa

#endif
