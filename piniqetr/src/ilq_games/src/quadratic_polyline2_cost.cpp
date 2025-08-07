/////////////////////////////////////////////////////////////////////////////
//
// Quadratic penalty on distance from a given Polyline2.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/cost/quadratic_polyline2_cost.h"

#include <tuple>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/geometry/polyline2.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

float QuadraticPolyline2Cost::Evaluate(const VectorXf& input) const {
  CHECK_LT(xidx_, input.size());
  CHECK_LT(yidx_, input.size());

  // Compute signed squared distance by finding closest point.
  float signed_squared_distance;
  bool is_endpoint;
  polyline_.ClosestPoint(Point2(input(xidx_), input(yidx_)), nullptr, nullptr,
                         &signed_squared_distance, &is_endpoint);

  if (is_endpoint) {
    // If the is_endpoint flag is raised, we set the signed_squared_distance to
    // 0.0.
    signed_squared_distance = 0.0;
  }

  return 0.5 * weight_ * std::abs(signed_squared_distance);
}

void QuadraticPolyline2Cost::Quadraticize(const VectorXf& input, MatrixXf* hess,
                                          VectorXf* grad) const {
  CHECK_LT(xidx_, input.size());
  CHECK_LT(yidx_, input.size());

  CHECK_NOTNULL(hess);
  CHECK_NOTNULL(grad);
  CHECK_EQ(input.size(), hess->rows());
  CHECK_EQ(input.size(), hess->cols());
  CHECK_EQ(input.size(), grad->size());

  // Unpack current position and find closest point / segment.
  const Point2 current_position(input(xidx_), input(yidx_));

  bool is_vertex;
  bool is_endpoint;
  LineSegment2 segment(Point2(0.0, 0.0), Point2(1.0, 1.0));
  const Point2 closest_point = polyline_.ClosestPoint(
      current_position, &is_vertex, &segment, nullptr, &is_endpoint);

  // First check whether the closest point is a endpoint of the polyline.
  if (is_endpoint) return;

  // Handle cases separately depending on whether or not closest point is
  // a vertex of the polyline.
  float ddx = weight_;
  float ddy = weight_;
  float dxdy = 0.0;
  float dx = weight_ * (current_position.x() - closest_point.x());
  float dy = weight_ * (current_position.y() - closest_point.y());

  if (!is_vertex) {
    const Point2 relative = current_position - segment.FirstPoint();
    const Point2& unit_segment = segment.UnitDirection();

    // Handle Hessian first.
    ddx = weight_ * unit_segment.y() * unit_segment.y();
    ddy = weight_ * unit_segment.x() * unit_segment.x();
    dxdy = -weight_ * unit_segment.x() * unit_segment.y();

    // Handle gradient.
    const float w_cross = weight_ * (relative.x() * unit_segment.y() -
                                     relative.y() * unit_segment.x());

    dx = w_cross * unit_segment.y();
    dy = -w_cross * unit_segment.x();
  }

  (*grad)(xidx_) += dx;
  (*grad)(yidx_) += dy;

  (*hess)(xidx_, xidx_) += ddx;
  (*hess)(yidx_, yidx_) += ddy;
  (*hess)(xidx_, yidx_) += dxdy;
  (*hess)(yidx_, xidx_) += dxdy;
}

}  // namespace e2e_noa::planning
