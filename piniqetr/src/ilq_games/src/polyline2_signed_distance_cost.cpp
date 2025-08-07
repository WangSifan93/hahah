/////////////////////////////////////////////////////////////////////////////
//
// Signed distance from a given polyline.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/cost/polyline2_signed_distance_cost.h"

#include <tuple>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/geometry/polyline2.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

float Polyline2SignedDistanceCost::Evaluate(const VectorXf& input) const {
  CHECK_LT(xidx_, input.size());
  CHECK_LT(yidx_, input.size());

  // Compute signed squared distance by finding closest point.
  float signed_squared_distance;
  polyline_.ClosestPoint(Point2(input(xidx_), input(yidx_)), nullptr, nullptr,
                         &signed_squared_distance);
  if (!oriented_same_as_polyline_) signed_squared_distance *= -1.0;

  return sgn(signed_squared_distance) *
             std::sqrt(std::abs(signed_squared_distance)) -
         nominal_;
}

void Polyline2SignedDistanceCost::Quadraticize(const VectorXf& input,
                                               MatrixXf* hess,
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
  float signed_squared_distance;
  LineSegment2 segment(Point2(0.0, 0.0), Point2(1.0, 1.0));
  const Point2 closest_point = polyline_.ClosestPoint(
      current_position, &is_vertex, &segment, &signed_squared_distance);
  if (!oriented_same_as_polyline_) signed_squared_distance *= -1.0;

  const float sign = sgn(signed_squared_distance);
  const float distance = std::sqrt(std::abs(signed_squared_distance));
  const float delta_x = current_position.x() - closest_point.x();
  const float delta_y = current_position.y() - closest_point.y();

  // Handle cases separately depending on whether or not closest point is
  // a vertex of the polyline.
  float dx = sign * delta_x / distance;
  float dy = sign * delta_y / distance;

  const float denom = signed_squared_distance * distance;
  float ddx = delta_y * delta_y / denom;
  float ddy = delta_x * delta_x / denom;
  float dxdy = -delta_x * delta_y / denom;

  if (!is_vertex) {
    const Point2& unit_segment = segment.UnitDirection();

    dx = unit_segment.y();
    dy = -unit_segment.x();
    ddx = 0.0;
    ddy = 0.0;
    dxdy = 0.0;
  }

  (*grad)(xidx_) += dx;
  (*grad)(yidx_) += dy;

  (*hess)(xidx_, xidx_) += ddx;
  (*hess)(yidx_, yidx_) += ddy;
  (*hess)(xidx_, yidx_) += dxdy;
  (*hess)(yidx_, xidx_) += dxdy;
}

}  // namespace e2e_noa::planning
