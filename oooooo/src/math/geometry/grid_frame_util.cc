#include "math/geometry/grid_frame_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/geometry/segment2d.h"
#include "math/util.h"

namespace e2e_noa {

namespace {
inline int NextId(int id, int num_segments) {
  DCHECK_GE(id, 0);
  DCHECK_LT(id, num_segments);
  return id >= num_segments - 1 ? 0 : id + 1;
}

inline int PrevIdentifier(int id, int num_segments) {
  DCHECK_GE(id, 0);
  DCHECK_LT(id, num_segments);
  return id == 0 ? num_segments - 1 : id - 1;
}
}  // namespace

YMonotonicGridSet2di CalculateGridsTouchingConvexPolygon(
    const GridFrame2d& grid_frame, const Polygon2d& polygon) {
  CHECK(polygon.is_convex());
  return ComputeGridsTouchingConvexPolygonPoints(grid_frame, polygon.points());
}

YMonotonicGridSet2di ComputeGridsTouchingConvexPolygonPoints(
    const GridFrame2d& grid_frame, absl::Span<const Vec2d> points) {
  CHECK_GT(points.size(), 2);

  const int num_segments = static_cast<int>(points.size());
  std::vector<Segment2d> segments;
  segments.reserve(num_segments);
  for (int i = 0; i < num_segments - 1; ++i) {
    segments.emplace_back(points[i], points[i + 1]);
  }
  segments.emplace_back(points[points.size() - 1], points[0]);

  int segment_with_lowest_start_point_id = -1;
  int segment_with_highest_start_point_id = -1;

  double y_min = +std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_segments; ++i) {
    const double start_y = segments[i].start().y();
    if (i == 0) {
      y_min = start_y;
      y_max = start_y;
      segment_with_highest_start_point_id = i;
      segment_with_lowest_start_point_id = i;
    } else {
      if (start_y > y_max) {
        segment_with_highest_start_point_id = i;
        y_max = start_y;
      } else if (start_y < y_min) {
        segment_with_lowest_start_point_id = i;
        y_min = start_y;
      }
    }
  }
  CHECK_GT(y_max, y_min);

  const int yid_min = grid_frame.YToGridYId<int>(y_min);
  const int yid_max = grid_frame.YToGridYId<int>(y_max);
  CHECK_GE(yid_max, yid_min);

  const int left_segment_begin =
      PrevIdentifier(segment_with_lowest_start_point_id, num_segments);
  const int left_segment_end =
      PrevIdentifier(segment_with_highest_start_point_id, num_segments);
  const int right_segment_begin = segment_with_lowest_start_point_id;
  const int right_segment_end = segment_with_highest_start_point_id;

  int left = left_segment_begin;

  int right = right_segment_begin;

  std::vector<std::pair<int, int>> x_begin_end;
  x_begin_end.reserve(yid_max - yid_min + 1);
  for (int yid = yid_min; yid <= yid_max; ++yid) {
    const double current_y = grid_frame.YIdToY(yid);
    const double next_y = grid_frame.YIdToY(yid + 1);

    double x_max = -std::numeric_limits<double>::infinity();
    double x_min = +std::numeric_limits<double>::infinity();

    for (int i = left; i != left_segment_end;
         i = PrevIdentifier(i, num_segments)) {
      if (segments[i].min_y() > next_y) {
        break;
      }
      if (segments[i].max_y() < current_y) {
        continue;
      }
      Segment2d clamped_segment = segments[i];
      clamped_segment.ClampByYMin(current_y);
      clamped_segment.ClampByYMax(next_y);
      x_min = std::min(clamped_segment.min_x(), x_min);
      x_max = std::max(clamped_segment.max_x(), x_max);
    }

    for (int i = right; i != right_segment_end; i = NextId(i, num_segments)) {
      if (segments[i].min_y() > next_y) {
        break;
      }
      if (segments[i].max_y() < current_y) {
        continue;
      }
      Segment2d clamped_segment = segments[i];
      clamped_segment.ClampByYMin(current_y);
      clamped_segment.ClampByYMax(next_y);
      x_min = std::min(clamped_segment.min_x(), x_min);
      x_max = std::max(clamped_segment.max_x(), x_max);
    }

    int next_left = left_segment_end;
    for (int i = left; i != left_segment_end;
         i = PrevIdentifier(i, num_segments)) {
      const Segment2d& segment = segments[i];
      if (segment.max_y() >= next_y) {
        next_left = i;
        break;
      }
    }

    int next_right = right_segment_end;
    for (int i = right; i != right_segment_end; i = NextId(i, num_segments)) {
      const Segment2d& segment = segments[i];
      if (segment.max_y() >= next_y) {
        next_right = i;
        break;
      }
    }
    left = next_left;
    right = next_right;

    if (x_min <= x_max) {
      x_begin_end.emplace_back(grid_frame.XToGridXId<int>(x_min),
                               grid_frame.XToGridXId<int>(x_max) + 1);
    } else {
      x_begin_end.emplace_back(0, 0);
    }
  }

  return YMonotonicGridSet2di(yid_min, yid_max + 1, std::move(x_begin_end));
}

YMonotonicGridSet2di CalculateGridsTouchingCircle(const GridFrame2d& grid_frame,
                                                  const Circle2d& circle) {
  const double center_x = circle.center().x();
  const double center_y = circle.center().y();
  const double radius = circle.radius();
  const double radius_sqr = Sqr(radius);
  const double y_max = center_y + radius;
  const double y_min = center_y - radius;

  CHECK_GT(y_max, y_min);
  CHECK_GE(radius, 0.0);

  const int yid_min = grid_frame.YToGridYId<int>(y_min);
  const int yid_max = grid_frame.YToGridYId<int>(y_max);
  CHECK_GE(yid_max, yid_min);

  std::vector<std::pair<int, int>> x_begin_end;
  x_begin_end.reserve(yid_max - yid_min + 1);
  for (int yid = yid_min; yid <= yid_max; ++yid) {
    const double current_y = grid_frame.YIdToY(yid);
    const double next_y = grid_frame.YIdToY(yid + 1);

    double x_max = -std::numeric_limits<double>::infinity();
    double x_min = +std::numeric_limits<double>::infinity();

    if (next_y >= center_y && current_y <= center_y) {
      x_min = center_x - radius;
      x_max = center_x + radius;
    } else {
      if (current_y > center_y) {
        const double y_offset = current_y - center_y;
        const double half_chord =
            std::sqrt(std::max(radius_sqr - Sqr(y_offset), 0.0));
        x_min = center_x - half_chord;
        x_max = center_x + half_chord;
      } else {
        const double y_offset = next_y - center_y;
        const double half_chord =
            std::sqrt(std::max(radius_sqr - Sqr(y_offset), 0.0));

        x_min = center_x - half_chord;
        x_max = center_x + half_chord;
      }
    }

    CHECK_LE(x_min, x_max);
    x_begin_end.emplace_back(grid_frame.XToGridXId<int>(x_min),
                             grid_frame.XToGridXId<int>(x_max) + 1);
  }

  return YMonotonicGridSet2di(yid_min, yid_max + 1, std::move(x_begin_end));
}

}  // namespace e2e_noa
