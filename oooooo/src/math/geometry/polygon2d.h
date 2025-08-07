#ifndef MATH_GEOMETRY_POLYGON2D_H_
#define MATH_GEOMETRY_POLYGON2D_H_

#include <algorithm>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "math/geometry/aabox2d.h"
#include "math/geometry/box2d.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"

namespace e2e_noa {

class Polygon2d {
 public:
  Polygon2d() = default;

  explicit Polygon2d(const Box2d& box);

  explicit Polygon2d(std::vector<Vec2d> points);

  Polygon2d(std::vector<Vec2d> points, bool is_convex);

  Polygon2d(std::vector<Vec2d> points, int force_convex);

  void SetPoints(const std::vector<Vec2d>& points);

  const std::vector<Vec2d>& points() const { return points_; }

  const std::vector<Segment2d>& line_segments() const { return line_segments_; }

  int num_points() const { return num_points_; }

  bool is_convex() const { return is_convex_; }

  bool IsSelfIntersecting() const;

  double area() const { return area_; }

  Vec2d centroid() const {
    return std::accumulate(points_.begin(), points_.end(), Vec2d(0.0, 0.0)) /
           points_.size();
  }

  double DistanceToBoundary(const Vec2d& point) const;

  double DistanceTo(const Vec2d& point) const;

  double DistanceTo(const Segment2d& line_segment) const;

  double DistanceTo(const Box2d& box) const;

  double DistanceTo(const Polygon2d& polygon) const;

  double DistanceSquareTo(const Vec2d& point) const;

  bool IsPointIn(const Vec2d& point) const;

  bool IsPointOnBoundary(const Vec2d& point) const;

  bool Contains(const Segment2d& line_segment) const;

  bool Contains(const Polygon2d& polygon) const;

  static bool ComputeConvexHull(absl::Span<const Vec2d> points,
                                Polygon2d* const polygon);

  static std::optional<std::vector<Vec2d>> ComputeConvexHullPoints(
      absl::Span<const Vec2d> points);

  static Polygon2d MergeTwoBoxes(const Box2d& first_box,
                                 const Box2d& second_box);

  static Polygon2d MergeBoxes(const std::vector<Box2d>& boxes);

  static Polygon2d MergeTwoPolygons(const Polygon2d& first_polygon,
                                    const Polygon2d& second_polygon);

  static bool AreConvexHullPoints(absl::Span<const Vec2d> points);

  static bool AreNonDegeneratedConvexHullPoints(absl::Span<const Vec2d> points);

  bool HasOverlap(const Segment2d& line_segment) const;

  bool HasOverlapWithBuffer(const Box2d& box, double lat_buffer,
                            double lon_buffer) const;

  bool HasOverlap(const Box2d& box) const {
    return HasOverlapWithBuffer(box, 0.0, 0.0);
  }

  bool GetOverlap(const Segment2d& line_segment, Vec2d* const first,
                  Vec2d* const last) const;

  void GetAllVertices(std::vector<Vec2d>* const vertices) const {
    *vertices = points_;
  }

  const std::vector<Vec2d>& GetAllVertices() const { return points_; }

  std::vector<Segment2d> GetAllOverlaps(const Segment2d& line_segment) const;

  bool HasOverlap(const Polygon2d& polygon) const;

  bool GetMinPenetrationDistance(const Polygon2d& polygon,
                                 double* min_penetration, Vec2d* dir_vec) const;

  bool GetMinPenetrationDistance(const Polygon2d& polygon,
                                 double* min_penetration) const;

  bool GetPenetrationDistanceAlongDir(const Polygon2d& polygon,
                                      const Vec2d& dir_vec,
                                      double* penetration) const;

  bool ComputeOverlap(const Polygon2d& other_polygon,
                      Polygon2d* const overlap_polygon) const;

  double CircleRadius() const { return aabox_.half_diagonal(); }

  const Vec2d& CircleCenter() const { return aabox_.center(); }

  const AABox2d& AABoundingBox() const { return aabox_; }

  [[nodiscard]] Box2d BoundingBoxWithHeading(const double heading) const;

  [[nodiscard]] Box2d MinAreaBoundingBox() const;

  int ExtremePoint(const Vec2d& direction_vec) const;

  int ExtremePointBruteForce(const Vec2d& direction_vec) const;

  int ConvexExtremePointBinarySearch(const Vec2d& direction_vec) const;

  void ExtremePoints(const double heading, Vec2d* const first,
                     Vec2d* const last) const;

  void ExtremePoints(const Vec2d& direction_vec, Vec2d* first,
                     Vec2d* last) const;

  void ExtremePoints(const Vec2d& direction_vec, int* first, int* last) const;

  void ExtremePoints(const Vec2d& direction_vec, int* first_idx, int* last_idx,
                     Vec2d* first_pt, Vec2d* last_pt) const;

  [[nodiscard]] Polygon2d ExpandByDistance(const double distance) const;

  [[nodiscard]] Polygon2d ExtrudeAlongVector(const Vec2d& vec) const;

  [[nodiscard]] Polygon2d Transform(const Vec2d& center, double cos_angle,
                                    double sin_angle,
                                    const Vec2d& translation) const;

  [[nodiscard]] Polygon2d AffineTransform(const Vec2d& center, double cos_angle,
                                          double sin_angle,
                                          const Vec2d& translation) const;

  [[nodiscard]] Polygon2d Shift(const Vec2d& shift_vec) const;

  [[nodiscard]] Polygon2d Rotate(double yaw) const;

  [[nodiscard]] std::optional<Polygon2d> MinkowskiSumWithVec(
      const Vec2d& vec) const;

  std::string DebugString() const;

  std::string DebugStringFullPrecision() const {
    return absl::StrCat(e2e_noa::DebugStringFullPrecision(points_),
                        ", /*is_convex=*/", is_convex());
  }

  template <typename Container>
  static std::optional<Polygon2d> FromPoints(const Container& points,
                                             bool is_convex) {
    if (points.size() < 3) return std::nullopt;
    std::vector<Vec2d> pts;
    pts.reserve(points.size());
    for (const auto& pt : points) {
      pts.emplace_back(pt.x(), pt.y());
    }
    return Polygon2d(std::move(pts), is_convex);
  }

  Segment2d GetPrincipalAxis() const;

  double min_x() const { return aabox_.min_x(); }
  double max_x() const { return aabox_.max_x(); }
  double min_y() const { return aabox_.min_y(); }
  double max_y() const { return aabox_.max_y(); }

  int Next(int at) const;
  int Prev(int at) const;

 protected:
  void BuildFromPoints();
  bool IsPointInSlow(const Vec2d& point) const;
  int ConvexExtremePoint(const Vec2d& direction_vec) const;
  double ConvexDistanceToConvex(const Polygon2d& polygon) const;
  bool ConvexHasOverlapConvex(const Polygon2d& polygon) const;

  static bool ClipConvexHull(const Segment2d& line_segment,
                             std::vector<Vec2d>* const points);

  std::vector<Vec2d> points_;
  int num_points_ = 0;
  std::vector<Segment2d> line_segments_;
  bool is_convex_ = false;
  double area_ = 0.0;
  AABox2d aabox_;
};

inline bool Polygon2d::IsPointOnBoundary(const Vec2d& point) const {
  CHECK_GE(points_.size(), 3);
  return std::any_of(
      line_segments_.begin(), line_segments_.end(),
      [&](const Segment2d& poly_seg) { return poly_seg.IsPointIn(point); });
}

inline bool Polygon2d::IsPointIn(const Vec2d& point) const {
  if (!aabox_.IsPointIn(point)) return false;
  return IsPointInSlow(point);
}

}  // namespace e2e_noa

#endif
