#ifndef ST_PLANNING_MATH_FRENET_FRAME
#define ST_PLANNING_MATH_FRENET_FRAME

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "math/frenet_common.h"
#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/segment_matcher/segment_matcher_kdtree.h"
#include "math/vec.h"
#include "util/qtfm_segment_matcher_v2.h"

namespace e2e_noa {

enum class FrenetFrameType {
  kBruteFroce = 0,
  kKdTree = 1,
  kQtfmKdTree = 2,
};

class FrenetFrame {
 public:
  const std::vector<Vec2d>& points() const { return points_; }

  const std::vector<Vec2d>& tangents() const { return tangents_; }

  const std::vector<int>& raw_indices() const { return raw_indices_; }

  const std::vector<double>& s_knots() const { return s_knots_; }

  double start_s() const { return s_knots_.front(); }

  double end_s() const { return s_knots_.back(); }

  double length() const { return s_knots_.back() - s_knots_.front(); }

  Vec2d InterpolateTangentByS(double s) const;

  Vec2d InterpolateTangentByXY(const Vec2d& xy) const;

  Vec2d SLToXY(const FrenetCoordinate& sl) const;

  FrenetCoordinate XYToSL(const Vec2d& xy) const;

  absl::StatusOr<FrenetCoordinate> XYToSLWithHeadingDiffLimit(
      const Vec2d& xy, double heading, double max_heading_diff) const;

  void XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal) const;

  void XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal, int* index,
              double* alpha) const;

  void XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
              std::pair<int, int>* raw_index_pair, double* alpha) const;

  Vec2d FindAABBNearestPoint(const Polygon2d& polygon,
                             bool get_max_s = false) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxWithHeading(
      const Box2d& box, double max_heading_diff = M_PI_2) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAt(const Box2d& box) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContour(
      const Polygon2d& contour) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtPoints(
      absl::Span<const Vec2d> points) const;

  virtual ~FrenetFrame() = default;

  FrenetFrame(std::vector<Vec2d> points, std::vector<double> s_knots,
              std::vector<double> segment_len_inv, std::vector<Vec2d> tangents,
              std::vector<int> raw_indices)
      : points_(std::move(points)),
        s_knots_(std::move(s_knots)),
        segment_len_inv_(std::move(segment_len_inv)),
        tangents_(std::move(tangents)),
        raw_indices_(std::move(raw_indices)) {}

 protected:
  virtual absl::StatusOr<FrenetCoordinate> XYToSLWithHeadingDiffLimitImplement(
      const Vec2d& xy, double heading, double max_heading_diff) const;

  virtual void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl,
                               Vec2d* normal, int* index,
                               double* alpha) const = 0;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtPointsWithHeading(
      absl::Span<const Vec2d> points, double heading,
      double max_heading_diff) const;

  std::tuple<Vec2d, Vec2d, double> GetInterpolationRange(double s) const;

 protected:
  std::vector<Vec2d> points_;
  std::vector<double> s_knots_;
  std::vector<double> segment_len_inv_;
  std::vector<Vec2d> tangents_;
  std::vector<int> raw_indices_;
};

class BruteForceFrenetCoordinate : public FrenetFrame {
 public:
  BruteForceFrenetCoordinate(std::vector<Vec2d> points,
                             std::vector<double> s_knots,
                             std::vector<double> segment_len_inv,
                             std::vector<Vec2d> tangents,
                             std::vector<int> raw_indices)
      : FrenetFrame(std::move(points), std::move(s_knots),
                    std::move(segment_len_inv), std::move(tangents),
                    std::move(raw_indices)) {}

  ~BruteForceFrenetCoordinate() = default;

 private:
  void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                       int* index, double* alpha) const override;
};

class KdTreeFrenetCoordinate : public FrenetFrame {
 public:
  KdTreeFrenetCoordinate(std::vector<Vec2d> points, std::vector<double> s_knots,
                         std::vector<double> segment_len_inv,
                         std::vector<Vec2d> tangents,
                         std::vector<int> raw_indices,
                         std::vector<Segment2d> segments,
                         std::shared_ptr<KdtreeSegmentMatcher> segment_matcher)
      : FrenetFrame(std::move(points), std::move(s_knots),
                    std::move(segment_len_inv), std::move(tangents),
                    std::move(raw_indices)),
        segments_(std::move(segments)),
        segment_matcher_(std::move(segment_matcher)) {}

  ~KdTreeFrenetCoordinate() = default;

 protected:
  absl::StatusOr<FrenetCoordinate> XYToSLWithHeadingDiffLimitImplement(
      const Vec2d& xy, double heading, double max_heading_diff) const override;

  void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                       int* index, double* alpha) const override;

 protected:
  std::vector<Segment2d> segments_;
  std::shared_ptr<KdtreeSegmentMatcher> segment_matcher_;
};

class QtfmEnhancedKdTreeFrenetCoordinate : public KdTreeFrenetCoordinate {
 public:
  QtfmEnhancedKdTreeFrenetCoordinate(
      std::vector<Vec2d> points, std::vector<double> s_knots,
      std::vector<double> segment_len_inv, std::vector<Vec2d> tangents,
      std::vector<int> raw_indices, std::vector<Segment2d> segments,
      std::shared_ptr<KdtreeSegmentMatcher> segment_matcher,
      std::shared_ptr<planning::QtfmSegmentMatcherV2> qtfm_segment_matcher)
      : KdTreeFrenetCoordinate(std::move(points), std::move(s_knots),
                               std::move(segment_len_inv), std::move(tangents),
                               std::move(raw_indices), std::move(segments),
                               std::move(segment_matcher)),
        qtfm_segment_matcher_(std::move(qtfm_segment_matcher)) {}

  ~QtfmEnhancedKdTreeFrenetCoordinate() = default;

 private:
  void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                       int* index, double* alpha) const override;

 private:
  std::shared_ptr<planning::QtfmSegmentMatcherV2> qtfm_segment_matcher_;
};

absl::StatusOr<BruteForceFrenetCoordinate> BuildBruteForceFrenetFrame(
    absl::Span<const Vec2d> raw_points, bool down_sample_raw_points);

absl::StatusOr<KdTreeFrenetCoordinate> BuildKdTreeFrenetFrame(
    absl::Span<const Vec2d> raw_points, bool down_sample_raw_points);

absl::StatusOr<QtfmEnhancedKdTreeFrenetCoordinate>
BuildQtfmEnhancedKdTreeFrenetCoordinate(absl::Span<const Vec2d> raw_points,
                                        bool down_sample_raw_points);

}  // namespace e2e_noa

#endif
