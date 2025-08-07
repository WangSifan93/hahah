#ifndef MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_
#define MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "math/geometry/segment2d.h"
#include "math/segment_matcher/aabox_info.h"
#include "math/vec.h"

namespace e2e_noa {

class SegmentMatcher {
 public:
  explicit SegmentMatcher(absl::Span<const Vec2d> points);

  explicit SegmentMatcher(const std::vector<Segment2d>& segments);

  explicit SegmentMatcher(
      absl::Span<const std::pair<std::string, Segment2d>> named_segments);

  virtual ~SegmentMatcher() = default;

  SegmentMatcher(const SegmentMatcher&) = delete;

  const Segment2d* GetSegmentByIndex(int index) const;

  const Segment2d* GetSegmentById(const std::string& id) const;

  const std::vector<const Segment2d*>& segments() const { return segments_; }

  bool GetProjection(double x, double y, bool is_clamp, double* accumulated_s,
                     double* lateral) const;

  bool GetProjection(double x, double y, Vec2d* nearest_point = nullptr,
                     double* accumulated_s = nullptr,
                     double* min_dist = nullptr,
                     Segment2d* segment = nullptr) const;

  bool GetNearestSegmentIndexWithHeading(double x, double y, double theta,
                                         double max_radius,
                                         double max_heading_diff,
                                         int* nearest_index) const;

  bool GetNearestSegmentIdWithHeading(double x, double y, double theta,
                                      double max_radius,
                                      double max_heading_diff,
                                      std::string* nearest_id) const;

  std::vector<std::string> GetSegmentIdInRadiusWithHeading(
      double x, double y, double theta, double max_radius,
      double max_heading_diff) const;

  std::vector<int> GetSegmentIndexInRadiusWithHeading(
      double x, double y, double theta, double max_radius,
      double max_heading_diff) const;

  virtual bool GetNearestSegmentId(double x, double y,
                                   std::string* id) const = 0;

  virtual std::vector<std::string> GetSegmentIdInRadius(double x, double y,
                                                        double r) const = 0;

  virtual bool GetNearestSegmentIndex(double x, double y, int* index) const = 0;

  virtual std::vector<int> GetSegmentIndexInRadius(double x, double y,
                                                   double r) const = 0;

  virtual const Segment2d* GetNearestSegment(double x, double y) const = 0;

  virtual std::vector<const Segment2d*> GetSegmentInRadius(double x, double y,
                                                           double r) const = 0;

  virtual bool GetNearestNamedSegment(double x, double y, Segment2d* seg,
                                      std::string* id) const = 0;

  virtual std::vector<std::pair<const Segment2d*, std::string>>
  GetNamedSegmentsInRadius(double x, double y, double radius) const = 0;

 protected:
  std::vector<AABoxInfo> aa_boxes_info_;
  absl::flat_hash_map<std::string, const Segment2d*> segment_map_;
  std::vector<const std::string*> segment_names_;
  std::vector<const Segment2d*> segments_;
  bool index_flag_ = false;
  bool named_flag_ = false;
};

}  // namespace e2e_noa

#endif
