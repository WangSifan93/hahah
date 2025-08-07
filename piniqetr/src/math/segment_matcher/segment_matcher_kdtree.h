#ifndef MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE
#define MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE

#include <stdint.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "math/geometry/aabox2d.h"
#include "math/geometry/aabox_kdtree2d.h"
#include "math/geometry/segment2d.h"
#include "math/segment_matcher/aabox_info.h"
#include "math/segment_matcher/segment_matcher.h"
#include "math/vec.h"

namespace e2e_noa {

class KdtreeSegmentMatcher : public SegmentMatcher {
 public:
  explicit KdtreeSegmentMatcher(const std::vector<Vec2d>& points);

  explicit KdtreeSegmentMatcher(const std::vector<Segment2d>& segments);

  explicit KdtreeSegmentMatcher(
      const std::vector<std::pair<std::string, Segment2d>>& named_segments);

  bool GetNearestSegmentId(double x, double y, std::string* id) const override;

  std::vector<std::string> GetSegmentIdInRadius(double x, double y,
                                                double r) const override;

  bool GetNearestSegmentIndex(double x, double y, int* index) const override;

  std::vector<int> GetSegmentIndexInRadius(double x, double y,
                                           double r) const override;

  const Segment2d* GetNearestSegment(double x, double y) const override;

  std::vector<const Segment2d*> GetSegmentInRadius(double x, double y,
                                                   double r) const override;

  bool GetNearestNamedSegment(double x, double y, Segment2d* seg,
                              std::string* id) const override;

  std::vector<std::pair<const Segment2d*, std::string>>
  GetNamedSegmentsInRadius(double x, double y, double r) const override;

  KdtreeSegmentMatcher(const KdtreeSegmentMatcher&) = delete;

  std::vector<std::string> GetSegmentIdInAABox(const AABox2d& aabox) const;

  std::vector<int> GetSegmentIndexInAABox(const AABox2d& aabox) const;

  std::vector<const Segment2d*> GetSegmentInAABox(const AABox2d& aabox) const;

  std::vector<std::pair<const Segment2d*, std::string>> GetNamedSegmentsInAABox(
      const AABox2d& aabox) const;

 private:
  AABoxKDTree2d<AABoxInfo> segments_tree_;
};

}  // namespace e2e_noa

#endif
