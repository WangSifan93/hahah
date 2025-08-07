#include "math/segment_matcher/segment_matcher_kdtree.h"

#include <algorithm>
#include <ostream>

#include "absl/types/span.h"

namespace e2e_noa {
namespace {

bool IsSegmentInAABox(const AABoxInfo* aabox_info, const AABox2d& aabox) {
  return aabox.HasOverlap(aabox_info->segment());
}

AABoxKDTreeParams GetDefaultAABoxKDTreeParams() {
  return AABoxKDTreeParams{.max_leaf_size = 4};
}

}  // namespace

KdtreeSegmentMatcher::KdtreeSegmentMatcher(const std::vector<Vec2d>& points)
    : SegmentMatcher(points),
      segments_tree_(aa_boxes_info_, GetDefaultAABoxKDTreeParams()) {}

KdtreeSegmentMatcher::KdtreeSegmentMatcher(
    const std::vector<Segment2d>& segments)
    : SegmentMatcher(segments),
      segments_tree_(aa_boxes_info_, GetDefaultAABoxKDTreeParams()) {}

KdtreeSegmentMatcher::KdtreeSegmentMatcher(
    const std::vector<std::pair<std::string, Segment2d>>& named_segments)
    : SegmentMatcher(named_segments),
      segments_tree_(aa_boxes_info_, GetDefaultAABoxKDTreeParams()) {}

bool KdtreeSegmentMatcher::GetNearestSegmentId(double x, double y,
                                               std::string* id) const {
  CHECK(named_flag_) << " construct without id";
  CHECK_NOTNULL(id);
  const auto& object = segments_tree_.GetNearestObject({x, y});
  if (object == nullptr) {
    return false;
  }
  const auto idx = object->index();
  if (idx >= 0 && idx < segment_names_.size()) {
    *id = *segment_names_[idx];
    return true;
  }
  return false;
}

std::vector<std::string> KdtreeSegmentMatcher::GetSegmentIdInRadius(
    double x, double y, double r) const {
  CHECK(named_flag_) << " construct without id";
  const auto& aabox_in_radius = segments_tree_.GetObjects({x, y}, r);
  std::vector<std::string> names;
  names.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    const auto idx = aabox->index();
    if (idx >= 0 && idx < segment_names_.size()) {
      names.push_back(*segment_names_[idx]);
    }
  }
  return names;
}

bool KdtreeSegmentMatcher::GetNearestSegmentIndex(double x, double y,
                                                  int* index) const {
  CHECK(index_flag_) << " construct without index";
  CHECK_NOTNULL(index);
  const auto& object = segments_tree_.GetNearestObject({x, y});
  if (object == nullptr) {
    return false;
  }
  *index = object->index();
  return true;
}

std::vector<int> KdtreeSegmentMatcher::GetSegmentIndexInRadius(double x,
                                                               double y,
                                                               double r) const {
  CHECK(index_flag_) << " construct without index";
  const auto& aabox_in_radius = segments_tree_.GetObjects({x, y}, r);
  std::vector<int> indexes;
  indexes.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    indexes.emplace_back(aabox->index());
  }
  return indexes;
}

const Segment2d* KdtreeSegmentMatcher::GetNearestSegment(double x,
                                                         double y) const {
  const auto& object = segments_tree_.GetNearestObject({x, y});
  if (object == nullptr) {
    return nullptr;
  }
  return &object->segment();
}

std::vector<const Segment2d*> KdtreeSegmentMatcher::GetSegmentInRadius(
    double x, double y, double r) const {
  const auto& aabox_in_radius = segments_tree_.GetObjects({x, y}, r);
  std::vector<const Segment2d*> segments_ptr;
  segments_ptr.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    segments_ptr.emplace_back(&aabox->segment());
  }
  return segments_ptr;
}

bool KdtreeSegmentMatcher::GetNearestNamedSegment(double x, double y,
                                                  e2e_noa::Segment2d* seg,
                                                  std::string* id) const {
  CHECK(named_flag_) << " construct without id";
  CHECK_NOTNULL(seg);
  CHECK_NOTNULL(id);
  const auto& object = segments_tree_.GetNearestObject({x, y});
  if (object == nullptr) {
    return false;
  }

  const auto idx = object->index();
  if (idx >= 0 && idx < segment_names_.size()) {
    *seg = object->segment();
    *id = *segment_names_[idx];
    return true;
  }

  return false;
}

std::vector<std::pair<const Segment2d*, std::string>>
KdtreeSegmentMatcher::GetNamedSegmentsInRadius(double x, double y,
                                               double r) const {
  CHECK(named_flag_) << " construct without id";
  const auto& aabox_in_radius = segments_tree_.GetObjects({x, y}, r);
  std::vector<std::pair<const Segment2d*, std::string>> named_segments;
  named_segments.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    const auto idx = aabox->index();
    if (idx >= 0 && idx < segment_names_.size()) {
      named_segments.emplace_back(&aabox->segment(), *segment_names_[idx]);
    }
  }
  return named_segments;
}

std::vector<std::string> KdtreeSegmentMatcher::GetSegmentIdInAABox(
    const AABox2d& aabox) const {
  CHECK(named_flag_) << " construct without id";
  const auto& aabox_results =
      segments_tree_.GetObjectsWithinAABox(aabox, IsSegmentInAABox);
  std::vector<std::string> names;
  names.reserve(aabox_results.size());
  for (const auto& aabox : aabox_results) {
    const auto idx = aabox->index();
    if (idx >= 0 && idx < segment_names_.size()) {
      names.push_back(*segment_names_[idx]);
    }
  }
  return names;
}

std::vector<int> KdtreeSegmentMatcher::GetSegmentIndexInAABox(
    const AABox2d& aabox) const {
  CHECK(named_flag_) << " construct without index";
  const auto& aabox_results =
      segments_tree_.GetObjectsWithinAABox(aabox, IsSegmentInAABox);
  std::vector<int> indexes;
  indexes.reserve(aabox_results.size());
  for (const auto& aabox : aabox_results) {
    indexes.push_back(aabox->index());
  }
  return indexes;
}

std::vector<const Segment2d*> KdtreeSegmentMatcher::GetSegmentInAABox(
    const AABox2d& aabox) const {
  const auto& aabox_results =
      segments_tree_.GetObjectsWithinAABox(aabox, IsSegmentInAABox);
  std::vector<const Segment2d*> segments_ptr;
  segments_ptr.reserve(aabox_results.size());
  for (const auto& aabox : aabox_results) {
    segments_ptr.push_back(&aabox->segment());
  }
  return segments_ptr;
}

std::vector<std::pair<const Segment2d*, std::string>>
KdtreeSegmentMatcher::GetNamedSegmentsInAABox(const AABox2d& aabox) const {
  CHECK(named_flag_) << " construct without id";
  const auto& aabox_results =
      segments_tree_.GetObjectsWithinAABox(aabox, IsSegmentInAABox);
  std::vector<std::pair<const Segment2d*, std::string>> named_segments;
  named_segments.reserve(aabox_results.size());
  for (const auto& aabox : aabox_results) {
    const auto idx = aabox->index();
    if (idx >= 0 && idx < segment_names_.size()) {
      named_segments.emplace_back(&aabox->segment(), *segment_names_[idx]);
    }
  }
  return named_segments;
}

}  // namespace e2e_noa
