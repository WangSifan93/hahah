#ifndef ST_PLANNING_UTIL_QTFM_SEGMENT_MATCHER_V2
#define ST_PLANNING_UTIL_QTFM_SEGMENT_MATCHER_V2

#include <sys/types.h>

#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/geometry/aabox2d.h"
#include "math/geometry/grid_frame.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"
#include "util/quad_tree_field_map.h"

namespace e2e_noa::planning {

class QtfmSegmentMatcherV2 {
 public:
  static constexpr double kMinSegmentLength = 1e-5;
  static constexpr double kLengthEpsilon = 1e-6;
  static constexpr double kMaxSameDirCrossProdError = 1e-6;
  struct Config {
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;

    double resolution = 0.0;

    int qtfm_depth = 1;

    int cutoff_num_range = 1;
    int cutoff_max_num_candidate = 16;

    double cutoff_distance = 0.0;

    bool IsValid() const {
      return x_max > x_min && y_max > y_min && resolution > 0.0 &&
             qtfm_depth > 0 && cutoff_num_range > 0 &&
             cutoff_max_num_candidate > 0 && cutoff_distance > 0.0;
    }

    std::string DebugStringFullPrecision() const;
  };

  QtfmSegmentMatcherV2(Config config, std::vector<Segment2d> raw_segments,
                       std::vector<int> raw_start_segment_ids);

  bool GetNearestSegmentIndex(double x, double y, int* index) const;

  QtfmSegmentMatcherV2(const QtfmSegmentMatcherV2& other);
  QtfmSegmentMatcherV2& operator=(const QtfmSegmentMatcherV2& other);

  const Config& config() const { return config_; }

  std::string DebugStringFullPrecision() const;

 private:
  static constexpr double kCutoffComplexity = 2.0;
  static constexpr double kCutoff = 1.0;
  static constexpr double kKeepDivide = 3.0;
  using Range = std::pair<int, int>;
  using Bool = u_char;

  struct IdField {
    int range_id_begin;
    int range_id_end;

    QtfmSegmentMatcherV2* matcher_ptr = nullptr;
    bool is_simple_enough = false;

    double Complexity() const {
      return is_simple_enough ? kCutoff : kKeepDivide;
    }

    void SuperPose(const IdField& other) {
      matcher_ptr->SuperposeField(other, this);
      is_simple_enough = matcher_ptr->IsFieldSimpleEnough(*this);
    }

    void Simplify(const AABox2d& box) {
      matcher_ptr->StitchField(box, this);
      matcher_ptr->PurgeField(box, this);
      matcher_ptr->ClampField(box, this);
      is_simple_enough = matcher_ptr->IsFieldSimpleEnough(*this);
    }

    constexpr bool IsVoid() const { return size() == 0; }

    constexpr int size() const {
      return std::max(0, range_id_end - range_id_begin);
    }
  };

  bool IsFieldValid(const IdField& field) const {
    return field.range_id_begin >= 0 &&
           field.range_id_end <= static_cast<int>(all_ranges_.size());
  }

  std::string FieldDebugString(const IdField& field) const {
    if (field.IsVoid()) {
      return "";
    }
    std::string res = "ranges:";
    for (int rid = field.range_id_begin; rid < field.range_id_end; ++rid) {
      res += std::to_string(all_ranges_[rid].first);
      res += "~";
      res += std::to_string(all_ranges_[rid].second);
      res += ",";
    }
    return res;
  }

  void StitchField(const AABox2d& box, IdField* field);

  void PurgeField(const AABox2d& box, IdField* field);

  void ClampField(const AABox2d& box, IdField* field);

  void DownClampRange(const std::array<Vec2d, 4>& box_points,
                      Range* range) const;

  void UpClampRange(const std::array<Vec2d, 4>& box_points, Range* range) const;

  constexpr int GetNearestSegmentIndexFromRange(const Vec2d& point,
                                                const Range& range) const;

  void SuperposeField(const IdField& other, IdField* field);

  bool IsFieldSimpleEnough(const IdField& field) const;

  IdField CreateIdFieldFromSortedVector(
      const std::vector<int>& sorted_segment_ids);

  constexpr int GridIndexToVectorIndex(int xid, int yid) const {
    if (xid < 0 || xid >= x_dim_ || yid < 0 || yid >= y_dim_) {
      return -1;
    }
    return xid * y_dim_ + yid;
  }

  void CalculateGridsAffectedBySegment(
      int segment_id, std::vector<std::pair<int, int>>* result);

  static bool IsDivisible(const Segment2d& prev_seg, const Segment2d& cur_seg);

  static Segment2d GetDivisionLine(const Segment2d& prev_seg,
                                   const Segment2d& cur_seg);

  static void MakeSegmentsDivisible(
      const std::vector<Segment2d>& raw_segments,
      const std::vector<int>& raw_start_segment_ids,
      std::vector<Segment2d>* divisible_segments,
      std::vector<int>* divisible_start_segment_ids,
      std::vector<int>* raw_segment_index_of_divisible_segments);

  using SegmentMap = QuadTreeFieldMap<IdField>;

  Config config_;
  std::vector<Segment2d> raw_segments_;
  std::vector<int> raw_start_segment_ids_;

  std::vector<Segment2d> segments_;
  std::vector<Bool> is_start_segment_;
  std::vector<int> segment_id_to_raw_id_;

  std::vector<Segment2d> division_line_with_prev_;

  std::vector<std::pair<int, int>> all_ranges_;

  int x_dim_ = 0;
  int y_dim_ = 0;
  std::optional<GridFrame2d> grid_frame_;

  std::vector<int> grid_;
  std::vector<SegmentMap> segment_field_maps_;

  std::vector<double> center_distance_sqr_cache_;
  std::vector<int> single_object_grid_cache_;
};

constexpr int QtfmSegmentMatcherV2::GetNearestSegmentIndexFromRange(
    const Vec2d& point, const Range& range) const {
  DCHECK(range.second > range.first);
  DCHECK(range.first >= 0);

  if (range.first == range.second - 1) {
    return range.first;
  }

  int left = range.first + 1;
  int right = range.second - 1;

  if (division_line_with_prev_[right].ProductOntoUnit(point) < 0.0) {
    return right;
  }
  if (left == right ||
      division_line_with_prev_[left].ProductOntoUnit(point) >= 0.0) {
    return left - 1;
  }

  while (true) {
    if (left >= right - 1) {
      return left;
    }

    const int mid = (left + right) / 2;
    if (division_line_with_prev_[mid].ProductOntoUnit(point) >= 0.0) {
      right = mid;
    } else {
      left = mid;
    }
  }
  return left;
}

}  // namespace e2e_noa::planning

#endif
