#ifndef MATH_GEOMETRY_GRID_FRAME_H_
#define MATH_GEOMETRY_GRID_FRAME_H_

#include <cmath>
#include <utility>
#include <vector>

namespace e2e_noa {

namespace grid_frame_internal {
template <typename IndexType, typename T>
inline IndexType CoordinateFloorToIndex(T coord, T origin, T unit_inv) {
  return static_cast<IndexType>(std::floor((coord - origin) * unit_inv));
}

template <typename IndexType, typename T>
inline T IndexToCoordinate(IndexType index, T origin, T unit) {
  return unit * index + origin;
}
}  // namespace grid_frame_internal

class GridFrame2d final {
 public:
  GridFrame2d(double unit, double x0, double y0)
      : x0_(x0), y0_(y0), unit_(unit), unit_inv_(1.0 / unit) {
    CHECK_GT(unit_, 0.0);
  }

  double x0() const { return x0_; }
  double y0() const { return y0_; }

  double unit() const { return unit_; }

  template <typename IndexType>
  inline IndexType XToGridXId(double x) const {
    return grid_frame_internal::CoordinateFloorToIndex<IndexType>(x, x0_,
                                                                  unit_inv_);
  }

  template <typename IndexType>
  inline IndexType YToGridYId(double y) const {
    return grid_frame_internal::CoordinateFloorToIndex<IndexType>(y, y0_,
                                                                  unit_inv_);
  }

  template <typename IndexType>
  inline double YIdToY(IndexType yid) const {
    return grid_frame_internal::IndexToCoordinate(yid, y0_, unit_);
  }

  template <typename IndexType>
  inline double XIdToX(IndexType xid) const {
    return grid_frame_internal::IndexToCoordinate(xid, x0_, unit_);
  }

 private:
  double x0_ = 0.0;
  double y0_ = 0.0;
  double unit_ = 1.0;
  double unit_inv_ = 1.0;
};

template <typename IndexType>
class YMonotonicGridSet2d final {
 public:
  YMonotonicGridSet2d(
      IndexType yid_begin, IndexType yid_end,
      std::vector<std::pair<IndexType, IndexType>> xid_begin_end)
      : yid_begin_(yid_begin),
        yid_end_(yid_end),
        xid_begin_end_(std::move(xid_begin_end)) {
    CHECK_EQ(xid_begin_end_.size(), yid_end_ - yid_begin_);
  }

  IndexType yid_begin() const { return yid_begin_; }

  IndexType yid_end() const { return yid_end_; }

  const std::vector<std::pair<IndexType, IndexType>>& xid_begin_end() const {
    return xid_begin_end_;
  }

  bool IsInside(IndexType xid, IndexType yid) {
    if (yid < yid_begin_ || yid >= yid_end_) {
      return false;
    }
    const std::pair<IndexType, IndexType>& xid_interval =
        xid_begin_end_[yid - yid_begin_];
    return xid >= xid_interval.first && xid < xid_interval.second;
  }

 private:
  IndexType yid_begin_ = 0;
  IndexType yid_end_ = 0;
  std::vector<std::pair<IndexType, IndexType>> xid_begin_end_;
};

using YMonotonicGridSet2di = YMonotonicGridSet2d<int>;

}  // namespace e2e_noa

#endif
