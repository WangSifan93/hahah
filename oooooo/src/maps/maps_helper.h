#ifndef ONBOARD_MAPS_MAPS_HELPER_H_
#define ONBOARD_MAPS_MAPS_HELPER_H_

#include <stdint.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include "glog/logging.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"

namespace e2e_noa::mapping {

template <class T>
bool IsOnSameLevel(const T& a, const T& b) {
  for (const auto val : b) {
    if (std::find(a.begin(), a.end(), val) != a.end()) return true;
  }
  return false;
}

double ScaleToValue(int scale, int min_scale, int max_scale,
                    double scale_precision);
int ValueToScale(double value, int min_scale, int max_scale,
                 double scale_precision);
double ScaleToCurvature(int scale);
int CurvatureToScale(double curvature);
double ScaleToHeading(int scale);
int HeadingToScale(double heading);
double ScaleToSlope(int scale);
int SlopeToScale(double slope);
double ScaleToBanking(int scale);
int BankingToScale(double banking);

inline std::vector<Segment2d> Vec2dToSegments(
    const std::vector<Vec2d>& points) {
  CHECK_GE(points.size(), 2);
  std::vector<Segment2d> segments;
  segments.reserve(points.size() - 1);
  for (auto first = points.begin(), second = std::next(first);
       second != points.end(); ++first, ++second) {
    segments.emplace_back(*first, *second);
  }
  return segments;
}

}  // namespace e2e_noa::mapping

#endif
