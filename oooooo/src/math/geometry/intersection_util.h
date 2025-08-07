#ifndef MATH_GEOMETRY_INTERSECTION_UTIL
#define MATH_GEOMETRY_INTERSECTION_UTIL

#include <optional>
#include <utility>

#include "math/geometry/arc2d.h"
#include "math/geometry/circle2d.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polyline2d.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"

namespace e2e_noa {

bool FindIntersectionBetweenLinesWithTangents(const Vec2d& p0, const Vec2d& t0,
                                              const Vec2d& p1, const Vec2d& t1,
                                              double* s0, double* s1,
                                              Vec2d* inter_point);

bool FindIntersectionBetweenLinesWithTangents(const Vec2d& p0, const Vec2d& t0,
                                              const Vec2d& p1, const Vec2d& t1,
                                              Vec2d* inter_point);

bool FindFirstIntersectionBetweenCurves(const Polyline2d& curve1,
                                        const Polyline2d& curve2,
                                        Vec2d* inter_point, double* arc_len1,
                                        double* arc_len2);

bool ClampSegment2dByHalfPlane(const HalfPlane& plane, Segment2d* segment);

std::pair<std::optional<Vec2d>, std::optional<Vec2d> >
FindIntersectionBetweenCircle2dSegment2d(const Circle2d& circle,
                                         const Segment2d& segment);

std::pair<std::optional<Vec2d>, std::optional<Vec2d> >
FindIntersectionBetweenArc2dHRSegment2d(const Arc2dHR& arc_hr,
                                        const Segment2d& segment);

std::pair<std::optional<Vec2d>, std::optional<Vec2d> >
FindIntersectionBetweenArc2dHRs(const Arc2dHR& a1, const Arc2dHR& a2);

}  // namespace e2e_noa

#endif
