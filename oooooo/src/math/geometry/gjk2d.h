#ifndef MATH_GEOMETRY_GJK_H_
#define MATH_GEOMETRY_GJK_H_
#include <algorithm>
#include <functional>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "math/geometry/segment2d.h"
#include "math/vec.h"
namespace e2e_noa {
class Gjk2d {
 public:
  template <typename F>
  static bool HasOverlap(const Vec2d& point_inside_hint,
                         const F& calculate_extreme_point,
                         double min_distance_sqr, int max_iterations);

  template <typename F>
  static double DistanceSquareTo(const Vec2d& point_inside_hint,
                                 const F& calculate_extreme_point,
                                 double min_distance_sqr, int max_iterations);

  template <typename F>
  static double DistanceTo(const Vec2d& point_inside_hint,
                           const F& calculate_extreme_point,
                           double min_distance_sqr, int max_iterations) {
    return std::sqrt(DistanceSquareTo(point_inside_hint,
                                      calculate_extreme_point, min_distance_sqr,
                                      max_iterations));
  }

  template <typename F>
  static bool GetMinPenetrationDistance(Vec2d point_inside_hint,
                                        const F& calculate_extreme_point,
                                        double min_distance, int max_iterations,
                                        double* min_penetration,
                                        Vec2d* dir_vec);

  template <typename F>
  static bool GetMinPenetrationDistance(Vec2d point_inside_hint,
                                        const F& calculate_extreme_point,
                                        double min_distance, int max_iterations,
                                        double* min_penetration);

  template <typename F>
  static bool GetPenetrationDistanceAlongDir(Vec2d point_inside_hint,
                                             const F& calculate_extreme_point,
                                             const Vec2d& dir_vec,
                                             double min_distance,
                                             int max_iterations,
                                             double* penetration);

 private:
  static double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                          const Vec2d& end_point_2) {
    return Vec2d(end_point_1 - start_point)
        .CrossProd(end_point_2 - start_point);
  }
  static Vec2d UpdateSearchDirectionFromSegment(std::array<Vec2d, 3>* simplex,
                                                int* cardinality);
  static Vec2d ConeRegion(std::array<Vec2d, 3>* simplex, int index,
                          int* cardinality);
  static Vec2d UpdateSearchDirectionFromTriangle(std::array<Vec2d, 3>* simplex,
                                                 int* cardinality);
  static Vec2d SubDistance(std::array<Vec2d, 3>* simplex, int* cardinality);
  template <typename F>
  static std::optional<std::array<Vec2d, 3>> GetGjkSimplex(
      const Vec2d& point_inside_hint, const F& calculate_extreme_point,
      double min_distance_sqr, int max_iterations);
  static constexpr double kEpsilon = 1e-10;
};

inline Vec2d Gjk2d::UpdateSearchDirectionFromSegment(
    std::array<Vec2d, 3>* simplex, int* cardinality) {
  const auto& a = (*simplex)[0];
  const auto& b = (*simplex)[1];
  const auto ab = b - a;
  const double a_dot_ab = a.dot(ab);
  if (a_dot_ab >= 0.0) {
    *cardinality = 1;
    return a;
  }
  const double b_dot_ab = b.dot(ab);
  if (b_dot_ab <= 0.0) {
    (*simplex)[0] = b;
    *cardinality = 1;
    return b;
  }
  const double sum = a_dot_ab - b_dot_ab;
  const double u = -b_dot_ab / sum;
  const double v = a_dot_ab / sum;
  *cardinality = 2;
  return u * a + v * b;
}

inline Vec2d Gjk2d::ConeRegion(std::array<Vec2d, 3>* simplex, int index,
                               int* cardinality) {
  std::swap((*simplex)[index], (*simplex)[2]);
  const auto& m = (*simplex)[0];
  const auto& n = (*simplex)[1];
  const auto& v = (*simplex)[2];
  const auto mv = v - m;
  const auto nv = v - n;
  if (mv.dot(nv) < 0.0) {
    if (v.dot(mv) > 0.0) {
      (*simplex)[1] = v;
      return UpdateSearchDirectionFromSegment(simplex, cardinality);
    }
    if (v.dot(nv) > 0.0) {
      (*simplex)[0] = v;
      return UpdateSearchDirectionFromSegment(simplex, cardinality);
    }
  }
  *cardinality = 1;
  (*simplex)[0] = v;
  return v;
}

inline Vec2d Gjk2d::UpdateSearchDirectionFromTriangle(
    std::array<Vec2d, 3>* simplex, int* cardinality) {
  const auto ssign = [](double m, double n) -> int {
    return (m > 0.0) == (n > 0.0);
  };
  const auto sigma_u = (*simplex)[1].CrossProd((*simplex)[2]);
  const auto sigma_v = (*simplex)[2].CrossProd((*simplex)[0]);
  const auto sigma_w = (*simplex)[0].CrossProd((*simplex)[1]);
  const auto sum = sigma_u + sigma_v + sigma_w;
  const auto barycode = ssign(sum, sigma_w) + (ssign(sum, sigma_v) << 1) +
                        (ssign(sum, sigma_u) << 2);
  switch (barycode) {
    case 1:
      return ConeRegion(simplex, 2, cardinality);
    case 2:
      return ConeRegion(simplex, 1, cardinality);
    case 3:
      (*simplex)[0] = std::move((*simplex)[1]);
      (*simplex)[1] = std::move((*simplex)[2]);
      return UpdateSearchDirectionFromSegment(simplex, cardinality);
    case 4:
      return ConeRegion(simplex, 0, cardinality);
    case 5:
      (*simplex)[1] = std::move((*simplex)[2]);
      return UpdateSearchDirectionFromSegment(simplex, cardinality);
    case 6:
      return UpdateSearchDirectionFromSegment(simplex, cardinality);
    case 7:
      break;
  }
  return Vec2d(0.0, 0.0);
}

inline Vec2d Gjk2d::SubDistance(std::array<Vec2d, 3>* simplex,
                                int* cardinality) {
  switch (*cardinality) {
    case 3:
      return UpdateSearchDirectionFromTriangle(simplex, cardinality);
    case 2:
      return UpdateSearchDirectionFromSegment(simplex, cardinality);
    case 1:
      return (*simplex)[0];
  }
  return Vec2d(0.0, 0.0);
}

template <typename F>
inline std::optional<std::array<Vec2d, 3>> Gjk2d::GetGjkSimplex(
    const Vec2d& point_inside_hint, const F& calculate_extreme_point,
    double min_distance_sqr, int max_iterations) {
  if (point_inside_hint.Sqr() < min_distance_sqr) {
    return std::nullopt;
  }
  std::array<Vec2d, 3> simplex;
  simplex[0] = calculate_extreme_point(-point_inside_hint);
  auto support_vec = -simplex[0];
  int cardinality = 1;
  int iter = 0;
  while (cardinality < 3 && ++iter <= max_iterations) {
    auto w = -calculate_extreme_point(support_vec);
    if (support_vec.dot(w) > 0.0) {
      return std::nullopt;
    }
    simplex[cardinality] = -w;
    ++cardinality;
    support_vec = -SubDistance(&simplex, &cardinality);
    if (cardinality == 3) {
      break;
    }

    if (support_vec.Sqr() == 0.0) {
      support_vec = Vec2d(0.0, -1.0);
    }
  }
  if (cardinality != 3) {
    return std::nullopt;
  }
  return simplex;
}

template <typename F>
inline bool Gjk2d::HasOverlap(const Vec2d& point_inside_hint,
                              const F& calculate_extreme_point,
                              double min_distance_sqr, int max_iterations) {
  if (point_inside_hint.Sqr() < min_distance_sqr) {
    return true;
  }
  std::array<Vec2d, 3> simplex;
  simplex[0] = calculate_extreme_point(-point_inside_hint);
  auto support_vec = -simplex[0];
  int cardinality = 1;
  int iter = 0;
  while (cardinality < 3 && ++iter <= max_iterations) {
    const auto w = -calculate_extreme_point(support_vec);
    if (support_vec.dot(w) > 0.0) {
      return false;
    } else if (cardinality == 2 &&
               simplex[0].CrossProd(w) * simplex[1].CrossProd(w) <= 0.0) {
      return true;
    }
    simplex[cardinality] = -w;
    ++cardinality;
    support_vec = -SubDistance(&simplex, &cardinality);
    if (support_vec.Sqr() < min_distance_sqr) {
      return true;
    }
  }
  return support_vec.Sqr() < min_distance_sqr;
}

template <typename F>
inline double Gjk2d::DistanceSquareTo(const Vec2d& point_inside_hint,
                                      const F& calculate_extreme_point,
                                      double min_distance_sqr,
                                      int max_iterations) {
  if (point_inside_hint.Sqr() < min_distance_sqr) {
    return 0.0;
  }
  std::array<Vec2d, 3> simplex;
  simplex[0] = calculate_extreme_point(-point_inside_hint);
  auto support_vec = -simplex[0];
  int cardinality = 1;
  int iter = 0;
  while (cardinality < 3 && ++iter <= max_iterations) {
    const auto w = -calculate_extreme_point(support_vec);
    if (support_vec.dot(w) >= (1.0 - min_distance_sqr) * support_vec.Sqr()) {
      break;
    }
    simplex[cardinality] = -w;
    ++cardinality;
    support_vec = -SubDistance(&simplex, &cardinality);
    if (support_vec.Sqr() <= min_distance_sqr) {
      return 0.0;
    }
  }
  return support_vec.Sqr();
}

template <typename F>
inline bool Gjk2d::GetMinPenetrationDistance(Vec2d point_inside_hint,
                                             const F& calculate_extreme_point,
                                             double min_distance,
                                             int max_iterations,
                                             double* min_penetration,
                                             Vec2d* dir_vec) {
  CHECK_NOTNULL(min_penetration);
  CHECK_NOTNULL(dir_vec);
  const double min_distance_sqr = Sqr(min_distance);
  if (point_inside_hint.Sqr() < min_distance_sqr) {
    point_inside_hint = Vec2d(-1.0, 0.0);
  }
  const auto simplex_or =
      GetGjkSimplex(point_inside_hint, calculate_extreme_point,
                    min_distance_sqr, max_iterations);
  if (!simplex_or.has_value()) {
    *min_penetration = 0.0;
    return false;
  }
  const auto simplex = std::move(*simplex_or);
  const auto min_dist_edge_comp = [](const auto& p1, const auto& p2) -> bool {
    return std::fabs(p1.first) > std::fabs(p2.first);
  };
  std::priority_queue<std::pair<double, Segment2d>,
                      std::vector<std::pair<double, Segment2d>>,
                      decltype(min_dist_edge_comp)>
      min_dist_edge_queue(min_dist_edge_comp);
  const Vec2d origin(0.0, 0.0);
  const Vec2d center = (simplex[0] + simplex[1] + simplex[2]) / 3.0;
  for (int i = 0; i < 3; ++i) {
    Segment2d edge(simplex[i], simplex[(i + 1) % 3]);
    const double dist_to_origin = edge.SignedDistanceTo(origin);
    min_dist_edge_queue.emplace(dist_to_origin, std::move(edge));
  }

  while (true) {
    const auto signed_dist_to_origin = min_dist_edge_queue.top().first;
    const auto dist_to_origin = std::fabs(signed_dist_to_origin);
    const auto edge = min_dist_edge_queue.top().second;
    min_dist_edge_queue.pop();
    auto normal = edge.unit_direction().Perp();
    if (dist_to_origin < min_distance) {
      if (edge.SignedDistanceTo(center) < 0.0) {
        normal = -normal;
      }
    } else if (signed_dist_to_origin < 0.0) {
      normal = -normal;
    }
    auto extreme_point = calculate_extreme_point(normal);
    double d = extreme_point.dot(normal);
    if (d - dist_to_origin < min_distance) {
      *dir_vec = std::move(normal);
      *min_penetration = d;
      return true;
    } else {
      Segment2d one_edge(edge.start(), extreme_point);
      min_dist_edge_queue.emplace(one_edge.SignedDistanceTo(origin),
                                  std::move(one_edge));
      Segment2d other_edge(edge.end(), extreme_point);
      min_dist_edge_queue.emplace(other_edge.SignedDistanceTo(origin),
                                  std::move(other_edge));
    }
  }
  *min_penetration = 0.0;
  return false;
}

template <typename F>
inline bool Gjk2d::GetMinPenetrationDistance(Vec2d point_inside_hint,
                                             const F& calculate_extreme_point,
                                             double min_distance,
                                             int max_iterations,
                                             double* min_penetration) {
  Vec2d unused;
  return GetMinPenetrationDistance(std::move(point_inside_hint),
                                   calculate_extreme_point, min_distance,
                                   max_iterations, min_penetration, &unused);
}

template <typename F>
inline bool Gjk2d::GetPenetrationDistanceAlongDir(
    Vec2d point_inside_hint, const F& calculate_extreme_point,
    const Vec2d& dir_vec, double min_distance, int max_iterations,
    double* penetration) {
  CHECK_NOTNULL(penetration);
  const double min_distance_sqr = Sqr(min_distance);

  if (point_inside_hint.Sqr() < min_distance_sqr) {
    point_inside_hint = -dir_vec;
  }
  const auto simplex_or =
      GetGjkSimplex(point_inside_hint, calculate_extreme_point,
                    min_distance_sqr, max_iterations);
  if (!simplex_or.has_value()) {
    *penetration = 0.0;
    return false;
  }
  const auto simplex = std::move(*simplex_or);
  const Vec2d origin(0.0, 0.0);
  constexpr double kRayLength = 1e8;

  const Segment2d dir_ray(Vec2d(0.0, 0.0), dir_vec.Unit() * kRayLength);
  Segment2d cross_edge;

  for (int i = 0; i < 3; ++i) {
    Segment2d edge(simplex[i], simplex[(i + 1) % 3]);

    if (dir_ray.HasIntersect(edge)) {
      cross_edge = std::move(edge);
      break;
    }
  }

  while (true) {
    auto normal = cross_edge.unit_direction().Perp();
    double signed_dist_to_origin = cross_edge.SignedDistanceTo(origin);
    if (signed_dist_to_origin < 0.0) {
      normal = -normal;
      signed_dist_to_origin = -signed_dist_to_origin;
    }
    auto extreme_point = calculate_extreme_point(normal);
    double d = extreme_point.dot(normal);
    if (d - signed_dist_to_origin < min_distance) {
      Vec2d inter;
      dir_ray.GetIntersect(cross_edge, &inter);
      *penetration = inter.norm();
      return true;
    } else {
      if (CrossProd(dir_ray.start(), dir_ray.end(), cross_edge.start()) *
              CrossProd(dir_ray.start(), dir_ray.end(), extreme_point) <
          -kEpsilon) {
        Segment2d one_edge(cross_edge.start(), extreme_point);
        cross_edge = std::move(one_edge);
        continue;
      }
      Segment2d other_edge(cross_edge.end(), extreme_point);
      cross_edge = std::move(other_edge);
    }
  }
  *penetration = 0.0;
  return false;
}
}  // namespace e2e_noa
#endif
