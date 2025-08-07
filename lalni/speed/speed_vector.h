#ifndef PLANNER_SPEED_SPEED_VECTOR_H_
#define PLANNER_SPEED_SPEED_VECTOR_H_

#include <memory>
#include <optional>
#include <vector>

#include "speed/speed_point.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {

class SpeedVector : public std::vector<SpeedPoint> {
 public:
  SpeedVector() = default;

  explicit SpeedVector(std::vector<SpeedPoint> speed_points);

  std::optional<SpeedPoint> EvaluateByTime(double t) const;

  std::optional<SpeedPoint> EvaluateByS(double s) const;

  double TotalTime() const;

  double TotalLength() const;

  template <template <class...> class Container>
  void FromProto(const Container<SpeedPointProto>& speed_points) {
    reserve(speed_points.size());
    for (const auto& speed_point : speed_points) {
      emplace_back().FromProto(speed_point);
    }
    std::stable_sort(begin(), end(),
                     [](const SpeedPoint& p1, const SpeedPoint& p2) {
                       return p1.t() < p2.t();
                     });
  }

  void ToProto(SpeedPointsProto* speed_points) const {
    speed_points->Clear();
    for (auto it = begin(); it != end(); ++it) {
      it->ToProto(speed_points->add_speed_points());
    }
  }
};

}  // namespace e2e_noa::planning

#endif
