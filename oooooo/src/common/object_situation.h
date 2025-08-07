#ifndef COMMON_OBJECT_SITUATION
#define COMMON_OBJECT_SITUATION
#include <deque>

#include "common/type_def.h"
namespace ad_e2e::planning {

class ObjectSpacetime {
 public:
  ObjectSpacetime() = default;
  ~ObjectSpacetime() = default;

  double lon_speed() const { return lon_speed_; }
  void set_lon_speed(const double &val) { lon_speed_ = val; }

  double lon_acc() const { return lon_acc_; }
  void set_lon_acc(const double &val) { lon_acc_ = val; }

  double lat_speed() const { return lat_speed_; }
  void set_lat_speed(const double &val) { lat_speed_ = val; }

  double response_time() const { return response_time_; }
  void set_response_time(const double &val) { response_time_ = val; }

  double proper_brake() const { return proper_brake_; }
  void set_proper_brake(const double &val) { proper_brake_ = val; }

  double distance_to_junction() const { return distance_to_junction_; }
  void set_distance_junction(const double &val) { distance_to_junction_ = val; }

 private:
  double lon_speed_ = 0.0;
  double lon_acc_ = 0.0;
  double lat_speed_ = 0.0;
  double response_time_ = 1.0;
  double proper_brake_ = Constants::PROPER_BRAKE;
  double distance_to_junction_ = 0.0;
};

class ObjectSituation {
 public:
  ObjectSituation() = default;
  ~ObjectSituation() = default;

  double lon_distance() const { return lon_distance_; }
  void set_lon_distance(const double &val) { lon_distance_ = val; }

  double lon_max() const { return lon_max_; }
  void set_lon_max(const double &val) { lon_max_ = val; }

  double lon_min() const { return lon_min_; }
  void set_lon_min(const double &val) { lon_min_ = val; }

  double lat_distance() const { return lat_distance_; }
  void set_lat_distance(const double &val) { lat_distance_ = val; }

  const ObjectSpacetime &spacetime() const { return spacetime_; }
  ObjectSpacetime &mutableSpacetime() { return spacetime_; }

  void Reset();

 private:
  double lon_distance_ = 0.0;
  double lat_distance_ = 0.0;
  double lon_max_ = 0.0;
  double lon_min_ = 0.0;
  ObjectSpacetime spacetime_;
};
}  // namespace ad_e2e::planning

#endif
