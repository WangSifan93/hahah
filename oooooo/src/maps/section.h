#ifndef MAP_SECTION_H
#define MAP_SECTION_H
#include <map>
#include <unordered_map>
#include <utility>

#include "maps/lane.h"
#include "maps/lane_sequence.h"
#include "maps/map_def.h"

namespace ad_e2e {
namespace planning {
class Section {
 public:
  Section(){};
  ~Section(){};
  enum RoadClass { NORMAL = 1, CITY_EXPRESS = 2, HIGHWAY = 3 };

  void set_id(const std::string& id) { id_ = id; };
  void set_topo_length(double length) { length_ = length; };
  void set_curve_length(double curve_length) { curve_length_ = curve_length; };
  void set_lanes(const std::vector<std::string>& lanes) { lanes_ = lanes; };
  void set_outgoing_sections(
      const std::vector<std::string>& outgoing_sections) {
    outgoing_sections_ = outgoing_sections;
  };
  void set_incoming_sections(
      const std::vector<std::string>& incoming_sections) {
    incoming_sections_ = incoming_sections;
  };
  void set_speed_limit(double speed_limit) { speed_limit_ = speed_limit; };
  void set_average_limit(double average_limit) {
    average_limit_ = average_limit;
  };
  void set_road_class(RoadClass road_class) { road_class_ = road_class; };

  std::string id() const { return id_; };
  double topo_length() const { return length_; };
  double curve_length() const { return curve_length_; };
  const std::vector<std::string>& lanes() const { return lanes_; };
  const std::vector<std::string>& outgoing_sections() const {
    return outgoing_sections_;
  };
  const std::vector<std::string>& incoming_sections() const {
    return incoming_sections_;
  };
  double speed_limit() const { return speed_limit_; };
  double average_limit() const { return average_limit_; };
  RoadClass road_class() const { return road_class_; };

  void add_lanes(const std::string& lane) { lanes_.push_back(lane); };
  void add_outgoing_sections(const std::string& section) {
    outgoing_sections_.push_back(section);
  };
  void add_incoming_sections(const std::string& section) {
    incoming_sections_.push_back(section);
  };

 private:
  std::string id_ = "";
  double length_ = 0.0;
  double curve_length_ = 0.0;
  std::vector<std::string> lanes_;

  std::vector<std::string> outgoing_sections_;
  std::vector<std::string> incoming_sections_;
  double speed_limit_ = 60 / 3.6;
  double average_limit_ = 60 / 3.6;
  RoadClass road_class_ = RoadClass::NORMAL;
};
typedef std::shared_ptr<Section> SectionPtr;
typedef std::shared_ptr<const Section> SectionConstPtr;

}  // namespace planning
}  // namespace ad_e2e

#endif
