#ifndef MAP_MAP_DEF_H
#define MAP_MAP_DEF_H

#include <string>
#include <vector>

#include "common/type_def.h"

namespace ad_e2e {
namespace planning {
enum FeatureType {
  FEATURETYPE_LANE = 0,
  FEATURETYPE_LANEBOUNDARY = 1,
  FEATURETYPE_ROADBOUNDARY = 2,
  FEATURETYPE_STOPLINE = 3,
  FEATURETYPE_JUNCTION = 4,
  FEATURETYPE_CROSSWALK = 5,
  FEATURETYPE_SPEEDBUMP = 6,
  FEATURETYPE_CLEARAREA = 7,
};

enum LineColor {
  COLOR_UNKNOWN = 0,
  COLOR_WHITE = 1,
  COLOR_YELLOW = 2,
  COLOR_ORANGE = 3,
  COLOR_BLUE = 4,
  COLOR_GREEN = 5,
  COLOR_GRAY = 6,
  LEFT_GRAY_RIGHT_YELLOW = 7,
  LEFT_YELLOW_RIGHT_WHITE = 8,
  LEFT_WHITE_RIGHT_YELLOW = 9,
  OTHER = 10
};

enum LineType {
  UNKNOWN = 0,
  SOLID = 1,
  DASHED = 2,
  SOLID_SOLID = 3,
  DASHED_DASHED = 4,
  SOLID_DASHED = 5,
  DASHED_SOLID = 6,
  SHADED_AREA = 7,
  VIRTUAL_LANE = 8,
  VIRTUAL_JUNCTION = 9,
  RAMP = 10,
  FISH_LINE = 11,
  CURB_LINE = 12,
  FISH_SOLID = 13,
  FISH_DASH = 14
};

enum BoundaryType {
  UNKNOWN_BOUNDARY = 0,
  LANELINE = 1,
  CURB = 2,
  CENTER = 3,
  GUARDRAIL = 4,
  CONCRETE_BARRIER = 5,
  FENCE = 6,
  WALL = 7,
  CANOPY = 8,
  PAVE = 9,
  DITCH = 10,
  PUNCHEON = 11,
  VIRTUAL = 12,
  LIDAR_UU = 13,
  OCC_UU = 14,
  OCC_CURB = 15,
  OCC_VEGETATION = 16,
  OCC_CONJECTURE = 17
};

enum LaneType {
  LANE_UNKNOWN = 0,
  LANE_NORMAL = 1,
  LANE_ACC = 2,
  LANE_DEC = 3,
  LANE_RAMP = 4,
  LANE_EMERGENCY = 5,
  LANE_ACC_DCC = 6,
  LANE_BUS_NORMAL = 7,
  LANE_HOV_NORMAL = 8,
  LANE_NON_MOTOR = 9,
  LANE_LEFT_WAIT = 10,
  LANE_VIRTUAL_COMMON = 11,
  LANE_VIRTUAL_JUNCTION = 12,
  LANE_ROUND_ABOUT = 13
};

enum class CompositeTurnType {
  NORMAL_TURN = 0,
  LEFT_STRAIGHT = 1,
  STRAIGHT_RIGHT = 2,
  LEFT_RIGHT = 3,
  LEFT_STRAIGHT_RIGHT = 4
};

enum LightStatus {
  NONE_LIGHT = 0,
  GREEN_LIGHT = 1,
  YELLOW_LIGHT = 2,
  RED_LIGHT = 3,
  UNKNOWN_LIGHT = 4,
  YELLOW_BLINKING = 5,
  FAIL_DETECTION = 6,
  BLOCK_FAIL = 7
};

enum StopLineReason {
  REASON_NONE = 0,
  REASON_LIGHT_RED = 1,
  REASON_LIGHT_YELLOW = 2,
  REASON_CONFIRM = 3
};

enum StopLineInterface {
  STOP_LINE_NONE = 0,
  STOP_LINE_RED = 1,
  STOP_LINE_YELLOW = 2,
  STOP_LINE_CONFIRM = 3,
  STOP_LINE_UNKNOWN = 4,
  STOP_LINE_FAIL_DETECT = 5,
  STOP_LINE_LCC_TURN = 6,
  STOP_LINE_T_JUNCTION = 10
};

enum TrafficLightSubType {
  TRAFFICLIGHT_DIRECTION_OTHER_DIRECTION = 0x00,
  TRAFFICLIGHT_DIRECTION_UP = 0x02,
  TRAFFICLIGHT_DIRECTION_LEFT = 0x04,
  TRAFFICLIGHT_DIRECTION_RIGHT = 0x08,
  TRAFFICLIGHT_DIRECTION_UTURN = 0x10
};

struct TrafficLightInfo {
  std::string id{""};
  Point2d center_point;
  int32_t sub_type = 0;
  int32_t count_down_sec = 255;
  bool is_blinking = false;
  LightStatus light_status = NONE_LIGHT;
};

struct FsdTrafficLightDeciderInfo {
  double dist_to_stopline = std::numeric_limits<double>::max();
  double dist_to_leftwait_stopline = std::numeric_limits<double>::max();
  bool ego_is_in_leftwait = false;
  bool first_virtual_lane_is_leftwait = false;
  std::string first_virtual_lane = "";
  std::string focus_lane = "";
  std::string left_wait_lane = "";
  LightStatus first_virtual_lane_light = NONE_LIGHT;
  LightStatus focus_lane_light = NONE_LIGHT;
  LightStatus left_wait_lane_light = NONE_LIGHT;
  TrafficLightInfo focus_lane_light_info;
};

struct TrafficLightStatus {
  std::string lane_id;
  std::optional<std::string> junction_id = std::nullopt;
  LightStatus light_status = LightStatus::NONE_LIGHT;
  bool stop_line = false;
  bool is_left_wait_lane = false;
  TrafficLightInfo traffic_light_info;
};

using TrafficLightStatusMap =
    std::unordered_map<std::string, TrafficLightStatus>;

enum MergeTopology {
  TOPOLOGY_MERGE_NONE = 0,
  TOPOLOGY_MERGE_LEFT = 1,
  TOPOLOGY_MERGE_RIGHT = 2,
  TOPOLOGY_TO_BE_MERGED = 3
};

enum SplitTopology {
  TOPOLOGY_SPLIT_NONE = 0,
  TOPOLOGY_SPLIT_LEFT = 1,
  TOPOLOGY_SPLIT_RIGHT = 2
};

enum NoneOddType { TYPE_NONE = 0, TYPE_TOLL = 1, TYPE_CONSTRUCTION = 2 };

enum PoiType {
  Poi_NaviEnd = 0,
  Poi_Split = 1,
  Poi_To_Be_Merged = 2,
  Poi_Merge = 3
};

enum XRoadType {
  XRoadType_Ramp = 0,
  XRoadType_NoneOdd = 1,
  XRoadType_Tunnel = 2
};

enum ImpassableAeraType {
  UNKNOWN_KIND = 0,
  FLOWERBED = 1,
  SENTRY_BOX = 2,
  PHYSICAL_SAFE_ISLAND = 3,
  LINEAR_SAFE_ISLAND = 4
};
enum StopLineType {
  STOPLINETYPE_UNKNOWN = 0,
  STOPLINETYPE_STRAIGHT = 1,
  STOPLINETYPE_LEFT_WAIT = 2
};

struct RoadBoundaryInfo {
  std::string id;
  double width = 0.0;
  std::vector<Point2d> points;
  BoundaryType boundary_type = UNKNOWN_BOUNDARY;
};

struct RoadBoundaryType {
  double s = 0.0;
  double width = 0.0;
  BoundaryType boundary_type = UNKNOWN_BOUNDARY;
};

struct LaneBoundarySegmentInfo {
  std::string id;
  std::vector<Point2d> points;
  LineType line_type = UNKNOWN;
  LineColor line_color = COLOR_UNKNOWN;
};

struct LaneBoundaryType {
  double s = 0.0;
  LineType line_type = UNKNOWN;
  LineColor line_color = COLOR_UNKNOWN;
};

struct LaneBoundaryInfo {
  std::string id;
  std::vector<Point2d> points;
  LaneBoundaryType boundary_type;
};

struct LaneInfo {
  std::string id;
  std::string section_id;
  std::string junction_id;
  std::string left_lane_id;
  std::string right_lane_id;
  std::vector<std::string> next_lane_ids;
  std::vector<std::string> left_lane_boundary_ids;
  std::vector<std::string> right_lane_boundary_ids;
  std::vector<std::string> left_road_boundary_ids;
  std::vector<std::string> right_road_boundary_ids;
  LaneType type = LANE_NORMAL;
  NoneOddType none_odd_type = TYPE_NONE;
  TurnType turn_type = NO_TURN;
  uint32_t turn_type_group = 0;
  LightStatus light_status = NONE_LIGHT;
  SplitTopology split_topology = TOPOLOGY_SPLIT_NONE;
  MergeTopology merge_topology = TOPOLOGY_MERGE_NONE;
  std::vector<Point2d> points;
  double length = 0.0;
  double speed_limit = 135 / 3.6;
  bool is_virtual = false;
  bool is_split = false;
  bool is_merge = false;
  bool is_navigation = false;
  bool stop_line = false;
  std::vector<std::string> cross_walks;
  std::vector<std::string> speed_bumps;
  std::vector<std::string> parking_spaces;
  std::vector<std::string> clear_areas;
  std::vector<std::string> traffic_stop_lines;
  std::vector<std::string> traffic_lights;
  int32_t lane_operation_type = 0;
  std::vector<double> coeffs;
  int32_t arrow_type = 0;
};
struct SectionInfo {
  std::string id;
  double length = 0.0;
  NoneOddType none_odd_type = TYPE_NONE;
  std::vector<std::string> lane_ids;
  std::string navi_priority_lane_id;
};
struct NaviPosition {
  std::string section_id;
  double s_offset;
};
struct RouteInfo {
  std::string id;
  NaviPosition navi_start;
  NaviPosition navi_end;
  std::vector<SectionInfo> sections;
};
struct StopLineInfo {
  std::string id;
  std::vector<Point2d> points;
  StopLineType type;
  LightStatus light_type;
  int8_t sub_type;
  int8_t virtual_type;
};
struct JunctionInfo {
  std::string id;
  std::vector<Point2d> points;
};
struct CrossWalkInfo {
  std::string id;
  std::vector<Point2d> points;
};
struct SpeedBumpInfo {
  std::string id;
  std::vector<Point2d> points;
};
struct ParkingSpaceInfo {
  std::string id;
  std::vector<Point2d> points;
};
struct ClearAreaInfo {
  std::string id;
  ImpassableAeraType type;
  std::vector<Point2d> points;
};

struct V2RoadClass {
  double start_s = 0.0;
  double end_s = 0.0;
  enum V2RoadClassType {
    UNKNOWN_ROAD = 0,
    HIGH_WAY_ROAD = 1,
    EXPRESS_WAY_ROAD = 2,
    NATIOANL_ROAD = 3,
    PROVINCIAL_ROAD = 4,
    MAIN_ROAD = 5,
    SUB_ROAD = 6
  };
  V2RoadClassType type = UNKNOWN_ROAD;
};

struct V2TrafficFlow {
  double start_s = 0.0;
  double end_s = 0.0;
  enum V2TrafficFlowType {
    UNKNOWN_FLOW = 0,
    SMOOTH_FLOW = 1,
    SLOW_FLOW = 2,
    JAMMED_FLOW = 3,
    SERVERE_JAMMED_FLOW = 4,
    NO_FLOW = 5
  };
  V2TrafficFlowType type = UNKNOWN_FLOW;
};

struct RoadInfo {
  int road_class = 0;
  int lane_num = 0;
};

struct PassInfo {
  int curr_index = -1;
  int index_num = -1;
};

struct V2TurnInfo {
  std::string id = "";
  bool is_valid = false;
  enum V2TurnType {
    UNKNOWN = 0,
    LEFT = 1,
    RIGHT = 2,
    STRAIGHT = 3,
    U_TURN_LEFT = 4,
    U_TURN_RIGHT = 5,
    MERGE_LEFT = 6,
    MERGE_RIGHT = 7,
    RAMP_LEFT = 11,
    RAMP_RIGHT = 12,
    RAMP_STRAIGHT = 13,
    RAMP_U_TURN_LEFT = 14,
    RAMP_U_TURN_RIGHT = 15,
  };
  V2TurnType turn_type = UNKNOWN;
  enum V2DetailTurnType {
    NONE = 0,
    TURN_LEFT = 2,
    TURN_RIGHT = 3,
    SLIGHT_LEFT = 4,
    SLIGHT_RIGHT = 5,
    TURN_HARD_LEFT = 6,
    TURN_HARD_RIGHT = 7,
    UTURN = 8,
    CONTINUE = 9,
    TURN_RIGHT_ONLY = 10,
    UTURN_RIGHT = 19,
    LEFT_MERGE = 65,
    RIGHT_MERGE = 66,
  };
  V2DetailTurnType detail_turn_type = NONE;
  double dist = 0.0;
  uint32_t original_dist = 0;
  RoadInfo before_turn;
  RoadInfo after_turn;
  std::vector<std::string> infos;
  PassInfo straight_pass_info;
};

struct EHPV2Info {
  bool has_navigation = false;
  double dist_to_ramp = 0.0;
  double dist_to_toll = 0.0;
  std::vector<V2RoadClass> road_class;
  std::vector<V2TrafficFlow> traffic_flow;
  std::vector<V2TurnInfo> turn_info;
  std::vector<V2TurnInfo> pnp_turn_info;
};

struct MapInfo {
  MapType type = BEV_MAP;
  MapSubType sub_type;
  double timestamp = 0.0;
  int64_t seq = 0;
  bool is_on_highway = false;
  std::vector<LaneInfo> all_lanes_vec;
  std::vector<LaneBoundaryInfo> all_lane_boundaries_vec;
  std::vector<RoadBoundaryInfo> all_road_boundaries_vec;
  std::vector<StopLineInfo> all_stop_lines_vec;
  std::vector<JunctionInfo> all_junctions_vec;
  std::vector<CrossWalkInfo> all_cross_walks_vec;
  std::vector<SpeedBumpInfo> all_speed_bumps_vec;
  std::vector<ParkingSpaceInfo> all_parking_spaces_vec;
  std::vector<ClearAreaInfo> all_clear_areas_vec;
  std::vector<TrafficLightInfo> all_traffic_lights_vec;
  RouteInfo route;
  EHPV2Info v2_info;
};

}  // namespace planning
}  // namespace ad_e2e

#endif
