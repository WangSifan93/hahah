
#include "common/log_data.h"

#include <absl/time/clock.h>
#include <absl/time/time.h>

#ifdef BUILD_ONBOARD
DEFINE_int32(Log2FG_verbosity_level, 2, "Log2FG verbosity level orin");
#else
DEFINE_int32(Log2FG_verbosity_level, 4, "Log2FG verbosity level local");
#endif
namespace e2e_noa {
namespace planning {

namespace debug {

Marker::Point VectorToPoint(const e2e_noa::Vec2d& vec) {
  Marker::Point point{};
  if (vec.size() > 0) {
    point.set_x(vec[0]);
  }
  if (vec.size() > 1) {
    point.set_y(vec[1]);
  }
  if (vec.size() > 2) {
    point.set_z(vec[2]);
  }
  return point;
}

void ConvertVectorsToMarkerPoints(Marker* marker,
                                  const std::vector<e2e_noa::Vec2d>& vectors) {
  for (const auto& vec : vectors) {
    Marker::Point* point = marker->add_points();
    *point = VectorToPoint(vec);
  }
}

void VectorsToMarker(Marker* marker, std::string marker_id,
                     const std::vector<e2e_noa::Vec2d>& vectors,
                     Marker::Type type, Marker::Color color, bool is_closed) {
  ConvertVectorsToMarkerPoints(marker, vectors);
  marker->set_id(marker_id);
  marker->set_type(type);
  marker->mutable_color()->MergeFrom(color);
}

void PolygonToMarker(Marker* marker, std::string marker_id,
                     const e2e_noa::Polygon2d& polygon, Marker::Color color,
                     bool is_closed, Marker::Type type) {
  ConvertVectorsToMarkerPoints(marker, polygon.points());
  marker->set_id(marker_id);
  marker->set_type(type);
  marker->mutable_color()->MergeFrom(color);
}
}  // namespace debug

const Log2FG::Color Log2FG::kBlack = MAKE_COLOR(0, 0, 0, 1);
const Log2FG::Color Log2FG::kWhite = MAKE_COLOR(255, 255, 255, 1);
const Log2FG::Color Log2FG::kRed = MAKE_COLOR(255, 0, 0, 1);
const Log2FG::Color Log2FG::kGreen = MAKE_COLOR(0, 128, 0, 1);
const Log2FG::Color Log2FG::kBlue = MAKE_COLOR(0, 0, 255, 1);
const Log2FG::Color Log2FG::kYellow = MAKE_COLOR(255, 255, 0, 1);
const Log2FG::Color Log2FG::kOrange = MAKE_COLOR(255, 165, 0, 1);
const Log2FG::Color Log2FG::kPink = MAKE_COLOR(255, 192, 203, 1);
const Log2FG::Color Log2FG::kPurple = MAKE_COLOR(128, 0, 128, 1);
const Log2FG::Color Log2FG::kBrown = MAKE_COLOR(165, 42, 42, 1);
const Log2FG::Color Log2FG::kGray = MAKE_COLOR(128, 128, 128, 1);
const Log2FG::Color Log2FG::kHotpink = MAKE_COLOR(255, 105, 180, 1);
const Log2FG::Color Log2FG::kCoral = MAKE_COLOR(255, 127, 80, 1);
const Log2FG::Color Log2FG::kDarkkhaki = MAKE_COLOR(189, 183, 107, 1);
const Log2FG::Color Log2FG::kViolet = MAKE_COLOR(238, 130, 238, 1);
const Log2FG::Color Log2FG::kLime = MAKE_COLOR(0, 255, 0, 1);
const Log2FG::Color Log2FG::kAqua = MAKE_COLOR(0, 255, 255, 1);
const Log2FG::Color Log2FG::kMagenta = MAKE_COLOR(255, 0, 255, 1);
const Log2FG::Color Log2FG::kDarkRed = MAKE_COLOR(139, 0, 0, 1);
const Log2FG::Color Log2FG::kLightGray = MAKE_COLOR(179, 179, 179, 0.1);
const Log2FG::Color Log2FG::kMiddleBlueGreen = MAKE_COLOR(141, 217, 204, 1);
const Log2FG::Color Log2FG::kTiffanyBlue = MAKE_COLOR(129, 216, 208, 1);
const Log2FG::Color Log2FG::kLightBlue = MAKE_COLOR(173, 216, 230, 1);
const Log2FG::Color Log2FG::kDarkGreen = MAKE_COLOR(136, 135, 13, 1);
const Log2FG::Color Log2FG::kDarkBlue = MAKE_COLOR(0, 0, 139, 1);
const Log2FG::Color Log2FG::kGrassGreen = MAKE_COLOR(18, 231, 115, 1);

zark::e2e_noa::debug::NOADebugInfo_PlanningDebugInfo&
Log2FG::GetThreadDebugFrame(const std::string plan_id) {
  auto& debug_info = NOA_DEBUG_INFO;
  std::thread::id thread_id = std::this_thread::get_id();
  std::string thread_id_hash =
      std::to_string(std::hash<std::thread::id>{}(thread_id));
  std::unique_lock mutex(mutex_);
  if (plan_id != "") {
    auto it = thread_plan_id_map_.find(plan_id);
    if (it != thread_plan_id_map_.end() &&
        debug_info.planning_debug_info_map().find(it->second) !=
            debug_info.planning_debug_info_map().end()) {
      return (*debug_info.mutable_planning_debug_info_map())[it->second];
    } else if (it == thread_plan_id_map_.end()) {
      thread_plan_id_map_[plan_id] = thread_id_hash;
      auto& result =
          (*debug_info.mutable_planning_debug_info_map())[thread_id_hash] =
              zark::e2e_noa::debug::NOADebugInfo_PlanningDebugInfo{};
      result.set_plan_id(plan_id);
      return result;
    }
  } else {
    auto it = debug_info.planning_debug_info_map().find(thread_id_hash);
    if (it != debug_info.planning_debug_info_map().end()) {
      return (*debug_info.mutable_planning_debug_info_map())[thread_id_hash];
    }
  }
  auto it = debug_info.planning_debug_info_map().find(default_pos);
  if (it != debug_info.planning_debug_info_map().end()) {
    return (*debug_info.mutable_planning_debug_info_map())[default_pos];
  }

  auto& result = (*debug_info.mutable_planning_debug_info_map())[default_pos] =
      zark::e2e_noa::debug::NOADebugInfo_PlanningDebugInfo{};
  result.set_plan_id(default_pos);

  return result;
}

zark::e2e_noa::debug::NOADebugInfo::ChooseLaneDebugInfo::PlanPassage&
Log2FG::GetPlanPassage(std::string plan_id) {
  auto& chs = *NOA_DEBUG_INFO.mutable_choose_lane_debug_info();
  {
    std::shared_lock read_lock(plan_passage_mutex_);
    auto it = chs.plan_passages().find(plan_id);
    if (it != chs.plan_passages().end()) {
      return (*chs.mutable_plan_passages())[plan_id];
    }
  }

  std::unique_lock write_lock(plan_passage_mutex_);

  auto it = chs.plan_passages().find(plan_id);
  if (it != chs.plan_passages().end()) {
    return (*chs.mutable_plan_passages())[plan_id];
  }

  auto& result = (*chs.mutable_plan_passages())[plan_id] =
      zark::e2e_noa::debug::NOADebugInfo_ChooseLaneDebugInfo::PlanPassage{};
  return result;
}

void Log2FG::LogMissionData(const std::string& key, const std::string& value) {
  auto* mission_debug = NOA_DEBUG_INFO.mutable_mission_debug_info();
  if (mission_debug->find(key) != mission_debug->end()) {
    (*mission_debug)[key] = (*mission_debug)[key] + "\n" + value;
  } else {
    (*mission_debug)[key] = value;
  }
}

void Log2FG::LogVehData(const std::string& key, const std::string& value) {
  auto* vehicle_debug = NOA_DEBUG_INFO.mutable_vehicle_debug_info();
  if (vehicle_debug->find(key) != vehicle_debug->end()) {
    vehicle_debug->at(key) = vehicle_debug->at(key) + "\n" + value;
  } else {
    (*vehicle_debug)[key] = value;
  }
}

void Log2FG::LogDataV(int level, const std::string& key,
                      const std::string& value) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }

  auto& debug_frame = GetThreadDebugFrame();
  if (debug_frame.plan_id() == default_pos) {
    VLOG(2) << "Forbidden to log data in default plan id!";
    return;
  }

  if (debug_frame.strings().find(key) != debug_frame.strings().end()) {
    (*debug_frame.mutable_strings())[key] =
        (*debug_frame.mutable_strings())[key] + "\n\t" + value;
  } else {
    (*debug_frame.mutable_strings())[key] = value;
  }
}

void Log2FG::LogDataV(int level, const std::string& key, double value) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }
  auto& debug_frame = GetThreadDebugFrame();
  if (debug_frame.plan_id() == default_pos) {
    VLOG(2) << "Forbidden to log data in default plan id!";
    return;
  }

  (*debug_frame.mutable_numbers())[key] = value;
}

void Log2FG::LogDataV(int level, const std::string& key,
                      const std::vector<std::string>& value) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }
  auto& debug_frame = GetThreadDebugFrame();
  if (debug_frame.plan_id() == default_pos) {
    VLOG(2) << "Forbidden to log data in default plan id!";
    return;
  }

  debug::NOADebugInfo_PlanningDebugInfo_VecString vec_string;
  for (const auto& va : value) {
    auto* a_s = vec_string.add_str();
    *a_s = va;
  }
  (*debug_frame.mutable_stringlists())[key] = vec_string;
}

void Log2FG::LogDataV(int level, const std::string& key,
                      const std::vector<double>& value) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }
  auto& debug_frame = GetThreadDebugFrame();
  if (debug_frame.plan_id() == default_pos) {
    VLOG(2) << "Forbidden to log data in default plan id!";
    return;
  }

  debug::NOADebugInfo_PlanningDebugInfo_VecNumber vec_num;
  for (const auto& va : value) {
    vec_num.add_num(va);
  }

  (*debug_frame.mutable_numberlists())[key] = vec_num;
}

void Log2FG::Log2DMarkerV(int level, const std::string& id,
                          const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          const Marker::Color color, const Marker::Type type,
                          const Marker::Propertry properties) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }
  auto& debug_frame = GetThreadDebugFrame();

  if (debug_frame.plan_id() == default_pos) {
    VLOG(2) << "Forbidden to log data in default plan id!";
    return;
  }
  auto* marker = debug_frame.add_markers2d();
  marker->set_id(id);
  marker->set_type(type);
  marker->mutable_color()->MergeFrom(color);
  marker->mutable_properties()->MergeFrom(properties);
  marker->mutable_points()->Clear();

  for (size_t i = 0; i < xs.size(); ++i) {
    auto* point = marker->add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
    point->set_z(0.0);
  }
}

void Log2FG::Log3DMarkerV(int level, const std::string& id,
                          const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          const std::vector<double>& zs,
                          const Marker::Color color,
                          const Marker3D::Type3D type,
                          const Marker::Propertry properties) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }
  auto& debug_frame = GetThreadDebugFrame();

  if (debug_frame.plan_id() == default_pos) {
    VLOG(2) << "Forbidden to log data in default plan id!";
    return;
  }
  auto* marker = debug_frame.add_markers3d();
  marker->set_id(id);
  marker->set_type(type);
  marker->mutable_color()->MergeFrom(color);
  marker->mutable_properties()->MergeFrom(properties);
  marker->mutable_points()->Clear();
  for (size_t i = 0; i < xs.size(); ++i) {
    auto* point = marker->add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
    point->set_z(zs[i]);
  }
}

void Log2FG::Info2DMarkerV(int level, const std::string& id,
                           const std::vector<double>& xs,
                           const std::vector<double>& ys,
                           const Marker::Color color, const Marker::Type type,
                           const Marker::Propertry properties) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }
  std::cout << "Info -> 2D Mv";
  auto* marker = NOA_DEBUG_INFO.add_markers2d();
  marker->set_id(id);
  marker->set_type(type);
  marker->mutable_color()->MergeFrom(color);
  marker->mutable_properties()->MergeFrom(properties);
  marker->mutable_points()->Clear();

  for (size_t i = 0; i < xs.size(); ++i) {
    auto* point = marker->add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
    point->set_z(0.0);
  }
}

void Log2FG::Info3DMarkerV(int level, const std::string& id,
                           const std::vector<double>& xs,
                           const std::vector<double>& ys,
                           const std::vector<double>& zs,
                           const Marker::Color color,
                           const Marker3D::Type3D type,
                           const Marker::Propertry properties) {
  if (level > FLAGS_Log2FG_verbosity_level) {
    return;
  }

  auto* marker = NOA_DEBUG_INFO.add_markers3d();
  marker->set_id(id);
  marker->set_type(type);
  marker->mutable_color()->MergeFrom(color);
  marker->mutable_properties()->MergeFrom(properties);
  marker->mutable_points()->Clear();
  for (size_t i = 0; i < xs.size(); ++i) {
    auto* point = marker->add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
    point->set_z(zs[i]);
  }
}

}  // namespace planning
}  // namespace e2e_noa
