#ifndef ST_PLANNING_COMMON_LOG_DATA
#define ST_PLANNING_COMMON_LOG_DATA
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <functional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "math/geometry/polygon2d.h"
#include "math/vec2d.h"
#include "messages/planning/driving/noa_debug_info.pb.h"

DECLARE_int32(Log2FG_verbosity_level);
namespace e2e_noa {
namespace planning {

#define ST_LOG_DATA_DEF(level)                                          \
  template <typename T>                                                 \
  static void LogDataV##level(const std::string& key, const T& value) { \
    Log2FG::Get().LogDataV(level, key, value);                          \
  }

#define ST_LOG_MARKER_DEF(level)                                            \
  template <typename T>                                                     \
  static void Log2DMarkerV##level(                                          \
      const std::string& id, const std::vector<T>& xs,                      \
      const std::vector<T>& ys,                                             \
      const zark::e2e_noa::debug::Marker::Color color,                      \
      const zark::e2e_noa::debug::Marker::Type type =                       \
          zark::e2e_noa::debug::Marker_Type_LINE,                           \
      const zark::e2e_noa::debug::Marker::Propertry properties =            \
          zark::e2e_noa::debug::Marker::Propertry()) {                      \
    Log2FG::Get().Log2DMarkerV(level, id, xs, ys, color, type, properties); \
  }

#define ST_INFO_MARKER_DEF(level)                                            \
  template <typename T>                                                      \
  static void Info2DMarkerV##level(                                          \
      const std::string& id, const std::vector<T>& xs,                       \
      const std::vector<T>& ys,                                              \
      const zark::e2e_noa::debug::Marker::Color color,                       \
      const zark::e2e_noa::debug::Marker::Type type =                        \
          zark::e2e_noa::debug::Marker_Type_LINE,                            \
      const zark::e2e_noa::debug::Marker::Propertry properties =             \
          zark::e2e_noa::debug::Marker::Propertry()) {                       \
    Log2FG::Get().Info2DMarkerV(level, id, xs, ys, color, type, properties); \
  }

#define ST_LOG_MARKER_3D_DEF(level)                                    \
  template <typename T>                                                \
  static void Log3DMarkerV##level(                                     \
      const std::string& id, const std::vector<T>& xs,                 \
      const std::vector<T>& ys, const std::vector<T>& zs,              \
      const zark::e2e_noa::debug::Marker::Color color,                 \
      const zark::e2e_noa::debug::Marker3D::Type3D type =              \
          zark::e2e_noa::debug::Marker3D_Type3D::Marker3D_Type3D_LINE, \
      const zark::e2e_noa::debug::Marker::Propertry properties =       \
          zark::e2e_noa::debug::Marker::Propertry()) {                 \
    Log2FG::Get().Log3DMarkerV(level, id, xs, ys, zs, color, type,     \
                               properties);                            \
  }

#define ST_INFO_MARKER3D_DEF(level)                                    \
  template <typename T>                                                \
  static void Info3DMarkerV##level(                                    \
      const std::string& id, const std::vector<T>& xs,                 \
      const std::vector<T>& ys, const std::vector<T>& zs,              \
      const zark::e2e_noa::debug::Marker::Color color,                 \
      const zark::e2e_noa::debug::Marker3D::Type3D type =              \
          zark::e2e_noa::debug::Marker3D_Type3D::Marker3D_Type3D_LINE, \
      const zark::e2e_noa::debug::Marker::Propertry properties =       \
          zark::e2e_noa::debug::Marker::Propertry()) {                 \
    Log2FG::Get().Info3DMarkerV(level, id, xs, ys, zs, color, type,    \
                                properties);                           \
  }

#define ST_LOG_ALL_LEVEL_DEF(ST_MACRO_NAME) \
  ST_MACRO_NAME(0)                          \
  ST_MACRO_NAME(1)                          \
  ST_MACRO_NAME(2)                          \
  ST_MACRO_NAME(3)                          \
  ST_MACRO_NAME(4)                          \
  ST_MACRO_NAME(5)

#define MAKE_COLOR(r, g, b, alpha)             \
  []() {                                       \
    zark::e2e_noa::debug::Marker::Color color; \
    color.set_r(r);                            \
    color.set_g(g);                            \
    color.set_b(b);                            \
    color.set_alpha(alpha);                    \
    return color;                              \
  }()

namespace debug {
using namespace zark::e2e_noa::debug;

Marker::Point VectorToPoint(const e2e_noa::Vec2d& vec);

void ConvertVectorsToMarkerPoints(Marker* marker,
                                  const std::vector<e2e_noa::Vec2d>& vectors);
void VectorsToMarker(Marker* marker, std::string marker_id,
                     const std::vector<e2e_noa::Vec2d>& vectors,
                     Marker::Type type, Marker::Color color,
                     bool is_closed = false);

void PolygonToMarker(Marker* marker, std::string marker_id,
                     const e2e_noa::Polygon2d& polygon, Marker::Color color,
                     bool is_closed = true, Marker::Type = Marker::POLYGON);
}  // namespace debug

class Log2FG {
 public:
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_DATA_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_MARKER_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_MARKER_3D_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_INFO_MARKER_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_INFO_MARKER3D_DEF)

  using Color = zark::e2e_noa::debug::Marker::Color;
  using Propertry = zark::e2e_noa::debug::Marker::Propertry;
  using Marker = zark::e2e_noa::debug::Marker;
  using Marker3D = zark::e2e_noa::debug::Marker3D;
  static const Color kBlack;
  static const Color kWhite;
  static const Color kRed;
  static const Color kGreen;
  static const Color kBlue;
  static const Color kYellow;
  static const Color kOrange;
  static const Color kPink;
  static const Color kPurple;
  static const Color kBrown;
  static const Color kGray;
  static const Color kHotpink;
  static const Color kCoral;
  static const Color kDarkkhaki;
  static const Color kViolet;
  static const Color kLime;
  static const Color kAqua;
  static const Color kMagenta;
  static const Color kDarkRed;
  static const Color kLightGray;
  static const Color kMiddleBlueGreen;
  static const Color kTiffanyBlue;
  static const Color kLightBlue;
  static const Color kDarkGreen;
  static const Color kDarkBlue;
  static const Color kGrassGreen;

  static Log2FG& Get() {
    static Log2FG instance;
    return instance;
  }

  static std::string TaskPrefix(int i) {
    return "task" + std::to_string(i) + "_";
  }
  zark::e2e_noa::debug::NOADebugInfo_PlanningDebugInfo& GetThreadDebugFrame(
      const std::string plan_id = "");

  zark::e2e_noa::debug::NOADebugInfo::ChooseLaneDebugInfo::PlanPassage&
  GetPlanPassage(std::string plan_id);

  static void LogMissionData(const std::string& key, const std::string& value);

  static void LogVehData(const std::string& key, const std::string& value);

  zark::e2e_noa::debug::NOADebugInfo& getDebugInfo() { return debug_info_; }

  void clear() {
    debug_info_.Clear();
    thread_plan_id_map_.clear();
  }

  Log2FG(const Log2FG&) = delete;
  Log2FG& operator=(const Log2FG&) = delete;

 private:
  void LogDataV(int level, const std::string& key, const std::string& value);
  void LogDataV(int level, const std::string& key, double value);
  void LogDataV(int level, const std::string& key,
                const std::vector<std::string>& value);
  void LogDataV(int level, const std::string& key,
                const std::vector<double>& value);

  void Log2DMarkerV(
      int level, const std::string& id, const std::vector<double>& xs,
      const std::vector<double>& ys, const Marker::Color color,
      const Marker::Type type = zark::e2e_noa::debug::Marker_Type_LINE,
      const Marker::Propertry properties = Marker::Propertry());

  void Log3DMarkerV(
      int level, const std::string& id, const std::vector<double>& xs,
      const std::vector<double>& ys, const std::vector<double>& zs,
      const Marker::Color color,
      const Marker3D::Type3D type =
          zark::e2e_noa::debug::Marker3D_Type3D::Marker3D_Type3D_LINE,
      const Marker::Propertry properties = Marker::Propertry());

  void Info2DMarkerV(
      int level, const std::string& id, const std::vector<double>& xs,
      const std::vector<double>& ys, const Marker::Color color,
      const Marker::Type type = zark::e2e_noa::debug::Marker_Type_LINE,
      const Marker::Propertry properties = Marker::Propertry());
  void Info3DMarkerV(
      int level, const std::string& id, const std::vector<double>& xs,
      const std::vector<double>& ys, const std::vector<double>& zs,
      const Marker::Color color,
      const Marker3D::Type3D type =
          zark::e2e_noa::debug::Marker3D_Type3D::Marker3D_Type3D_LINE,
      const Marker::Propertry properties = Marker::Propertry());
  Log2FG() {}

  zark::e2e_noa::debug::NOADebugInfo debug_info_{};

  std::shared_mutex plan_passage_mutex_;
  std::shared_mutex mutex_;

  std::unordered_map<std::string, std::string> thread_plan_id_map_;

  std::string default_pos = "Default";
};

}  // namespace planning
}  // namespace e2e_noa

#define NOA_DEBUG_CLEAR e2e_noa::planning::Log2FG::Get().clear()
#define NOA_DEBUG_INFO e2e_noa::planning::Log2FG::Get().getDebugInfo()
#define NOA_PLANPASSAGE_DEBUG_INFO(id) \
  e2e_noa::planning::Log2FG::Get().GetPlanPassage(id)

#define NOA_DebugFrameCreate(id) \
  e2e_noa::planning::Log2FG::Get().GetThreadDebugFrame(id)
#define NOA_PLAN_DEBUG e2e_noa::planning::Log2FG::Get().GetThreadDebugFrame()
#define NOA_SEARCH_DEBUG NOA_PLAN_DEBUG.mutable_spatio_search_debug()
#define NOA_OPTIM_DEBUG NOA_PLAN_DEBUG.mutable_spaito_optimization_debug()
#define NOA_SPEED_DEBUG NOA_PLAN_DEBUG.mutable_speed_finder_debug()

namespace ad_e2e {
namespace planning {
using Log2FG = e2e_noa::planning::Log2FG;
}
}  // namespace ad_e2e

#endif
