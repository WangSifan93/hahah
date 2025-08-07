#include "apps/planning/src/decision/longitudinal/st_topology_builder.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/common/speed/st_point.h"
#include "gtest/gtest.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

using namespace zark::planning;

class STTopologyBuilderTest : public ::testing::Test {
 public:
  STTopologyBuilderTest() {
    LongitudinalDeciderConfig::STTopologyConfig config;
    config.alpha = 0.33;
    st_topology_builder_ = std::make_unique<STTopologyBuilder>(config);
  }
  STBoundary CreatObject(double v, double s_init, double t_init, double t_end,
                         double length) {
    STBoundary obj;
    STPoint point1(s_init, t_init);
    STPoint point2(s_init + (t_end - t_init) * v, t_end);
    std::vector<STPoint> lower_points;
    lower_points.push_back(point1);
    lower_points.push_back(point2);
    STPoint point3(s_init + length, t_init);
    STPoint point4(s_init + length + (t_end - t_init) * v, t_end);
    std::vector<STPoint> upper_points;
    upper_points.push_back(point3);
    upper_points.push_back(point4);
    obj = STBoundary::CreateInstanceAccurate(lower_points, upper_points, " 0");
    return obj;
  }

 protected:
  std::unique_ptr<STTopologyBuilder> st_topology_builder_;
};

TEST_F(STTopologyBuilderTest, TestBuildDAG) {
  std::vector<STBoundary> st_graph;
  double v_extrap = 0.0;
  STBoundary obj1, obj2, obj3;
  obj1 = CreatObject(5.0, 35.0, 3.0, 5.0, 10.0);
  obj1.set_id("1");
  obj2 = CreatObject(5.0, -10.0, 6.0, 8.0, 10.0);
  obj2.set_id("2");
  obj3 = CreatObject(7.0, 35.0, 5.5, 8.0, 10.0);
  obj3.set_id("3");
  st_graph.push_back(obj1);
  st_graph.push_back(obj2);
  st_graph.push_back(obj3);
  std::unordered_map<const STBoundary*, std::vector<const STBoundary*>> dag =
      st_topology_builder_->BuildDAG(st_graph, v_extrap);
  EXPECT_EQ(dag.size(), 3);
}

TEST_F(STTopologyBuilderTest, TestTopologicalSort) {
  std::vector<STBoundary> st_graph;
  double v_extrap = 0.0;
  STBoundary obj1, obj2, obj3;
  obj1 = CreatObject(5.0, 35.0, 3.0, 5.0, 10.0);
  obj1.set_id("1");
  obj2 = CreatObject(5.0, -10.0, 6.0, 8.0, 10.0);
  obj2.set_id("2");
  obj3 = CreatObject(7.0, 35.0, 5.5, 8.0, 10.0);
  obj3.set_id("3");
  st_graph.push_back(obj1);
  st_graph.push_back(obj2);
  st_graph.push_back(obj3);
  const std::unordered_map<const STBoundary*, std::vector<const STBoundary*>>
      dag = st_topology_builder_->BuildDAG(st_graph, v_extrap);
  STTopology st_topologys = st_topology_builder_->TopologicalSort(dag);
  EXPECT_EQ(st_topologys[0][0]->id(), "1");
  EXPECT_EQ(st_topologys[0][1]->id(), "3");
  EXPECT_EQ(st_topologys[1][0]->id(), "2");
}

TEST_F(STTopologyBuilderTest, TestIsValidEdge) {
  std::vector<STBoundary> st_graph;
  double v_extrap = 0.0;
  STBoundary obj1, obj2, obj3;
  obj1 = CreatObject(5.0, 35.0, 3.0, 5.0, 10.0);
  obj1.set_id("1");
  obj2 = CreatObject(5.0, -10.0, 6.0, 8.0, 10.0);
  obj2.set_id("2");
  st_graph.push_back(obj1);
  st_graph.push_back(obj2);
  const STBoundary& node_1 = st_graph.front();
  const STBoundary& node_2 = st_graph.back();
  bool status = st_topology_builder_->IsValidEdge(node_1, node_2, v_extrap);
  EXPECT_EQ(status, true);
}

TEST_F(STTopologyBuilderTest, TestInterpolation) {
  std::vector<STBoundary> st_graph;
  STBoundary obj1, obj2, obj3;
  obj1 = CreatObject(5.0, 35.0, 3.0, 5.0, 10.0);
  obj1.set_id("1");

  st_graph.push_back(obj1);

  double t_curr = 6.0;
  std::vector<STPoint> upper_points = st_graph.front().upper_points();
  double inter_point =
      st_topology_builder_->Interpolation(t_curr, upper_points);
  EXPECT_NEAR(inter_point, 55.0, 1e-2);
}

}  // namespace planning
}  // namespace zark
