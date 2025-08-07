#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/common/speed/st_boundary.h"
#include "apps/planning/src/common/speed/st_point.h"
#include "apps/planning/src/decision//longitudinal/data_type.h"
#include "apps/planning/src/decision//longitudinal/st_proposal_builder.h"
#include "apps/planning/src/decision/longitudinal/st_topology_builder.h"
#include "gtest/gtest.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

using namespace zark::planning;

class STProposalBuilderTest : public ::testing::Test {
 public:
  STProposalBuilderTest() {
    v_extrap_ = std::make_unique<double>(13.888888889);

    // construct STGraph
    std::vector<std::pair<STPoint, STPoint>> st_pairs_1;
    st_pairs_1.clear();
    STPoint p1_1(-40.181814488475, -0.33487631700000003);
    STPoint p1_2(-30.750281985012617, -0.33487631700000003);
    STPoint p1_3(71.95324347216493, 8.165123682999985);
    STPoint p1_4(81.41573809330089, 8.165123682999985);
    st_pairs_1.emplace_back(std::make_pair(p1_1, p1_2));
    st_pairs_1.emplace_back(std::make_pair(p1_3, p1_4));
    STBoundary st_boundary_1(st_pairs_1, "174_0");

    std::vector<std::pair<STPoint, STPoint>> st_pairs_2;
    st_pairs_2.clear();
    STPoint p2_1(61.41474977874999, -0.33487631700000003);
    STPoint p2_2(71.1020687768809, -0.33487631700000003);
    STPoint p2_3(197.1473461471152, 8.165123682999985);
    STPoint p2_4(206.7715990497941, 8.165123682999985);
    st_pairs_2.emplace_back(std::make_pair(p2_1, p2_2));
    st_pairs_2.emplace_back(std::make_pair(p2_3, p2_4));
    STBoundary st_boundary_2(st_pairs_2, "175_0");

    std::vector<std::pair<STPoint, STPoint>> st_pairs_3;
    st_pairs_3.clear();
    STPoint p3_1(58.33785152395119, 3.0651236830000017);
    STPoint p3_2(67.91724699559141, 3.0651236830000017);
    STPoint p3_3(151.19954568544267, 8.165123682999985);
    STPoint p3_4(160.70399668234296, 8.165123682999985);
    st_pairs_3.emplace_back(std::make_pair(p3_1, p3_2));
    st_pairs_3.emplace_back(std::make_pair(p3_3, p3_4));
    STBoundary st_boundary_3(st_pairs_3, "176_0");

    st_graph_ = std::make_unique<STGraph>();
    st_graph_->clear();
    st_graph_->emplace_back(st_boundary_1);
    st_graph_->emplace_back(st_boundary_2);
    st_graph_->emplace_back(st_boundary_3);

    // construct STTopologyBuilder
    LongitudinalDeciderConfig::STTopologyConfig sttopo_config;
    sttopo_config.alpha = 0.33;
    st_topology_builder_ = std::make_unique<STTopologyBuilder>(sttopo_config);

    // construct STProposalBuilder
    LongitudinalDeciderConfig::STProposalConfig config;
    config.max_num_proposals = 2;
    config.a_min_stiff = -6.0;
    config.w_front = 1.0;
    config.w_rear = 1.0;
    config.cost_max = 20.0;
    config.t_start_blocking_max = 4.0;
    config.a_decel_limit = -4.5;

    std::vector<double> points_x;
    points_x.clear();
    std::vector<double> points_y;
    points_y.clear();
    points_x.push_back(0.0);
    points_x.push_back(10.0);
    points_y.push_back(10.0);
    points_y.push_back(0.0);
    config.narrow_gap_cost_table.points_x = points_x;
    config.narrow_gap_cost_table.points_y = points_y;

    points_x.clear();
    points_y.clear();
    points_x.push_back(5.0);
    points_x.push_back(10.0);
    points_x.push_back(20.0);
    points_x.push_back(30.0);
    points_y.push_back(2.5);
    points_y.push_back(1.2);
    points_y.push_back(0.6);
    points_y.push_back(0.3);
    config.a_acc_max_table.points_x = points_x;
    config.a_acc_max_table.points_y = points_y;

    st_proposal_builder_ = std::make_unique<STProposalBuilder>(config);
  }

 protected:
  std::unique_ptr<double> v_extrap_;
  std::unique_ptr<STGraph> st_graph_;
  std::unique_ptr<STTopologyBuilder> st_topology_builder_;
  std::unique_ptr<STProposalBuilder> st_proposal_builder_;
};

TEST_F(STProposalBuilderTest, TestGenerateSTProposals) {
  CorridorInfo::Type type = CorridorInfo::Type::LANE_KEEP;
  // CorridorInfo::Type type = CorridorInfo::Type::LANE_CHANGE;

  STTopology st_topology =
      st_topology_builder_->BuildSTTopology(*st_graph_, *v_extrap_);
  EXPECT_EQ(st_topology.size(), 3);
  EXPECT_EQ(st_topology[0][0]->id(), "175_0");
  EXPECT_EQ(st_topology[1][0]->id(), "176_0");
  EXPECT_EQ(st_topology[2][0]->id(), "174_0");

  std::vector<STProposal> st_proposals =
      st_proposal_builder_->GenerateSTProposals(st_topology, type);
  EXPECT_EQ(st_proposals.size(), 4);
}

TEST_F(STProposalBuilderTest, TestSortAndFilterSTProposals) {
  CorridorInfo::Type type = CorridorInfo::Type::LANE_KEEP;
  STTopology st_topology =
      st_topology_builder_->BuildSTTopology(*st_graph_, *v_extrap_);

  std::vector<STProposal> st_proposals =
      st_proposal_builder_->GenerateSTProposals(st_topology, type);
  EXPECT_EQ(st_proposals.size(), 4);

  std::vector<std::pair<STProposal, double>> cost_proposals;
  std::unordered_map<const STBoundary*, double> cost_map_front;
  std::unordered_map<const STBoundary*, double> cost_map_rear;
  st_proposal_builder_->SortAndFilterSTProposals(st_topology, *v_extrap_,
                                                 cost_map_front, cost_map_rear,
                                                 cost_proposals, st_proposals);

  EXPECT_EQ(st_proposals.size(), 1);

  EXPECT_EQ(st_proposals.at(0).size(), 3);
  EXPECT_EQ(std::get<1>(st_proposals.at(0).at(0)), true);
  EXPECT_EQ(std::get<1>(st_proposals.at(0).at(1)), true);
  EXPECT_EQ(std::get<1>(st_proposals.at(0).at(2)), false);

  EXPECT_EQ(cost_proposals.size(), 1);
  EXPECT_EQ(cost_proposals.at(0).second, 0.0);
}

TEST_F(STProposalBuilderTest, TestComputeAccelMap) {
  STTopology st_topology =
      st_topology_builder_->BuildSTTopology(*st_graph_, *v_extrap_);
  std::unordered_map<const STBoundary*, double> accel_map_front;
  std::unordered_map<const STBoundary*, double> accel_map_rear;
  st_proposal_builder_->ComputeAccelMap(st_topology, *v_extrap_,
                                        accel_map_front, accel_map_rear);

  EXPECT_EQ(st_topology.size(), 3);
  EXPECT_EQ(st_topology[0][0]->id(), "175_0");
  EXPECT_EQ(st_topology[1][0]->id(), "176_0");
  EXPECT_EQ(st_topology[2][0]->id(), "174_0");

  EXPECT_EQ(accel_map_front[st_topology[0][0]], 0.0);
  EXPECT_EQ(accel_map_front[st_topology[1][0]], 0.0);
  EXPECT_EQ(accel_map_front[st_topology[2][0]],
            -std::numeric_limits<double>::infinity());

  EXPECT_EQ(accel_map_rear[st_topology[0][0]],
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(accel_map_rear[st_topology[1][0]], 5.3956636671172626);
  EXPECT_EQ(accel_map_rear[st_topology[2][0]], 0.0);
}

TEST_F(STProposalBuilderTest, TestComputeAccel) {
  double s_ego = 0.0;

  // 176_0
  double o1_acc_front = st_proposal_builder_->ComputeAccel(
      s_ego, *v_extrap_, 58.33785152395119, 3.0651236830000017);
  double o1_acc_rear = st_proposal_builder_->ComputeAccel(
      s_ego, *v_extrap_, 67.91724699559141, 3.0651236830000017);

  EXPECT_EQ(o1_acc_front, 3.356406059991782);
  EXPECT_EQ(o1_acc_rear, 5.3956636671172626);
}

TEST_F(STProposalBuilderTest, TestComputeMinumumGap) {
  CorridorInfo::Type type = CorridorInfo::Type::LANE_KEEP;
  STTopology st_topology =
      st_topology_builder_->BuildSTTopology(*st_graph_, *v_extrap_);

  std::vector<STProposal> st_proposals =
      st_proposal_builder_->GenerateSTProposals(st_topology, type);
  EXPECT_EQ(st_proposals.size(), 4);

  std::vector<std::pair<STProposal, double>> cost_proposals;
  std::unordered_map<const STBoundary*, double> cost_map_front;
  std::unordered_map<const STBoundary*, double> cost_map_rear;
  st_proposal_builder_->SortAndFilterSTProposals(st_topology, *v_extrap_,
                                                 cost_map_front, cost_map_rear,
                                                 cost_proposals, st_proposals);

  double gap = st_proposal_builder_->ComputeMinumumGap(st_proposals.at(0));
  EXPECT_EQ(gap, std::numeric_limits<double>::infinity());
}

}  // namespace planning
}  // namespace zark
