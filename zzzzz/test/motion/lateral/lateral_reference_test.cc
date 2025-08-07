#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/motion/lateral/data_type.h"
#include "apps/planning/src/motion/lateral/lateral_reference.h"
#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

class LateralReferenceTest : public ::testing::Test {
 public:
  void SetUp() override {
    std::string config_file =
        "/zark/apps/planning/test/config/json/"
        "lateral_optimizer_config_test.json";
    Config config{config_file};
    std::ifstream jdata(config_file);
    lateral_config_ =
        config.SetLateralOptimizerConfig(nlohmann::json::parse(jdata));
    ref_line_ptr = std::unique_ptr<LateralReference>(
        new LateralReference(lateral_config_));
  }

 protected:
  LateralOptimizerConfig lateral_config_;
  std::unique_ptr<LateralReference> ref_line_ptr;
};

TEST_F(LateralReferenceTest, ConstructReferenceTrajectory) {
  Tube tube;
  const int num_states = this->lateral_config_.model.num_states;
  const int num_ctrls = this->lateral_config_.model.num_ctrls;
  const int n_steps = this->lateral_config_.model.num_steps;
  const int n_nodes = n_steps + 1;
  for (int i = 0; i < n_nodes; i++) {
    Tube::TubePoint point;
    point.l_left_ref = 1.75;
    point.l_right_ref = 1.75;
    tube.pts.push_back(point);
  }
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ref_line_ =
      std::make_pair(Eigen::MatrixXd::Zero(num_states, n_nodes),
                     Eigen::MatrixXd::Zero(num_ctrls, n_steps));
  ref_line_.first.row(0) = Eigen::RowVectorXd::Constant(n_nodes, 1.75);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ref_line =
      ref_line_ptr->ConstructReferenceTrajectory(tube);
  EXPECT_EQ(ref_line, ref_line_);
}

}  // namespace planning
}  // namespace zark
