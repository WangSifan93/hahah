#include "apps/planning/src/on_lane_planning.h"
#include "apps/planning/src/reference_line_provider/local_route_provider.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

const double kTol = 1.0e-6;

class OnLanePlanningTest : public ::testing::Test {
 public:
  void SetUp() override {
    vehicle_state_.set_x(0.0);
    vehicle_state_.set_y(1.0);
    vehicle_state_.set_z(2.0);
    vehicle_state_.set_roll(0.1);
    vehicle_state_.set_pitch(0.2);
    vehicle_state_.set_yaw(0.3);
    vehicle_state_.set_linear_velocity(3.0);
    vehicle_state_.set_heading(M_PI / 2.0);
    vehicle_state_.set_timestamp(4.0);
    vehicle_state_.set_kappa(0.4);
    vehicle_state_.set_angular_velocity(0.5);
    vehicle_state_.set_linear_acceleration(0.6);
    vehicle_state_.set_front_wheel_angle(1.25);
    vehicle_state_.set_front_wheel_angle_rate(2.5);
    vehicle_param_.set_steer_ratio(16);
    vehicle_param_.set_max_steer_angle(8.20304748437);
  }

 protected:
  std::unique_ptr<OnLanePlanning> on_lane_planning_ptr;
  common::VehicleState vehicle_state_;
  common::VehicleParam vehicle_param_;
};

TEST_F(OnLanePlanningTest, TestComputeTrajectoryPointFromVehicleState) {
  const double dt_prev = 0.1;
  ::common::TrajectoryPoint tp =
      on_lane_planning_ptr->ComputeTrajectoryPointFromVehicleState(
          dt_prev, vehicle_state_, vehicle_param_);

  EXPECT_NEAR(tp.path_point().x(), vehicle_state_.x(), kTol);
  EXPECT_NEAR(tp.path_point().y(), vehicle_state_.y(), kTol);
  EXPECT_NEAR(tp.path_point().z(), vehicle_state_.z(), kTol);
  EXPECT_NEAR(tp.path_point().theta(), vehicle_state_.heading(), kTol);
  EXPECT_NEAR(tp.path_point().kappa(), vehicle_state_.kappa(), kTol);
  EXPECT_NEAR(tp.path_point().s(), 0.0, kTol);
  EXPECT_NEAR(tp.path_point().dkappa(), 0.0, kTol);
  EXPECT_NEAR(tp.path_point().ddkappa(), 0.0, kTol);

  EXPECT_NEAR(tp.v(), vehicle_state_.linear_velocity(), kTol);
  EXPECT_NEAR(tp.a(), vehicle_state_.linear_acceleration(), kTol);
  EXPECT_NEAR(tp.dt_prev(), 0.1, kTol);
  EXPECT_NEAR(tp.relative_time(), 0.0, kTol);
  EXPECT_NEAR(tp.steer(), vehicle_state_.front_wheel_angle(), kTol);
  EXPECT_NEAR(tp.steer_rate(), vehicle_state_.front_wheel_angle_rate(), kTol);
}

TEST_F(OnLanePlanningTest, TestComputePlanningStartPoint) {
  const double t_curr = 0.1;
  PublishableTrajectory traj_prev(0.0, DiscretizedTrajectory());
  const double dt = 0.2;
  const double v = 10.0;
  for (int k = 0; k < 10; ++k) {
    ::common::PathPoint pp(1.0, k * dt * v, 0.0,      // x, y, z
                           M_PI / 2.0, 0.0,           // theta, kappa
                           k * dt * v, 0.0, 0.0,      // s, dkappa, ddkappa
                           "", 0.0, 0.0);             // lane_id, x_der, y_der
    ::common::TrajectoryPoint tp(pp, v, 0.0, k * dt,  // path_pt, v, a, t,
                                 0.0, 0.0,            // da, steer
                                 GaussianInfo());
    traj_prev.AppendTrajectoryPoint(tp);
  }
  ::common::TrajectoryPoint tp =
      on_lane_planning_ptr->ComputePlanningStartPoint(t_curr, traj_prev,
                                                      vehicle_state_);
  EXPECT_NEAR(tp.path_point().x(), 1.0, kTol);
  EXPECT_NEAR(tp.path_point().y(), 1.0, kTol);
  EXPECT_NEAR(tp.path_point().z(), 0.0, kTol);
  EXPECT_NEAR(tp.path_point().theta(), M_PI / 2, kTol);
  EXPECT_NEAR(tp.path_point().kappa(), 0.0, kTol);
  EXPECT_NEAR(tp.path_point().s(), 0.0, kTol);
  EXPECT_NEAR(tp.path_point().dkappa(), 0.0, kTol);
  EXPECT_NEAR(tp.path_point().ddkappa(), 0.0, kTol);

  EXPECT_NEAR(tp.v(), 10.0, kTol);
  EXPECT_NEAR(tp.a(), 0.0, kTol);
  EXPECT_NEAR(tp.relative_time(), 0.0, kTol);
  EXPECT_NEAR(tp.steer(), 0.0, kTol);
  EXPECT_NEAR(tp.steer_rate(), 0.0, kTol);
}

}  // namespace planning
}  // namespace zark
