#include "apps/planning/src/common/hysteresis.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

TEST(Hysteresis_Test, TestUpdateState) {
  hysteresis::Hysteresis hysteresis_test(3);
  hysteresis_test.InitState(false);
  for (size_t i = 0; i < 2; i++) {
    hysteresis_test.UpdateState(true);
  }
  EXPECT_FALSE(hysteresis_test.GetCurrentState());
  for (size_t i = 0; i < 2; i++) {
    hysteresis_test.UpdateState(true);
  }
  EXPECT_TRUE(hysteresis_test.GetCurrentState());
}
}  // namespace planning
}  // namespace zark