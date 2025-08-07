#include "apps/planning/src/common/lookup_table.h"

#include <fstream>
#include <iostream>
#include <string>

#include "apps/planning/src/config/config_main.h"
#include "gtest/gtest.h"

using namespace zark::planning;
using namespace std;

TEST(LookupTableTest, basic_test) {
  LookupTableConfig config;
  std::vector<::common::Point2D> points;
  std::vector<double> points_x;
  std::vector<double> points_y;
  for (double i = 0.0; i < 10.0; i = i + 1.0) {
    double y = i;
    points_x.emplace_back(i);
    points_y.emplace_back(y);
  }
  config.points_x = points_x;
  config.points_y = points_y;
  LookupTable table(config);
  double res = table.Evaluate(3.2);
  EXPECT_DOUBLE_EQ(res, 3.2);
}

TEST(LookupTableTest, one_point_test) {
  LookupTableConfig config;
  std::vector<double> points_x;
  std::vector<double> points_y;
  points_x.emplace_back(1.0);
  points_y.emplace_back(2.0);
  config.points_x = points_x;
  config.points_y = points_y;
  LookupTable table(config);
  double res = table.Evaluate(0.5);
  EXPECT_DOUBLE_EQ(res, 2.0);
}

TEST(LookupTableTest, two_point_test2) {
  LookupTableConfig config;
  std::vector<double> points_x;
  std::vector<double> points_y;
  points_x.emplace_back(0.5);
  points_y.emplace_back(2.0);
  points_x.emplace_back(1.0);
  points_y.emplace_back(3.0);
  config.points_x = points_x;
  config.points_y = points_y;
  LookupTable table(config);
  double res = table.Evaluate(0.8);
  EXPECT_DOUBLE_EQ(res, 2.6);
}

TEST(LookupTableTest, three_point_test2) {
  LookupTableConfig config;
  std::vector<double> points_x;
  std::vector<double> points_y;
  points_x.emplace_back(0.5);
  points_y.emplace_back(2.0);
  points_x.emplace_back(1.0);
  points_y.emplace_back(3.0);
  points_x.emplace_back(4.0);
  points_y.emplace_back(-1.0);
  config.points_x = points_x;
  config.points_y = points_y;
  LookupTable table(config);
  double res = table.Evaluate(2.0);
  std::cout << " evaluate 0.8 = " << res << std::endl;
}

TEST(LookupTableTest, load_file_test) {
  LookupTableConfig config;
  string config_file =
      "/zark/apps/planning/test/config/json/lookup_table_test.json";
  Config table_config{config_file};
  std::ifstream jdata(config_file);
  nlohmann::json jnode = nlohmann::json::parse(jdata);
  std::string config_type = "table_config";
  config = table_config.GetLookupTableConfig(jnode.at(config_type));
  LookupTable table(config);
  double res = table.Evaluate(0.8);
  EXPECT_DOUBLE_EQ(res, 1.16);
  std::cout << " evaluate 0.8 = " << res << std::endl;
}
