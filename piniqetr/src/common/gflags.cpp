#include "common/gflags.h"

DEFINE_int32(ad_e2e_planning_log_level, 0,
             "log level, 0 = DEBUG, 1 = INFO, 2 = WARN, 3 = ERROR, 4 = FATAL");

DEFINE_int32(ad_e2e_prediction_pool_size, 10,
             "The number of threads in the default thread pool for prediction");
DEFINE_int32(
    ad_e2e_city_planner_pool_size, 15,
    "The number of threads in the default thread pool for city planner");

DEFINE_int32(ad_e2e_planning_future_wait_timeout, 60,
             "time for future get timeout in second");
DEFINE_double(ad_e2e_planning_zero_threshold, 1e-6,
              "zero threshold for ad_e2e_planning");

DEFINE_double(ad_e2e_planning_map_search_heading_limit, 1.04719755,
              "heading limit for search lane by point");
DEFINE_double(ad_e2e_planning_map_minimum_boundary_search_radius, 3.5,
              "minimun search radius for searching lane");
DEFINE_double(ad_e2e_planning_map_point_distance_threshold, 0.01,
              "skip map point by distance (between two map points)");
DEFINE_double(ad_e2e_planning_map_boundary_gap_threshold, 1.0,
              "distance of gap between two boundaries");
DEFINE_double(ad_e2e_planning_boundary_interpolate_dist, 1.0,
              "The distance for boundary interpolate");

DEFINE_double(
    ad_e2e_planning_lane_polyline_vector_dist, 2.0,
    "The distance between start point with end point of lane/boundary vector");
DEFINE_uint32(ad_e2e_planning_lane_polyline_vector_num, 32,
              "Number of lane/boundary polyline vector size");
DEFINE_uint32(ad_e2e_planning_lane_vector_dim, 24, "dimension of lane vector");

DEFINE_double(
    ad_e2e_planning_road_boundary_polyline_vector_dist, 2.0,
    "The distance between start point with end point of road_boundary vector");
DEFINE_uint32(ad_e2e_planning_road_boundary_polyline_vector_num, 32,
              "Number of road_boundary polyline vector size");
DEFINE_uint32(ad_e2e_planning_road_boundary_vector_dim, 16,
              "dimension of road boundary vector");

DEFINE_uint32(ad_e2e_planning_polygon_polyline_vector_num, 32,
              "Number of polygon(crosswalk/junction) polyline vector size");
DEFINE_uint32(ad_e2e_planning_polygon_polyline_vector_dim, 8,
              "dimension of polygon polyline vector");
DEFINE_double(ad_e2e_junction_search_radius, 50,
              "Distance for obstacle junction search radius");
DEFINE_double(ad_e2e_converged_entry_speed, 3.0,
              "Converge speed for obs entering junction");
DEFINE_double(ad_e2e_lane_sequence_unique_distance, 1.0,
              "Distance between lane sequence");
DEFINE_double(ad_e2e_min_juction_lane_length, 2.9, "Min juction lane length");
DEFINE_double(ad_e2e_planning_forward_lane_sequence_length, 150, "m");
DEFINE_double(ad_e2e_planning_backward_lane_sequence_length, 60, "m");

DEFINE_double(ad_e2e_caution_obstacle_max_distance, 100,
              "Maximum distance for caution obstacle ranking");
DEFINE_int32(ad_e2e_planning_obstacle_lru_cache_num, 100,
             "max number of obstacle lru cache size");
DEFINE_int32(ad_e2e_obstacle_lru_cache_num, 100,
             "max number of obstacle lru cache size");
DEFINE_double(ad_e2e_current_lane_search_dist, 20,
              "lane search distance for osbtacle current lane");
DEFINE_int32(ad_e2e_max_dynamic_obstacle_num, 64,
             "max number of dynamic obstacle to predictive");
DEFINE_string(ad_e2e_adc_id, "-1", "autonomous dring car's id");
DEFINE_uint32(ad_e2e_planning_max_obstacle_vector_size, 20,
              "max number of obstacle vector size");
DEFINE_uint32(ad_e2e_planning_dynamic_obstacle_vector_dim, 24,
              "dynamic obstacle vector dimension");
DEFINE_uint32(ad_e2e_planning_static_obstacle_vector_dim, 8,
              "static obstacle vector dimension");

DEFINE_string(ad_e2e_planning_platform, "x86", "platform");
DEFINE_string(ad_e2e_platform_config, "/platform_config.pb.txt",
              "config for platform module");

DEFINE_double(ad_e2e_prediction_predict_period, 6.0,
              "prediction trajectory prediction period");
DEFINE_double(ad_e2e_prediction_trajectory_time_step, 0.1,
              "prediction trajectory time step");

DEFINE_string(ad_e2e_city_config, "config/e2e_pnc/conf",
              "config for city plannner");
DEFINE_string(ad_e2e_vehicle_params_path, "/vehicle_params.pb.txt",
              "vehicle params file suffix");
