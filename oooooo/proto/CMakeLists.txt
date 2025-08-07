cmake_minimum_required(VERSION 3.10)

project(e2e_pnc_prpto)

set(PROTO_FILES
  common.proto
  dynamic_object.proto
  logic_map.proto
  plan_lane_sequence.proto
  plan_treesearch.proto
  vector_map.proto
  planning_config.proto

  aabox3d.proto
  aabox.proto
  affine_transformation.proto
  arc2d.proto
  autonomy_state.proto
  box2d.proto
  chassis.proto
  constraint.proto
  crosswalk_state.proto
  driving_style.proto
  st_planner_debug.proto
  halfplane.proto
  hmi_content.proto
  initialization_config.proto
  trajectory_initialization.proto
  lane_change.proto
  lane_path.proto
  lane_point.proto
  lite_common.proto
  localization.proto
  optimizer.proto
  perception.proto
  piecewise_const_function.proto
  piecewise_linear_function.proto
  planner_object.proto

  planner_params.proto
  planner.proto
  planner_state.proto
  planner_status.proto
  positioning.proto
  prediction_common.proto
  prediction.proto
  alc.proto

  reference_line_qp_smoother.proto
  route.proto
  route_specific_semantic_map.proto
  scene_understanding.proto
  decision_exploration.proto
  selector_debug.proto
  selector_params.proto
  selector_state.proto
  smooth_reference_line.proto
  speed_planning_params.proto
  speed_planning.proto
  traffic_light_info.proto
  trajectory_point.proto
  trajectory.proto
  trajectory_validation.proto
  turn_signal.proto
  vehicle.proto
  glog.proto
  pnp_info.proto
  behavior.proto
)

include_directories(
    ${CMAKE_INSTALL_PREFIX}/include
    ${Protobuf_INCLUDE_DIRS}
)

# file(GLOB_RECURSE PROTO_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*.proto")
set(PROTO_SOURCE_DIR "${CMAKE_SOURCE_DIR}/pnc_interface/proto")

# Set the directory where generated files will be stored
set(GENERATED_PROTO_DIR "${CMAKE_BINARY_DIR}/pnc_interface/proto")
file(MAKE_DIRECTORY ${GENERATED_PROTO_DIR})

set(GENERATED_PROTO_SRC_FILES)
# List to hold all generated source and header files
set(GENERATED_PROTO_HEADER_FILES)

# Use the project name as the library name
set(PROTO_LIB_NAME ${PROJECT_NAME})

foreach(PROTO_FILE ${PROTO_FILES})
    # Check if the generated files are already up to date
    # if(NOT EXISTS ${GENERATED_SRC_FILE})  #TODO BUG to be fixed, because now only regenerate only total rebuild
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)  # Get the base name without extension

    # Define the output path for the generated files
    # Create subdirectories based on the proto file's location (relative to source)
    get_filename_component(PROTO_DIR ${PROTO_FILE} DIRECTORY)
    if(PROTO_DIR STREQUAL "")
      set(PROTO_DIR ".")
    endif()

    string(REPLACE ${CMAKE_SOURCE_DIR} ${GENERATED_PROTO_DIR} GENERATED_PROTO_SUBDIR ${PROTO_DIR})
    file(MAKE_DIRECTORY ${GENERATED_PROTO_SUBDIR})

    set(GENERATED_SRC_FILE "${GENERATED_PROTO_DIR}/${GENERATED_PROTO_SUBDIR}/${PROTO_NAME}.pb.cc")
    set(GENERATED_HEADER_FILE "${GENERATED_PROTO_SUBDIR}/${PROTO_NAME}.pb.h")

    # Add the protoc command to generate the source and header files if they are outdated or not present
    # message("
    # export LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib/protobuf:$ENV{LD_LIBRARY_PATH} && ${Protobuf_PROTOC_EXECUTABLE} --experimental_allow_proto3_optional ARGS --proto_path=${PROTO_SOURCE_DIR} --proto_path=${Protobuf_INCLUDE_DIRS} --cpp_out=${GENERATED_PROTO_DIR} ${PROTO_FILE}
    # ")

    add_custom_command(
        OUTPUT ${GENERATED_SRC_FILE} ${GENERATED_HEADER_FILE}
        COMMAND export LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib/protobuf:$ENV{LD_LIBRARY_PATH} && ${Protobuf_PROTOC_EXECUTABLE} --experimental_allow_proto3_optional
        ARGS --proto_path=${PROTO_SOURCE_DIR} --proto_path=${Protobuf_INCLUDE_DIRS} --cpp_out=${GENERATED_PROTO_DIR} ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}  # This ensures the command runs only if the .proto file changes
        COMMENT "Generating ${PROTO_NAME}.pb.cc and ${PROTO_NAME}.pb.h from ${PROTO_FILE}"
        VERBATIM
    )
    # else()
    #     message(STATUS "Skipping regeneration of ${PROTO_NAME}.pb.cc and ${PROTO_NAME}.pb.h; files are up to date.")
    # endif()

    # Collect generated files for later use in the library
    list(APPEND GENERATED_PROTO_SRC_FILES ${GENERATED_SRC_FILE})
    list(APPEND GENERATED_PROTO_HEADER_FILES ${GENERATED_HEADER_FILE})
endforeach()

# protobuf_generate_cpp(GENERATED_PROTO_SRC_FILES GENERATED_PROTO_HEADER_FILES ${PROTO_FILES})

add_custom_target(proto_target ALL DEPENDS ${GENERATED_PROTO_SRC_FILES} ${GENERATED_PROTO_HEADER_FILES})

set(LIB_PROTOS "e2e_planning_protos")

add_library(${LIB_PROTOS} SHARED
  ${GENERATED_PROTO_SRC_FILES}
)

add_dependencies(${LIB_PROTOS} proto_target)

target_link_libraries(${LIB_PROTOS}
  PRIVATE ${Protobuf_LIBRARIES}
)
target_include_directories(${LIB_PROTOS} PUBLIC ${GENERATED_PROTO_DIR} ${Protobuf_INCLUDE_DIRS})