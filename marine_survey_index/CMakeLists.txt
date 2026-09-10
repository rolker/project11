cmake_minimum_required(VERSION 3.8)
project(marine_survey_index)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(marine_autonomy REQUIRED)
find_package(marine_sidescan_mosaic REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(SQLite3 REQUIRED)

# Core library: schema, interval accumulator, footprint->tile enumeration, the
# query join, and the bag fingerprint the incremental-skip ledger decides on.
# Deliberately free of rosbag2/tf2/rclcpp so the unit tests exercise the
# correctness paths without bag I/O.
add_library(${PROJECT_NAME}_core
  src/schema.cpp
  src/footprint.cpp
  src/query.cpp
  src/bag_fingerprint.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core SQLite::SQLite3)
ament_target_dependencies(${PROJECT_NAME}_core
  marine_autonomy
  geographic_msgs
)

add_executable(survey_index_bag src/survey_index_bag_main.cpp)
target_link_libraries(survey_index_bag ${PROJECT_NAME}_core)
ament_target_dependencies(survey_index_bag
  rclcpp
  marine_acoustic_msgs
  sensor_msgs
  geodesy
  tf2
  tf2_msgs
  geometry_msgs
  builtin_interfaces
  marine_sidescan_mosaic
  rosbag2_cpp
  rosbag2_storage
)

add_executable(survey_index_query src/survey_index_query_main.cpp)
target_link_libraries(survey_index_query ${PROJECT_NAME}_core)
ament_target_dependencies(survey_index_query
  marine_autonomy
  geographic_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS survey_index_bag survey_index_query
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_schema test/test_schema.cpp)
  target_link_libraries(test_schema ${PROJECT_NAME}_core)

  ament_add_gtest(test_interval_merge test/test_interval_merge.cpp)
  target_link_libraries(test_interval_merge ${PROJECT_NAME}_core)

  ament_add_gtest(test_nav_decimation test/test_nav_decimation.cpp)
  target_link_libraries(test_nav_decimation ${PROJECT_NAME}_core)

  ament_add_gtest(test_tile_enumeration test/test_tile_enumeration.cpp)
  target_link_libraries(test_tile_enumeration ${PROJECT_NAME}_core)

  ament_add_gtest(test_query_join test/test_query_join.cpp)
  target_link_libraries(test_query_join ${PROJECT_NAME}_core)

  ament_add_gtest(test_bag_fingerprint test/test_bag_fingerprint.cpp)
  target_link_libraries(test_bag_fingerprint ${PROJECT_NAME}_core)

  # Runs the real indexer binary: its exit-status contract lives in main(),
  # where no library call can reach it.
  ament_add_gtest(test_indexer_exit_status test/test_indexer_exit_status.cpp)
  target_link_libraries(test_indexer_exit_status ${PROJECT_NAME}_core)
  ament_target_dependencies(test_indexer_exit_status rosbag2_cpp rosbag2_storage)
  target_compile_definitions(test_indexer_exit_status PRIVATE
    SURVEY_INDEX_BAG_EXE=\"$<TARGET_FILE:survey_index_bag>\")
  # $<TARGET_FILE:> expands to a path, not a dependency: without this the test
  # can build and run against a stale binary (or none). Same pairing as the
  # three existing binary-running tests in this repo.
  add_dependencies(test_indexer_exit_status survey_index_bag)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(marine_autonomy geographic_msgs SQLite3)
ament_package()
