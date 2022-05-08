cmake_minimum_required(VERSION 3.5)
project(mppic)

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)

set(dependencies_pkgs
  rclcpp
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

add_library(mppic SHARED
  src/controller.cpp
  src/optimizer_core_interface.cpp
  src/trajectory_visualizer.cpp
  src/path_handler.cpp
  src/parameters_handler.cpp
)

add_library(xtensor-optimizer SHARED
  src/optimizers/xtensor/optimizer.cpp
  src/optimizers/xtensor/critic_manager.cpp
)

add_library(xtensor-critics SHARED
  src/optimizers/xtensor/critics/reference_trajectory_critic.cpp
  src/optimizers/xtensor/critics/goal_angle_critic.cpp
  src/optimizers/xtensor/critics/path_angle_critic.cpp
  src/optimizers/xtensor/critics/goal_critic.cpp
  src/optimizers/xtensor/critics/obstacles_critic.cpp
  src/optimizers/xtensor/critics/prefer_forward_critic.cpp
  src/optimizers/xtensor/critics/twirling_critic.cpp
  src/optimizers/xtensor/critics/local_goal_critic.cpp
)

set(libraries mppic xtensor-critics xtensor-optimizer)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC include)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

target_link_libraries(xtensor-optimizer xtensor)
target_link_libraries(xtensor-critics xtensor)

install(TARGETS mppic xtensor-optimizer xtensor-critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core plugins/mppic.xml)
pluginlib_export_plugin_description_file(mppic plugins/optimizers.xml)
pluginlib_export_plugin_description_file(mppic plugins/xtensor-critics.xml)

ament_package()
