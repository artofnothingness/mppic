cmake_minimum_required(VERSION 3.5)
project(mppic)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

include(cmake/conan.cmake)

set(packages 
  rclcpp 
  pluginlib 
  tf2 
  geometry_msgs 
  visualization_msgs
  nav_msgs 
  nav2_core 
  nav2_costmap_2d 
  nav2_util
)

foreach(pkg IN LISTS packages)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

add_subdirectory(src)

ament_export_libraries(mppic)
ament_export_dependencies(${packages})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)

add_subdirectory(test)
add_subdirectory(benchmarks)

install(TARGETS mppic optimizer-benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)


ament_package()
