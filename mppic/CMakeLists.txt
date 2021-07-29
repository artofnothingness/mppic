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
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
)

foreach(pkg IN LISTS packages)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

add_library(mppic SHARED src/Controller.cpp)
target_include_directories(mppic PRIVATE include)

ament_target_dependencies(mppic
  ${packages}
)

target_link_libraries(mppic CONAN_PKG::xtensor)

# prevent pluginlib from using boost
target_compile_definitions(mppic PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_options(mppic PRIVATE -fconcepts)

set_target_properties(mppic
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
)



install(TARGETS mppic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(mppic)
ament_export_dependencies(${packages})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)

add_subdirectory(test)

ament_package()
