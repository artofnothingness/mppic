cmake_minimum_required(VERSION 3.15)
project(mppic CXX)

option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable Test Builds" ON)
option(ENABLE_CONAN "Use Conan for dependency management" OFF)

include(cmake/StandardProjectSettings.cmake)
include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Doxygen.cmake)
include(cmake/StaticAnalyzers.cmake)
include(cmake/Conan.cmake)

if(ENABLE_CONAN)
  run_conan()
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(xtensor REQUIRED)

add_library(project_options INTERFACE)
target_include_directories(project_options INTERFACE include)
target_compile_features(project_options INTERFACE cxx_std_17)
target_compile_options(project_options INTERFACE -fconcepts)
target_link_libraries(project_options INTERFACE xtensor)

enable_sanitizers(project_options)
set_project_warnings(project_options)


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

add_library(mppic SHARED src/Controller.cpp src/PathHandler.cpp)
ament_target_dependencies(mppic
  ${packages}
)

target_link_libraries(mppic project_options)

if(ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_export_libraries(mppic)
ament_export_dependencies(${packages})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)

install(TARGETS mppic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
