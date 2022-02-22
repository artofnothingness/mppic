cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(_project_options URL https://github.com/cpp-best-practices/project_options/archive/refs/tags/v0.17.0.zip)
FetchContent_MakeAvailable(_project_options)
include(${_project_options_SOURCE_DIR}/Index.cmake)


project(mppic CXX)

project_options(
  ENABLE_CONAN
  WARNINGS_AS_ERRORS
  ENABLE_DOXYGEN
  # ENABLE_COVERAGE
  # ENABLE_SANITIZER_ADDRESS
  # ENABLE_SANITIZER_LEAK
  # ENABLE_SANITIZER_UNDEFINED_BEHAVIOR
  # ENABLE_SANITIZER_THREAD
  # ENABLE_SANITIZER_MEMORY
  # CONAN_OPTIONS
)

option(ENABLE_TESTING "Enable Test Builds" ON)

if(ENABLE_CONAN)
  run_conan()
endif()


find_package(ament_cmake REQUIRED)

set(dependencies_pkgs
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

set(dependencies_libs
  xtensor
)

set(critics_sources
  src/critics/ApproxReferenceTrajectoryCritic.cpp
  src/critics/ReferenceTrajectoryCritic.cpp
  src/critics/GoalAngleCritic.cpp
  src/critics/AngleToGoalCritic.cpp
  src/critics/GoalCritic.cpp
  src/critics/ObstaclesCritic.cpp
)

set(mppic_sources
  src/Controller.cpp
  src/PathHandler.cpp
)


foreach(pkg IN LISTS dependencies_pkgs dependencies_libs)
  find_package(${pkg} REQUIRED)
endforeach()

add_library(mppic SHARED ${mppic_sources})
add_library(critics SHARED ${critics_sources})

set(libraries mppic critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC include)
  target_link_libraries(${lib} project_options ${dependencies_libs})
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

# <<<< Critics
if(ENABLE_TESTING)
  enable_testing()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(mppic critics.xml)

install(TARGETS mppic critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
