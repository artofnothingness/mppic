cmake_minimum_required(VERSION 3.15)
project(mppic_critics CXX)

option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable Test Builds" ON)
option(ENABLE_CONAN "Use Conan for dependency management" ON)

include(../cmake/StandardProjectSettings.cmake)
include(../cmake/CompilerWarnings.cmake)
include(../cmake/Sanitizers.cmake)
include(../cmake/Doxygen.cmake)
include(../cmake/StaticAnalyzers.cmake)
include(../cmake/Conan.cmake)

if(ENABLE_CONAN)
  run_conan()
endif()

set(packages 
  rclcpp 
  pluginlib 
  geometry_msgs 
  nav_msgs 
  nav2_costmap_2d 
  mppic_base 
  mppic
)

add_library(project_options INTERFACE)
target_include_directories(project_options INTERFACE include)
target_compile_features(project_options INTERFACE cxx_std_17)
target_compile_options(project_options INTERFACE -fconcepts)
target_link_libraries(project_options INTERFACE xtensor)
enable_sanitizers(project_options)
set_project_warnings(project_options)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(pkg IN LISTS packages)
  find_package(${pkg} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)


set(CRITICS 
  src/ApproxReferenceTrajectoryCritic.cpp
  src/ReferenceTrajectoryCritic.cpp
  src/GoalAngleCritic.cpp
  src/GoalCritic.cpp
  src/ObstaclesCritic.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${CRITICS}
)

target_link_libraries(${PROJECT_NAME} project_options)

ament_target_dependencies(${PROJECT_NAME}
  ${packages}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES critics.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()


ament_export_include_directories(include) 
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${packages})

pluginlib_export_plugin_description_file(mppic_base critics.xml)

ament_package()
