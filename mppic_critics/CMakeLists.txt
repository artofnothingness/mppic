cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(_project_options URL https://github.com/cpp-best-practices/project_options/archive/refs/tags/v0.17.0.zip)
FetchContent_MakeAvailable(_project_options)
include(${_project_options_SOURCE_DIR}/Index.cmake)

project(mppic_critics CXX)

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

set(packages 
  rclcpp 
  pluginlib 
  geometry_msgs 
  nav_msgs 
  nav2_costmap_2d 
  mppic_base 
  mppic
)

set(libraries 
  xtensor
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(pkg IN LISTS packages libraries)
  find_package(${pkg} REQUIRED)
endforeach()


set(CRITICS 
  src/ApproxReferenceTrajectoryCritic.cpp
  src/ReferenceTrajectoryCritic.cpp
  src/GoalAngleCritic.cpp
  src/AngleToGoalCritic.cpp
  src/GoalCritic.cpp
  src/ObstaclesCritic.cpp
)

add_library(${PROJECT_NAME} SHARED ${CRITICS})
target_compile_options(${PROJECT_NAME} PRIVATE -fconcepts)
target_link_libraries(${PROJECT_NAME} project_options ${libraries})
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} ${packages})

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
