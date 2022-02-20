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
  ENABLE_COVERAGE
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
  mppic_base 
)

set(libraries 
  xtensor
)

foreach(pkg IN LISTS packages libraries)
  find_package(${pkg} REQUIRED)
endforeach()

add_library(mppic SHARED src/Controller.cpp src/PathHandler.cpp)
target_include_directories(mppic PUBLIC include)
target_compile_options(mppic PUBLIC -fconcepts)
ament_target_dependencies(mppic ${packages})
target_link_libraries(mppic project_options ${libraries})


if(ENABLE_TESTING)
  enable_testing()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# >>>> Ros
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
# <<<< Ros

ament_package()
