cmake_minimum_required(VERSION 3.5)
project(mppic)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)

nav2_package()

# Download automatically, you can also just copy the conan.cmake file
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
   message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
   file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake"
                  "${CMAKE_BINARY_DIR}/conan.cmake")
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(REQUIRES 
                xtensor/0.21.5                
                BASIC_SETUP CMAKE_TARGETS)



add_library(mppic SHARED
    src/Controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(mppic PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_options(mppic PRIVATE -fconcepts)
target_include_directories(mppic PRIVATE include)


set_target_properties(mppic
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
)


set(mppic_dependencies
  rclcpp
  pluginlib
  tf2

  geometry_msgs
  nav_msgs

  nav2_core
  nav2_costmap_2d
  nav2_util
)

ament_target_dependencies(mppic
  ${mppic_dependencies}
)

target_link_libraries(mppic CONAN_PKG::xtensor)

install(TARGETS mppic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(mppic)
ament_export_dependencies(${mppic_dependencies})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)


ament_package()

