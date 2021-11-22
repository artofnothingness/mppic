cmake_minimum_required(VERSION 3.5)
project(pointcloud_filter_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(octomap_ros REQUIRED)

add_definitions(${PCL_DEFINITIONS} ${OCTOMAP_DEFINITIONS})
message("Pcl found in ${PCL_LIBRARY_DIRS}")

add_executable(pointcloud_subscriber src/subscriber.cpp)
target_include_directories(pointcloud_subscriber PUBLIC ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_subscriber PUBLIC ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
ament_target_dependencies(pointcloud_subscriber rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS mppic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_package()
