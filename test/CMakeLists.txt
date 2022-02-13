find_package(Catch2 REQUIRED)
include(Catch)

add_library(catch_main STATIC catch_main.cpp)
ament_target_dependencies(catch_main rclcpp)
target_link_libraries(catch_main Catch2::Catch2)

if (${DO_BENCHMARKS})
  target_compile_definitions(catch_main PUBLIC -DDO_BENCHMARKS)
endif()

if (${TEST_DEBUG_INFO})
  target_compile_definitions(catch_main PUBLIC -DTEST_DEBUG_INFO)
endif()

set(optimizer_test_name "optimizer_test")
add_executable(${optimizer_test_name} optimizer_test.cpp)
ament_target_dependencies(${optimizer_test_name} ${packages})
target_link_libraries(${optimizer_test_name} catch_main project_options)
catch_discover_tests(${optimizer_test_name})

set(geometry_test_name "geometry_test")
add_executable(${geometry_test_name} geometry_test.cpp)
ament_target_dependencies(${geometry_test_name} ${packages})
target_link_libraries(${geometry_test_name} catch_main project_options)
catch_discover_tests(${geometry_test_name})
