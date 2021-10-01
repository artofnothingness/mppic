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

add_executable(optimizer-test optimizer-test.cpp)
ament_target_dependencies(optimizer-test ${packages})
target_link_libraries(optimizer-test catch_main project_options)
catch_discover_tests(optimizer-test)

add_executable(geometry-test geometry-test.cpp)
ament_target_dependencies(geometry-test ${packages})
target_link_libraries(geometry-test catch_main project_options)
catch_discover_tests(geometry-test)
