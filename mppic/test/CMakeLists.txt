find_package(Catch2 REQUIRED)
include(Catch)

add_library(catch_main STATIC catch_main.cpp)
ament_target_dependencies(catch_main rclcpp)
target_link_libraries(catch_main Catch2::Catch2 mppic)

if (${DO_BENCHMARKS})
  target_compile_definitions(catch_main PUBLIC -DDO_BENCHMARKS)
endif()

if (${TEST_DEBUG_INFO})
  target_compile_definitions(catch_main PUBLIC -DTEST_DEBUG_INFO)
endif()


macro(mppi_add_tests test_names)
  foreach(mppi_test IN LISTS test_names)
    find_package(${pkg} REQUIRED)
    add_executable(${test_name} {test_name}.cpp)
    ament_target_dependencies(${test_name} ${packages})
    target_link_libraries(${test_name} catch_main project_options)
    catch_discover_tests(${test_name})
  endforeach()
endmacro()

mppi_add_tests(
  optimizer_test 
  geometry_test
)
