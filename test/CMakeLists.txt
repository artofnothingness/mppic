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

set(TEST_NAMES 
  optimizer_test
  controller_test
)

foreach(name IN LISTS TEST_NAMES)
  add_executable(${name} ${name}.cpp)
  ament_target_dependencies(${name} ${packages})
  target_link_libraries(${name} catch_main project_options)
  catch_discover_tests(${name})
endforeach()
