find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  ament_target_dependencies(${name}
    ${dependencies_pkgs}
  )
  target_link_libraries(${name}
    mppic xtensor-critics xtensor-optimizer benchmark
  )

target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
)

  if(${TEST_DEBUG_INFO})
    target_compile_definitions(${name} PUBLIC -DTEST_DEBUG_INFO)
  endif()
endforeach()
