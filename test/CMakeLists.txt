find_package(Catch2 REQUIRED)
include(Catch)

set(TEST_NAMES
  controller_test
  optimizer_test
)

foreach(name IN LISTS TEST_NAMES)
  ament_add_catch2(${name}
    ${name}.cpp
  )
  ament_target_dependencies(${name}
    ${dependencies_pkgs}
  )
  target_link_libraries(${name}
    mppic xtensor-critics xtensor-optimizer Catch2::Catch2WithMain
  )

  if(${DO_BENCHMARKS})
    target_compile_definitions(${name} PUBLIC -DDO_BENCHMARKS)
  endif()
endforeach()
