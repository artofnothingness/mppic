if(BUILD_TESTING)

  add_executable(optimizer-benchmark OptimizerBenchmark.cpp)

  ament_target_dependencies(optimizer-benchmark ${packages})
  target_link_libraries(optimizer-benchmark CONAN_PKG::xtensor CONAN_PKG::benchmark)
  target_include_directories(optimizer-benchmark PRIVATE ../include)

  target_compile_options(optimizer-benchmark PRIVATE -fconcepts)

  set_target_properties(optimizer-benchmark
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )

endif()
