if(BUILD_TESTING)

  enable_testing()

  add_library(catch_main STATIC catch_main.cpp)
  ament_target_dependencies(catch_main rclcpp)
  target_link_libraries(catch_main CONAN_PKG::catch2)

  add_executable(optimizer-test optimizer-test.cpp)
  ament_target_dependencies(optimizer-test ${packages})
  target_link_libraries(optimizer-test CONAN_PKG::xtensor catch_main)
  target_include_directories(optimizer-test PRIVATE ../include)
  target_compile_options(optimizer-test PRIVATE -fconcepts)
  set_target_properties(optimizer-test
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )


  add_executable(geometry-test geometry-test.cpp)
  ament_target_dependencies(geometry-test ${packages})
  target_link_libraries(geometry-test CONAN_PKG::xtensor catch_main)
  target_include_directories(geometry-test PRIVATE ../include)
  target_compile_options(geometry-test PRIVATE -fconcepts)
  set_target_properties(geometry-test 
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )


  
  if (${DO_BENCHMARKS})
    target_compile_definitions(optimizer-test PRIVATE -DDO_BENCHMARKS)
    target_compile_definitions(geometry-test PRIVATE -DDO_BENCHMARKS)
  endif()

  catch_discover_tests(optimizer-test)
  catch_discover_tests(geometry-test)

endif()
