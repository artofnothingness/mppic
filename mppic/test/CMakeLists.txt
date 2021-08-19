if(BUILD_TESTING)

  enable_testing()

  add_executable(optimizer-test OptimizerTest.cpp)
  ament_target_dependencies(optimizer-test ${packages})
  target_link_libraries(optimizer-test CONAN_PKG::xtensor CONAN_PKG::catch2)
  target_include_directories(optimizer-test PRIVATE ../include)

  target_compile_options(optimizer-test PRIVATE -fconcepts)
  set_target_properties(optimizer-test
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )

  catch_discover_tests(optimizer-test)

  add_subdirectory(geometry)

endif()
