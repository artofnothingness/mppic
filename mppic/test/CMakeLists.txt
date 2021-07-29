if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(optimizer-test OptimizerTest.cpp)
  ament_target_dependencies(optimizer-test ${packages})
  target_link_libraries(optimizer-test CONAN_PKG::xtensor)
  target_include_directories(optimizer-test PRIVATE ../include)

  set_target_properties(optimizer-test
      PROPERTIES
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
  )

endif()
