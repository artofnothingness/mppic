
add_library(mppic SHARED Controller.cpp PathHandler.cpp)

target_include_directories(mppic PRIVATE ../include)
ament_target_dependencies(mppic
  ${packages}
)

target_link_libraries(mppic CONAN_PKG::xtensor)

# prevent pluginlib from using boost
target_compile_definitions(mppic PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_options(mppic PRIVATE -fconcepts)

set_target_properties(mppic
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
)

