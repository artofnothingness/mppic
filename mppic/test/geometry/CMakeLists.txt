add_executable(closestPointToLineSegment2dTest closestPointToLineSegment2dTest.cpp)

ament_target_dependencies(closestPointToLineSegment2dTest ${packages})
target_link_libraries(closestPointToLineSegment2dTest CONAN_PKG::xtensor CONAN_PKG::catch2)
target_include_directories(closestPointToLineSegment2dTest PRIVATE ../../include)

target_compile_options(closestPointToLineSegment2dTest PRIVATE -fconcepts)

set_target_properties(closestPointToLineSegment2dTest 
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
)

catch_discover_tests(closestPointToLineSegment2dTest)
