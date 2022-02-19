cmake_minimum_required(VERSION 3.5)
project(mppic_base)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_package()

# cmake_minimum_required(VERSION 3.15)
# project(mppic_core CXX)
#
# option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
# option(ENABLE_TESTING "Enable Test Builds" ON)
# option(ENABLE_CONAN "Use Conan for dependency management" ON)
#
# include(..cmake/StandardProjectSettings.cmake)
# include(..cmake/CompilerWarnings.cmake)
# include(..cmake/Sanitizers.cmake)
# include(..cmake/Doxygen.cmake)
# include(..cmake/StaticAnalyzers.cmake)
# include(..cmake/Conan.cmake)
#
# if(ENABLE_CONAN)
#   run_conan()
# endif()
#
# find_package(ament_cmake REQUIRED)
# find_package(xtensor REQUIRED)
#
# add_library(project_options INTERFACE)
# target_include_directories(project_options INTERFACE include)
# target_compile_features(project_options INTERFACE cxx_std_17)
# target_compile_options(project_options INTERFACE -fconcepts)
# target_link_libraries(project_options INTERFACE xtensor)
#
# enable_sanitizers(project_options)
# set_project_warnings(project_options)
#
# set(packages 
#   rclcpp 
#   pluginlib 
#   tf2 
#   geometry_msgs 
# )
#
# foreach(pkg IN LISTS packages)
#   find_package(${pkg} REQUIRED)
# endforeach()
#
# install(
#   DIRECTORY include/
#   DESTINATION include
# )
#
#
# install(TARGETS mppic
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
#
# install(DIRECTORY include/
#   DESTINATION include/
# )
#
# ament_package()
