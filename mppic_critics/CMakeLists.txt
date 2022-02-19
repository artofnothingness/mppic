cmake_minimum_required(VERSION 3.5)
project(mppic_critics)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mppic_base REQUIRED)
find_package(pluginlib REQUIRED)

set(critics
  ApproxReferenceTrajectoryCritic
  ReferenceTrajectoryCritic
  GoalAngleCritic
  GoalCritic
  ObstaclesCritic
)

function(ADD_PLUGINS PATH NAMES)
  foreach(name IN NAMES)
    string(CONCAT full_path ${PATH}/${name}.cpp)
    add_library(${name} SHARED ${full_path})
    target_link_libraries(${name} project_options)
    ament_target_dependencies(${name}
      "mppic_base"
      "pluginlib"
    )

    install(
      TARGETS ${name}
      EXPORT export_${name}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    ament_export_libraries(
      ${name}
    )
    ament_export_targets(
      export_${name}
    )
  endforeach()

endfunction()

ADD_PLUGINS(src ${critics})
pluginlib_export_plugin_description_file(mppic_base critics.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_export_include_directories(
  include
)

ament_package()
