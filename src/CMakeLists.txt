add_library(mppic SHARED Controller.cpp PathHandler.cpp)
ament_target_dependencies(mppic
  ${packages}
)

target_link_libraries(mppic project_options)
