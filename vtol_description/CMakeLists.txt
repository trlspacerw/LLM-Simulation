cmake_minimum_required(VERSION 3.8)
project(vtol_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    meshes
    models
    scripts
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    derick_VTOL_readme.md
    custom_VTOL_URDF_setup.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
