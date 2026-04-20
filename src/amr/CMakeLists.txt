cmake_minimum_required(VERSION 3.8)
project(amr)
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  map
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_package()