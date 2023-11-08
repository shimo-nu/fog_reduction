cmake_minimum_required(VERSION 3.14)
project(fog_reduction)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

ament_auto_add_executable(fog_reduction
  src/fog_reduction_node.cpp
  src/fog_reduction_core.cpp
)

link_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(fog_reduction ${PCL_LIBRARIES})

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
