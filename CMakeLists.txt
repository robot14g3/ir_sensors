cmake_minimum_required(VERSION 2.8.3)
project(ir_sensors)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ir_sensors
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ir_sensors
#   src/${PROJECT_NAME}/ir_sensors.cpp
# )

## Declare a cpp executable
# add_executable(ir_sensors_node src/ir_sensors_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ir_sensors_node ir_sensors_generate_messages_cpp)
