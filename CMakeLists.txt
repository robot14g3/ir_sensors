cmake_minimum_required(VERSION 2.8.3)
project(ir_sensors)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES ir_sensors
 # CATKIN_DEPENDS roscpp std_msgs
 # DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(ir_calibration src/ir_calibration.cpp)
target_link_libraries(ir_calibration ${catkin_LIBRARIES})

add_executable(ir_sensors src/ir_sensors.cpp)
target_link_libraries(ir_sensors ${catkin_LIBRARIES})
