cmake_minimum_required(VERSION 2.8.3)
project(rbx1_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf
    nav_msgs
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf geometry_msgs nav_msgs
)


# the node:  move forward and back
add_executable(timed_out_and_back src/timed_out_and_back.cpp)
target_link_libraries(timed_out_and_back ${catkin_LIBRARIES})
add_dependencies(timed_out_and_back ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move forward and back with odom
add_executable(odom_out_and_back src/odom_out_and_back.cpp )
target_link_libraries(odom_out_and_back ${catkin_LIBRARIES})
add_dependencies(odom_out_and_back ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move following a square
add_executable(Move_square src/Move_suqare.cpp)
target_link_libraries(Move_square ${catkin_LIBRARIES})
add_dependencies(Move_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# the node: move forward and back with odom
add_executable(Keyboard_Control src/Keyboard_Control.cpp)
target_link_libraries(Keyboard_Control ${catkin_LIBRARIES})
add_dependencies(Keyboard_Control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})