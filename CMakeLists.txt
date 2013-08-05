cmake_minimum_required(VERSION 2.8.3)
project(pr2_surrogate)

find_package(catkin REQUIRED 
  COMPONENTS
    sensor_msgs
    geometry_msgs
    pr2_controllers_msgs
    actionlib
    roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_surrogate
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(head_pointer
  src/head_pointer_node.cpp
  src/head_pointer.cpp
  )
target_link_libraries(head_pointer
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(head_pointer
  sensor_msgs_generate_messages_cpp
)

add_executable(arm_mover
  src/arm_mover_node.cpp
  src/arm_mover.cpp
  )
target_link_libraries(arm_mover
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(arm_mover
  sensor_msgs_generate_messages_cpp
)

add_executable(terminator_eye
  src/terminator_eye_node.cpp
  src/terminator_eye.cpp
  )
target_link_libraries(terminator_eye
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(terminator_eye
  sensor_msgs_generate_messages_cpp
)

install(TARGETS head_pointer arm_mover terminator_eye
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
