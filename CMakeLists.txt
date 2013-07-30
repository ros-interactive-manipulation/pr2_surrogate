cmake_minimum_required(VERSION 2.8.3)
project(pr2_surrogate)

find_package(catkin REQUIRED 
  COMPONENTS
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

add_executable(pr2_surrogate
  src/teleop_node.cpp
  src/head_pointer.cpp
  )

target_link_libraries(pr2_surrogate
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

install(TARGETS pr2_surrogate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
