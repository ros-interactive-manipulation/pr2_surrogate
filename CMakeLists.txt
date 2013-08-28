cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_make_distribution(0.1.3)

rosbuild_init()

find_package(Boost REQUIRED COMPONENTS system)

# Oculus SDK (required because we're depending on oculus_rviz_plugins)
FIND_PACKAGE( OculusSDK REQUIRED )
link_directories(${OculusSDK_LIBRARY_DIRS})


rosbuild_add_executable(head_pointer
  src/head_pointer_node.cpp
  src/head_pointer.cpp
  )
target_link_libraries(head_pointer
   ${Boost_LIBRARIES}
)

rosbuild_add_executable(arm_mover
  src/arm_mover_node.cpp
  src/arm_mover.cpp
  )
target_link_libraries(arm_mover
   ${Boost_LIBRARIES}
)

rosbuild_add_executable(terminator_eye
  src/terminator_eye_node.cpp
  src/terminator_eye.cpp
  )
target_link_libraries(terminator_eye
   ${Boost_LIBRARIES}
)

