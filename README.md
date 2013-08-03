PR2 Surrogate
=============

This package enables fully immersive control of a PR2 robot using the Oculus Rift and Razer Hydra.

What it does:
- Render the Kinect point cloud and robot model to the Oculus Rift
- Use the Oculus head tracking to control the PR2 head
- Connect the Hydra to the PR2 joystick teleop (base motion, torso lift & gripper control)
- Track the Hydra motion with the PR2 grippers

Installation:
- Clone this repo and [rviz_oculus](https://github.com/ros-visualization/rviz_oculus) into a catkin
- `sudo apt-get install ros-groovy-razer-hydra`
- call `catkin_make`

How to run:
- Connect your Hydra and Oculus Rift
- on the robot: `robot start`, then `roslaunch pr2_surrogate pr2_teleop_robot.launch`
- on the desktop: `roslaunch pr2_surrogate pr2_teleop_desktop.launch`
