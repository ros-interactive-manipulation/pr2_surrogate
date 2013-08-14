PR2 Surrogate
=============

This package enables fully immersive control of a PR2 robot using the Oculus Rift and Razer Hydra.

What it does:
- Render the Kinect point cloud and robot model to the Oculus Rift
- Use the Oculus head tracking to control the PR2 head
- Connect the Hydra to the PR2 joystick teleop (base motion, torso lift & gripper control)
- Track the Hydra motion with the PR2 grippers

How to run:
- Connect your Hydra and Oculus Rift
- On the robot: `robot start`, then `roslaunch pr2_surrogate robot.launch`
- On the desktop: `roslaunch pr2_surrogate desktop.launch`
