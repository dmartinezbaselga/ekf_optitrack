# EKF for Optitrack

This repository contrains a ROS implementation of a Kalman Filter for 2-D position a velocity tracking with an Optitrack system, using a constant velocity assumption.

## Considerations
It assumes there is an Optitrack system publishing poses in the topics **/vrpn_client_node/pedestrian_i/pose** and **/vrpn_client_node/robot_j/pose** for i in N pedestrians and j in M robots in the environment. You can easily change this topics in the file **src/ekf.cpp**. It publishes the tracked poses and velocities in the topic **/ekf_poses**.

## Launch commands
Edit the file ekf.launch according to the number of robots and pedestrians you have.
'''
roslaunch ekf_optitrack ekf.launch
'''
