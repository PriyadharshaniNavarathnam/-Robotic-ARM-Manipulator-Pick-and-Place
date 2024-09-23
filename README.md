# Robotic-ARM-Manipulator-Pick-and-Place
Manipulator assigned to do Pick &amp; Place task

This repository contains the MATLAB code for simulating a robotic arm performing a pick-and-place task, 
alongside the RoboDK setup files for visual simulation. 
The project focuses on trajectory planning, kinematic analysis, and velocity profile generation.

Project Overview
The task involves simulating a robotic arm that:

Picks a disk from Station 1.
Moves the disk to Station 2 for labeling.
Places the disk at Station 3 for transfer to another conveyor.
Tools Used
RoboDK: To simulate the robotic arm's motion.
MATLAB: For trajectory planning, forward and inverse kinematics, and velocity profile generation.

Key Features
  1. Forward Kinematics: Calculate the position of the arm's end effector based on the given joint angles.
  2. Inverse Kinematics: Determine the joint angles needed for the arm to reach a specific position.
  3. Trajectory Planning: Plan the path between pick-and-place points using waypoints.
  4. Velocity Profiles: Generate smooth velocity transitions for the joints.


Running the MATLAB Code

1.Clone this repository:

    git clone https://github.com/yourusername/robotic-arm-pick-place.git

2. Navigate to the MATLAB/ folder and open the scripts in MATLAB.
3. Run trajectory_planning.m to simulate the arm’s motion and generate kinematic analysis.

Running the RoboDK Simulation
1. Open the RoboDK project files in the RoboDK/ folder.
2. Set up the robotic arm and station environment as described in the task.
3. Run the simulation to visualize the pick-and-place operation.

   For More Clarification:
   https://medium.com/@priyadharshaninavarathnam/automating-a-robotic-arm-for-pick-and-place-tasks-b140833f1749
   
