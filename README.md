# -FRE--Maize-Crop-Detection-Tracking-and-Counting
# Task Description
In this task, robots navigate autonomously through a maize field (see Figure below) starting at the starting location S. Turning has to follow adjacent rows for tracks T1 to Tn, e.g.S =)T1 =)T2 =)T3 =)T4 =)T5 =)T6 =). . . In addition to the first task, the field robots have to count the number of plants in each row and present the results row by row immediately after finishing the round.

<img width="600" height="600" alt="image" src="https://github.com/user-attachments/assets/b82767b5-af75-4065-8443-f3beb99a8842" />

# Sensor Used
Realsense D435i Camera

# Pre-requisites
Install ROS2 Humble: https://docs.ros.org/en/humble/index.html

Install Realsense ROS2 Wrapper: https://github.com/IntelRealSense/realsense-ros

# Steps of Implementation
Clone the repository
```bash
git clone https://github.com/swarajtendulkar10/-FRE--Maize-Crop-Detection-Tracking-and-Counting.git
```
Add folders to the root folder
```bash
mkdir launch/ worlds/ config/ include/ description/
```
Build the workspace
```bash
colcon build
```
Run the Maize Tracker Node
```bash
ros2 run obj_track maize_tracker_new.py 
```
# Results of Implementation


https://github.com/user-attachments/assets/840bc086-d031-4ff3-bb00-1c9cc14cd1fe




