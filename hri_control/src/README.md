# Intro to each file
## algo.py
This file is for yicheng's individual algorithm implementation  

## constants.py
Stores a list of constants  

## control_gui.py
This node convert input from webGUI to robot commands  

## control_right_arm.py
This node enables user to send pose to the right arm. It also publishes the current pose of the right arm  

## estimateState.py
This node include the task-specific viewpoint select algorithm for the group project  

## mainwindow.py & mainwindow.ui & right_arm_gui
These are the files for the simple GUI which can control the right arm  

# How to use?
For group project,  
roslaunch hri_control trina2.launch  
rosrun hri_control estimateState.py  
*you may want to change the world to stack_new to replace the can to smaller sticks  

For individual algorithm implementation,  
roslaunch hri_control trina2_algo.launch  
rosrun hri_control algo.py  
*you are able to change the starting position by modifying algo.py (line 110-113 and line 142-156)



