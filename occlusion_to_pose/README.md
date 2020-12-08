# Overview
This package will launch the robot in simulation environment, take images from the autonomous camera arm and the current pose, and compute a new pose that minimizes occlusions.

## Topics
|                  Topic                  |        Type        |                                  Description                                  |
|:---------------------------------------:|:------------------:|:-----------------------------------------------------------------------------:|
| /trina2\_1/right\_arm\_cam/color/image\_raw | sensor\_msgs/Image  | The image topic from the Trina2 robot right arm that this node subscribes to. |
| /camArmPoseGoal                         | geometry\_msgs/Pose | The new goal Pose of the Trina2 robot right arm that this node subscribes to. |
| /currentCamArmPose                      | geometry\_msgs/Pose | The current Pose of the Trina2 robot right arm that this node subscribes to.  |
| /raw_image_with_contour                 |  sensor\_msgs/Image  | The image topic that processes images from /trina2\_1/right\_arm\_cam/color/image\_raw |

## To Run the Node
- To open the Gazebo simulation environment, run
`roslaunch occlusion_to_pose trina2.launch`
- To start the occlusion avoidance algorithm after Gazebo simulation starts and the robot reaches the initial position, run
`rosrun occlusion_to_pose occlusion_to_pose.py`
- To control the robot through web gui, open `joy.html` file in your browser. This file is under `hri_web_gui\web\html`. The image shown in this web gui is the first raw image topic listed above.
- To change the raw image topic to `/raw_image_with_contour` topic for contours, open `joy.html` file in editor, change the line
`<iframe img src="http://localhost:8080/stream_viewer?topic=/trina2_1/right_arm_cam/color/image_raw" width="800" height="900"></iframe>`
to
`<iframe img src="http://localhost:8080/stream_viewer?topic=/raw_image_with_contour" width="800" height="900"></iframe>`
