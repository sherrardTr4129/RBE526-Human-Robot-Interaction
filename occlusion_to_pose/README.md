# Node Overview
This node will take images from the autonomous camera arm and the current pose, and compute a new pose that minimizes occlusions.

## Topics
|                  Topic                  |        Type        |                                  Description                                  |
|:---------------------------------------:|:------------------:|:-----------------------------------------------------------------------------:|
| /trina2\_1/right\_arm\_cam/color/image\_raw | sensor\_msgs/Image  | The image topic from the Trina2 robot right arm that this node subscribes to. |
| /camArmPoseGoal                         | geometry\_msgs/Pose | The new goal Pose of the Trina2 robot right arm that this node subscribes to. |
| /currentCamArmPose                      | geometry\_msgs/Pose | The current Pose of the Trina2 robot right arm that this node subscribes to.  |
