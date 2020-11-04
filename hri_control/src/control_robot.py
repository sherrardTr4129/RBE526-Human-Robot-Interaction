#!/usr/bin/env python

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
# from RoboPuppetMQP.msg import joint_angle
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global x,y,z, roll, pitch, yaw, ox, oy, oz, ow

class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). 
        ## This interface can be used to plan and execute motions:
        group1_name = "left_arm"
        group2_name = "right_arm"
        move_group1 = moveit_commander.MoveGroupCommander(group1_name)
        move_group2 = moveit_commander.MoveGroupCommander(group2_name)
        #rospy.Subscriber('/desired_angle_left', joint_angle, self.move_left_arm, queue_size=1)
        #rospy.Subscriber('/desired_angle_right', joint_angle, self.move_right_arm, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.go_to_pose_goal, queue_size=1)
        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group1 = move_group1
        self.move_group2 = move_group2

    def go_to_pose_goal(self,joy):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group1
        global x, y, z, roll, pitch, yaw, ox, oy, oz, ow

        ## ----- Control based on Ninetendo(Jialin's) Joystick -----
        current_pose = move_group.get_current_rpy()
        roll = current_pose[0] + joy.axes[4]*.3
        pitch = current_pose[1] + joy.axes[5]*.3
        yaw = current_pose[2] +  joy.axes[3]*.3
        x = x + joy.buttons[0]*0.05-joy.buttons[2]*0.05  
        y= y + joy.buttons[3]*0.05-joy.buttons[1]*0.05  
        z = z + joy.buttons[7] * 0.05-joy.buttons[6]*0.05
        arr = quaternion_from_euler(roll,pitch,yaw)
        ox = arr[0]
        oy = arr[1]
        oz = arr[2]
        ow = arr[3]

        ## ----- Control based on Xbox(Jack's) Joystick -----
        # x = x + joy.buttons[1]*0.05-joy.buttons[2]*0.05
        # y= y + joy.buttons[3]*0.05-joy.buttons[0]*0.05
        # z = z + joy.buttons[7] * 0.05
        
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = ow
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz

        ## If you want the end-effector to be prependicular to the groud
        # pose_goal.orientation.w = 0.0
        # pose_goal.orientation.x = 1.0
        # pose_goal.orientation.y = 0.0
        # pose_goal.orientation.z = 0.0

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)
        move_group.set_goal_tolerance = 0.08
        #rospy.loginfo("going to" + str(pose_goal.position.x) + "    "+ str(pose_goal.position.y) + "    "+ str(pose_goal.position.z))
        while True:
            plan = move_group.go(wait=True)
            current_pose = move_group.get_current_pose().pose
            if all_close(pose_goal, current_pose, 0.05):
                break


        ## Now, we call the planner to compute the plan and execute it.

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)


    def go_to_cat_goal(self,a,b,c,d):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        move_group = self.move_group1
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.0
        pose_goal.orientation.x = 1.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.position.x = a
        pose_goal.position.y = b
        pose_goal.position.z = c


        #rospy.loginfo("going to" + str(pose_goal.position.x) + "    "+ str(pose_goal.position.y) + "    "+ str(pose_goal.position.z))
        while True:
            move_group.set_pose_target(pose_goal)
            plan = move_group.go(wait=True)

            current_pose = move_group.get_current_pose().pose
            if all_close(pose_goal, current_pose, 0.1):
                break

        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.1)


    def move_left_arm(self,angles):
        move_group1 = self.move_group1
        joint_goal1 = move_group1.get_current_joint_values()
        joint_goal1[0] = angles.left[0]
        joint_goal1[1] = angles.left[1]
        joint_goal1[2] = angles.left[2]
        joint_goal1[3] = angles.left[3]
        joint_goal1[4] = angles.left[4]
        joint_goal1[5] = angles.left[5]
        joint_goal1[6] = angles.left[6]
        # self.move_to_goal(move_group1, joint_goal1, 10)
        # move_group1.stop()
        move_group1.go(joint_goal1, wait=True)
        move_group1.stop()

    def move_right_arm(self,angles):
        move_group2 = self.move_group2
        joint_goal2 = move_group2.get_current_joint_values()
        joint_goal2[0] = angles.right[0]
        joint_goal2[1] = angles.right[1]
        joint_goal2[2] = angles.right[2]
        joint_goal2[3] = angles.right[3]
        joint_goal2[4] = angles.right[4]
        joint_goal2[5] = angles.right[5]
        joint_goal2[6] = angles.right[6]
        # self.move_to_goal(move_group2, joint_goal2, 10)
        # move_group2.stop()
        # move_group2.go(joint_goal2, wait=True)
        # move_group2.stop()

    def home_robot(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group1 = self.move_group1
        move_group2 = self.move_group2

        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the joint values from the group and adjust some of the values:
        joint_goal1 = move_group1.get_current_joint_values()
        joint_goal1[0] = -2.1
        joint_goal1[1] = 1.57
        joint_goal1[2] = 1.57
        joint_goal1[3] = 1.57
        joint_goal1[4] = 0.0
        joint_goal1[5] = 0.0
        joint_goal1[6] = 1.57
        # must be decimal
        # joint_goal1[0] = 0.0
        # joint_goal1[1] = 0.0
        # joint_goal1[2] = 0.0
        # joint_goal1[3] = 0.0
        # joint_goal1[4] = 0.0
        # joint_goal1[5] = 0.0
        # joint_goal1[6] = 0.0

        joint_goal2 = move_group2.get_current_joint_values()
        joint_goal2[0] = -2.8
        joint_goal2[1] = 1.57
        joint_goal2[2] = 0.0
        joint_goal2[3] = 1.1
        joint_goal2[4] = 0.2
        joint_goal2[5] = 2.015
        joint_goal2[6] = -3.05

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # move_group1.go(joint_goal1, wait=True)

        # Move the robot arm to home position
        # 10 trials
        self.move_to_goal(move_group1, joint_goal1, 10)
        self.move_to_goal(move_group2, joint_goal2, 10)

        # while True:
        #     move_group1.go(joint_goal1, wait=True)
        #     move_group2.go(joint_goal2, wait=True)
        #     current_joints1 = move_group1.get_current_joint_values()
        #     current_joints2 = move_group2.get_current_joint_values()
        #     if all_close(joint_goal1, current_joints1, 0.05) and all_close(joint_goal2, current_joints2, 0.05):
        #         break



        # Calling ``stop()`` ensures that there is no residual movement
        move_group1.stop()
        move_group2.stop()

    def move_to_goal(self, move_group, joint_goal, trials):
        trial = 0
        while True:
            move_group.go(joint_goal, wait=True)
            current_joints = move_group.get_current_joint_values()
            if all_close(joint_goal, current_joints, 0.05):
                break
            
            # Check trial times
            trial += 1
            if trial > 9:
                rospy.loginfo("Moving arm to home failed")
                break


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if (abs(actual[index] - goal[index]) > tolerance) and \
               (abs(abs(actual[index]-goal[index])-2*pi) > tolerance) :
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


def main():
    try:
        # Home robot arm
        interface = MoveGroupPythonInteface()
        interface.home_robot()
        rospy.sleep(5)
        global x, y, z, roll, pitch, yaw, ox, oy, oz, ow
        current_pose = interface.move_group1.get_current_pose().pose
        ox = current_pose.orientation.x
        oy = current_pose.orientation.y
        oz = current_pose.orientation.z
        ow = current_pose.orientation.w
        (r, p, y) = euler_from_quaternion([ox, oy, oz, ow])
        roll = r
        pitch = p
        yaw = y
        x =current_pose.position.x
        y=current_pose.position.y
        z= current_pose.position.z
        # interface.go_to_cat_goal(0.2,0.3,0.7,1.0)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
