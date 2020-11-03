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
from sensor_msgs.msg import Joy
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list

# create global pose
global pose_goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.2
pose_goal.position.y = 0.2
pose_goal.position.z = 0.6
pose_goal.orientation.w = 0.1

def joyCB():
    pass


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

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group1 = move_group1
        self.move_group2 = move_group2


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

    def move_to_pose(self, move_group, pose_goal):
        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()


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
        # setup robot arms
        interface = MoveGroupPythonInteface()

        # home robot
        #interface.home_robot()

        # select arm
        move_group = interface.move_group2
        current_pose = move_group.get_current_pose()
        pose_goal.position.x = current_pose.pose.position.x + 0.0
        pose_goal.position.y = current_pose.pose.position.y - 0.1
        pose_goal.position.z = current_pose.pose.position.z - 0.0
        pose_goal.orientation.w =  0.0

        # try to move to a location
        interface.move_to_pose(move_group, pose_goal)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
