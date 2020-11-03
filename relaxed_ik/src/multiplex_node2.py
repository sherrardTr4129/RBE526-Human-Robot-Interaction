#! /usr/bin/env python
__author__ = "_r_"

######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name
from RelaxedIK.relaxedIK import RelaxedIK
# from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
import rospy
import roslaunch
import os
import tf
import math


class MultiplexerInterface(object):
    def __init__(self):
        rospy.init_node('multiplex_node')
        self.control_state = None
        self.weight_array = []

        self.right_pose = Pose()
        self.left_pose = Pose()

        self.relaxedIK = RelaxedIK.init_from_config(config_file_name)

        self.left_js_pub = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size=1)
        self.right_js_pub = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size=1)

        rospy.Subscriber('/controlState', String, self.cb_control_state, queue_size=1)
        rospy.Subscriber('/uiJointPose', JointState, self.cb_joint_control, queue_size=1)
        # rospy.Subscriber('/uiJointPose', JointState, self.cb_joint_control, queue_size=1)
        rospy.Subscriber('/uiCartesianPose', Pose, self.cb_cartesion_control, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.cb_objective_control, queue_size=1)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_ee_pose_state, queue_size=1)
        rospy.Subscriber('/uiRelaxedIk', JointState, self.cb_obj_weight, queue_size=1)

    def cb_obj_weight(self, data):
        if self.weight_array == []:
            return
        if data.position == []:
            print(type(data.position))
            return

        if self.control_state == 'objective':
            self.weight_array = [1, 1, 1, 1, 1,
                                 1, 1, 1, 0, 1,
                                 0, 0]

            print("objective weights updated: ",data.position)

            for i, val in enumerate(data.position):
                self.weight_array[i] = self.weight_array[i] * val

            for ind, wi in enumerate(self.weight_array):
                self.relaxedIK.vars.weight_funcs[ind].set_value(wi)
        return

    def cb_ee_pose_state(self, data):

        indr = data.name.index("robot::rightbracelet_link")
        indl = data.name.index("robot::leftbracelet_link")
        self.right_pose = data.pose[indr]
        self.left_pose = data.pose[indl]

    def cb_joint_control(self, data):
        if not self.control_state == 'joint':
            return

        # left_arm_names = data.name[:7]
        # left_arm_positions = data.position[:7]

        right_arm_names = data.name
        right_arm_positions = data.position

        # left_msg = self.make_JointTrajectory_from_joints(vals=left_arm_positions, names=left_arm_names)
        right_msg = self.make_JointTrajectory_from_joints(vals=right_arm_positions, names=right_arm_names)

        # self.left_js_pub.publish(left_msg)
        # print(right_msg)
        self.right_js_pub.publish(right_msg)

        return

    def cb_cartesion_control(self, data):
        if not self.control_state == 'cartesian' or self.weight_array == []:
            return

        goal_pos = [[data.position.x, data.position.y, data.position.z],
                    [self.left_pose.position.x, self.left_pose.position.y, self.left_pose.position.z]]
        goal_quat = [[data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z],
                     [self.left_pose.orientation.w, self.left_pose.orientation.x, self.left_pose.orientation.y,
                      self.left_pose.orientation.z]]

        xopt = self.relaxedIK.solve(goal_pos, goal_quat)

        # print(xopt)

        right_arm_names = joint_ordering[:7]
        right_arm_positions = xopt[:7]

        right_msg = self.make_JointTrajectory_from_joints(vals=right_arm_positions, names=right_arm_names)

        self.right_js_pub.publish(right_msg)

    def cb_objective_control(self, data):
        if not self.control_state == 'objective' or self.weight_array == []:
            return

        goal_pos = [[self.right_pose.position.x, self.right_pose.position.y, self.right_pose.position.y],
                    [self.left_pose.position.x, self.left_pose.position.y, self.left_pose.position.z]]
        goal_quat = [[self.right_pose.orientation.w, self.right_pose.orientation.x, self.right_pose.orientation.y,
                      self.right_pose.orientation.z],
                     [self.left_pose.orientation.w, self.left_pose.orientation.x, self.left_pose.orientation.y,
                      self.left_pose.orientation.z]]
        #
        goal_pos = [[0, 0, 0], [0, 0, 0]]
        goal_quat = [[1, 0, 0, 0], [1, 0, 0, 0]]

        overwrite_joints = ['leftjoint_1', 'leftjoint_2', 'leftjoint_3', 'leftjoint_4', 'leftjoint_5', 'leftjoint_6',
                            'leftjoint_7']
        overwrite_joint_values = data.position[:7]

        xopt = self.relaxedIK.solve(goal_pos, goal_quat, overwrite_joints, overwrite_joint_values)
        # xopt = self.relaxedIK.solve(goal_pos, goal_quat)

        # print(xopt)

        right_arm_names = joint_ordering[:7]
        right_arm_positions = xopt[:7]

        right_msg = self.make_JointTrajectory_from_joints(vals=right_arm_positions, names=right_arm_names)
        self.right_js_pub.publish(right_msg)

    def cb_control_state(self, data):

        if data.data == self.control_state:
            return
        else:
            self.control_state = data.data
            print("mode changed to :", self.control_state)

        if data.data == 'joint':
            self.weight_array = [0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0,
                                 0, 0]
        elif data.data == 'cartesian':
            self.weight_array = [0, 0, 0, 0, 0,
                                 1, 1, 1, 1, 50,
                                 1, 1]
        elif data.data == 'objective':
            self.weight_array = [1, 1, 1, 1, 1,
                                 1, 1, 1, 0, 1,
                                 0, 0]

        for ind, wi in enumerate(self.weight_array):
            self.relaxedIK.vars.weight_funcs[ind].set_value(wi)
        return

    def make_JointTrajectory_from_joints(self, vals=[], names=[]):
        if not len(vals) == len(names):
            print("ERROR: length miss match")
            return None

        JT_msg = JointTrajectory()
        JT_msg.joint_names = names

        JTP_msg = JointTrajectoryPoint()
        JT_msg.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        JTP_msg.time_from_start = rospy.Duration.from_sec(1.0)

        JTP_msg.positions = vals
        JTP_msg.velocities = [0.0] * len(names)
        JTP_msg.accelerations = [0.0] * len(names)
        JTP_msg.effort = [0.0] * len(names)

        JT_msg.points.append(JTP_msg)
        return JT_msg


def main():
    try:

        multiplexer = MultiplexerInterface()

        rospy.spin()

        print "===xxxxxxxx End"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
