#! /usr/bin/env python
__author__ = "_r_"
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals
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
from sensor_msgs.msg import Joy
import  sys
import  tty, termios


global last_pos, last_quat

class move_arm(object):
    def __init__(self):
        rospy.init_node('arm_trajectory')
        #rospy.Subscriber('/joy',Joy,self.cartesion_control,queue_size=1)
        self.relaxedIK = RelaxedIK.init_from_config(config_file_name)
	
        self.left_js_pub = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size=1)
        self.weight_array = [0, 0, 0, 0, 0,
                             1, 1, 1, 1, 50,
                             1, 1]
        for ind, wi in enumerate(self.weight_array):
            self.relaxedIK.vars.weight_funcs[ind].set_value(wi)

    def cartesion_control(self):
        # global last_pos,last_quat
        # if joy_data.buttons[0] == 1:
        #     last_pos[2]=last_pos[2]+0.1

        goal_pos = [[0,0,0],last_pos]
        goal_quat = [[1,0,0,0],last_quat]


        xopt = self.relaxedIK.solve(goal_pos, goal_quat)

        # print(xopt)

        left_arm_names = joint_ordering[7:]
        left_arm_positions = xopt[7:]

        left_msg = self.make_JointTrajectory_from_joints(vals=left_arm_positions, names=left_arm_names)

        self.left_js_pub.publish(left_msg)


    def home_robot(self):
        goal_pos = [[0,0,0],[0,0,0]]
        goal_quat = [[1,0,0,0],[1,0,0,0]]

        xopt = self.relaxedIK.solve(goal_pos, goal_quat)

        # print(xopt)

        left_arm_names = joint_ordering[7:]
        left_arm_positions = xopt[7:]

        left_msg = self.make_JointTrajectory_from_joints(vals=left_arm_positions, names=left_arm_names)

        self.left_js_pub.publish(left_msg)

    def make_JointTrajectory_from_joints(self, vals=[], names=[]):
        if not len(vals) == len(names):
            print("ERROR: length miss match")
            return None

        JT_msg = JointTrajectory()
        JT_msg.joint_names = names

        JTP_msg = JointTrajectoryPoint()
        JT_msg.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        JTP_msg.time_from_start = rospy.Duration.from_sec(2.5)

        JTP_msg.positions = vals
        JTP_msg.velocities = [0.0] * len(names)
        JTP_msg.accelerations = [0.0] * len(names)
        JTP_msg.effort = [0.0] * len(names)

        JT_msg.points.append(JTP_msg)
        return JT_msg

def keyboard_control(arm):
    global last_pos
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if ch == 'w':
        last_pos[0] = last_pos[0] + 0.1
    elif ch == 's':
        last_pos[0]=last_pos[0]-0.1
    elif ch == 'a':
        last_pos[1] = last_pos[1] + 0.1
    elif ch == 'd':
        last_pos[1] = last_pos[1] - 0.1
    elif ch == 'j':
        last_pos[2] = last_pos[2] + 0.1
    elif ch == 'k':
        last_pos[2] = last_pos[2] - 0.1

    arm.cartesion_control()


def main():
    try:
        left_a = move_arm()
        global last_pos,last_quat
        last_pos=[0,0,0]
        last_quat=[1,0,0,0]
        counter = 1
        stride = 0.3
        s = 0.4
        left_a.home_robot()
        while not rospy.is_shutdown():
            rospy.loginfo(last_pos)
            keyboard_control(left_a)
            #c= math.cos(counter)
            # c=  -1 * abs(c)
            #left_a.cartesion_control(goal_pos=[-0.2, c*s- 0.1, 0.7])
            #counter+=stride
            rospy.sleep(0.1)


        print "===xxxxxxxx End"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
