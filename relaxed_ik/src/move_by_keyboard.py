#! /usr/bin/env python
__author__ = "_j_"
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
from tf.transformations import quaternion_from_euler

global last_pos, last_quat #quat actually means euler

class move_arm(object):
    def __init__(self):
        rospy.init_node('arm_trajectory')
        #rospy.Subscriber('/joy',Joy,self.cartesion_control,queue_size=1)
        self.relaxedIK = RelaxedIK.init_from_config(config_file_name)
        self.right_pose = Pose()
        self.left_pose = Pose()
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_ee_pose_state, queue_size=1)
        self.left_js_pub = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size=1)

    def cb_ee_pose_state(self, data):
        # idk what this is but seems useful
        indr = data.name.index("robot::rightbracelet_link")
        indl = data.name.index("robot::leftbracelet_link")
        self.right_pose = data.pose[indr]
        self.left_pose = data.pose[indl]

    def cartesion_control(self):
        # goal_pos = [last_pos,
        #             [self.left_pose.position.x, self.left_pose.position.y, self.left_pose.position.z]]
        raw = last_quat[0]*math.pi/180
        pitch = last_quat[1]*math.pi/180
        yaw= last_quat[2]*math.pi/180
        q = quaternion_from_euler(raw, pitch, yaw)
        rospy.loginfo(str(q))
        goal_pos = [[self.left_pose.position.x, self.left_pose.position.y, self.left_pose.position.z],last_pos]
        goal_quat = [[self.left_pose.orientation.w, self.left_pose.orientation.x, self.left_pose.orientation.y,
                      self.left_pose.orientation.z],q]
        xopt = self.relaxedIK.solve(goal_pos, goal_quat)
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
    global last_pos, last_quat
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # ws-x ad-y qe-z hu-r ji-p ko-y
    if ch == 'w':
        last_pos[0] = last_pos[0] + 0.1
    elif ch == 's':
        last_pos[0] = last_pos[0] - 0.1
    elif ch == 'a':
        last_pos[1] = last_pos[1] + 0.1
    elif ch == 'd':
        last_pos[1] = last_pos[1] - 0.1
    elif ch == 'q':
        last_pos[2] = last_pos[2] + 0.1
    elif ch == 'e':
        last_pos[2] = last_pos[2] - 0.1
    elif ch == 'h':
        last_quat[0] = min(last_quat[0] + 10,180)
    elif ch == 'u':
        last_quat[0] = max(last_quat[0] - 10,-180)
    elif ch == 'j':
        last_quat[1] = min(last_quat[1] + 10,180)
    elif ch == 'i':
        last_quat[1] = max(last_quat[1] - 10,-180)
    elif ch == 'k':
        last_quat[2] = min(last_quat[2] + 10,180)
    elif ch == 'o':
        last_quat[2] = max(last_quat[2] - 10,-180)

    rospy.loginfo(last_pos)
    rospy.loginfo(last_quat)
    arm.cartesion_control()


def main():
    try:
        left_a = move_arm()
        global last_pos,last_quat
        last_pos=[0,0,0]
        last_quat=[1,0,0,0]
        left_a.home_robot()
        while not rospy.is_shutdown():

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
