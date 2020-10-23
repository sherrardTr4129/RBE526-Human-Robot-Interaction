#! /usr/bin/env python
'''
Adapted from Relaxed_IK work
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 7/1/18

Designed to subscribe to a topic containing end-effector pose of one hand, then calculating the pose of the other hand to achieve camera following.
'''
######################################################################################################
import os
import sys
import rospy
import relaxed_ik
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float32
# from relaxed_ik.Utils.colors import bcolors
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState

class Follower_Node():
    def __init__(self):
        rospy.loginfo("Follower Node started.")
    
        '''
        you could put class data variables in here if you wanted to.
        current_joint_states = JointState()
        #eepg = None
        #def eePoseGoals_cb(data):
        #    global eepg
        #    eepg = data
        '''
        
        rospy.Subscriber('/robot/joint_states', JointState, callback_pose)
        angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions',JointAngles ,queue_size=3)
        config_file_name = 'second_victory.config' #rospy.get_param('config_file_name', default='second_victory.config')
        my_relaxedIK = RelaxedIK.init_from_config(config_file_name)
        num_chains = my_relaxedIK.vars.robot.numChains
        rospy.loginfo("rospy.spin()")
        rospy.spin()

def callback_pose(data):
     print 'here!'
     global current_joint_states
     current_joint_states = data # [data.pose.position.x, data.pose.position.y, data.pose.position.z]
     left_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
     left_joint_values = current_joint_states.position[9:]
     
     print(current_joint_states)

if __name__ == '__main__':
    print("starting follower node")
    rospy.init_node('Follower_Node')
    Follower_Node = Follower_Node()


