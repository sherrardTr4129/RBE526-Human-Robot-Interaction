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
import os                                   # python
import sys                                  # python
import rospy                                # python
import relaxed_ik                           # ROS


from RelaxedIK.relaxedIK import RelaxedIK   # python
# from relaxed_ik.msg import EEPoseGoals, JointAngles
# from std_msgs.msg import Float32
# from relaxed_ik.Utils.colors import bcolors
# ebolabot_root = os.getenv("EBOLABOT_PATH",".")
# sys.path.append(ebolabot_root)
# from Common.system_config import EbolabotSystemConfig
# from baxter_core_msgs.msg import EndpointState # ROS
from sensor_msgs.msg import JointState         # ROS
from relaxed_ik.srv import NewJointAngles, NewJointAnglesResponse   # ROS

# Add publisher to send solution to Gamepad_Action_Sequence 

# make a class under the part that runs once
# put everything else under the function call that runs at the top
# finish the service call
# comment out the node and subscriber 
# look at more examples 

left_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
joint_ordering = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
pos_goals = []
quat_goals = []


def new_joint_angles(req):
    # left arm is contained in req.position[:]
    left_joint_values = 7*[0.0]
    for i,name in enumerate(left_joints):
        index = req.left_arm.name.index(name)
        left_joint_values[i] = req.left_arm.position[index]

    print 'left hand pose:'
    print left_joint_values
    
    # call relaxed_ik.solve
    xopt = my_relaxedIK.solve(pos_goals, quat_goals, left_joints, left_joint_values)
    
    print 'solution for right hand:'
    print xopt[0:7]
    print 'solution for left hand:'
    print xopt[7:]

    # create response both_arm
    both_arms = JointState()
    # copy header from req into both_arm
    both_arms.header = req.left_arm.header
    # both_arm.name = joint_ordering from start_here.py
    both_arms.name = joint_ordering
    # xopt becomes both_arm.position    
    both_arms.position = xopt
    
    return NewJointAnglesResponse(both_arms)

    
if __name__ == '__main__':
    '''
    rospy.init_node('follower_node')
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions',JointAngles ,queue_size=3)
    rospy.Subscriber('/robot/joint_states', JointState, callback_pose)
    #rospy.sleep(0.3)
    '''

    rospy.init_node('follower_node')

    config_file_name = 'second_victory.config' #rospy.get_param('config_file_name', default='second_victory.config')
    my_relaxedIK = RelaxedIK.init_from_config(config_file_name)
    num_chains = my_relaxedIK.vars.robot.numChains

    s = rospy.Service('new_joint_angles', relaxed_ik.srv.NewJointAngles, new_joint_angles)
    
    rospy.spin()
            
            
    '''
    for p in pose_goals:
    
    pos_x = p.position.x
    pos_y = p.position.y
    pos_z = p.position.z

    quat_w = p.orientation.w
    quat_x = p.orientation.x
    quat_y = p.orientation.y
    quat_z = p.orientation.z

    pos_goals.append([pos_x, pos_y, pos_z])
    quat_goals.append([quat_w, quat_x, quat_y, quat_z])
    
    ja = JointAngles()
    ja.header = header
    for x in xopt:
        ja.angles.append(Float32(x))
    angles_pub.publish(ja)
    '''
            
            
         



            

            
    







        
 



