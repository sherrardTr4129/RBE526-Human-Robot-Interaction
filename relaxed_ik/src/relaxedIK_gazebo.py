#! /usr/bin/env python
'''

'''
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name
from RelaxedIK.relaxedIK import RelaxedIK
# from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import roslaunch
import os
import tf
import math

rospy.init_node('relaxedIK_gazebo')

left_js_pub = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size=1)
right_js_pub = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size=1)

def moveJoint(jointcmds, prefix='right', nbJoints=7):
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, nbJoints):
        jointCmd.joint_names.append(prefix + 'joint_' + str(i + 1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    rate = rospy.Rate(100)
    count = 0
    while (count < 5):
        right_js_pub.publish(jointCmd)
        count = count + 1
        rate.sleep()

def make_JointTrajectory_from_joints(vals=[],names=[]):
    if not len(vals) == len(names):
        print("ERROR: length miss match")
        return None

    JT_msg = JointTrajectory()
    JT_msg.joint_names = names



    JTP_msg = JointTrajectoryPoint()
    JT_msg.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    JTP_msg.time_from_start = rospy.Duration.from_sec(5.0)
    # JTP_msg.time_from_start = 0.0

    JTP_msg.positions = vals
    JTP_msg.velocities = [0.0]*len(names)
    JTP_msg.accelerations = [0.0]*len(names)
    JTP_msg.effort = [0.0]*len(names)

    JT_msg.points.append(JTP_msg)



    return JT_msg


if __name__ == '__main__':
    # Don't change this code####################################################################################


    ####################################################################################################################
    relaxedIK = RelaxedIK.init_from_config(config_file_name)
    ####################################################################################################################

    # urdf_file = open(relaxedIK.vars.urdf_path, 'r')
    # urdf_string = urdf_file.read()
    # rospy.set_param('robot_description', urdf_string)
    # js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
    # ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)
    # tf_pub = tf.TransformBroadcaster()
    #
    # rospy.sleep(0.3)

    # Don't change this code ###########################################################################################
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch_path = os.path.dirname(__file__) + '/../launch/robot_state_pub.launch'
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    # launch.start()
    ####################################################################################################################

    rospy.sleep(1.0)

    counter = 0.0
    stride = 0.08
    idx = 0

    js = JointState()
    js.name = joint_ordering
    # js.position = [-0.296825, -0.88549, 1.14512, 1.59227, -0.622029, 1.08337, -3.01887, -0.0636602, -0.944932, -1.19574,
    #                1.71614, 0.674952, 1.01434, 2.47738]
    js.position = [0.0] * 14
    while not rospy.is_shutdown():
        c = math.cos(counter)
        #c = abs(c)
        s = 0.9
        num_ee = relaxedIK.vars.robot.numChains
        goal_pos = []
        goal_quat = []
        # j_val = raw_input()
        # j_val = float(j_val)
        goal_pos.append([c*s, 0, 0])
        for i in range(num_ee):
            goal_quat.append([1, 0, 0, 0])
            if i == 0:
                continue
            else:
                goal_pos.append([0, 0, 0])

        #        overwrite_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        # overwrite_joints = ['leftjoint_1', 'leftjoint_2', 'leftjoint_3', 'leftjoint_4', 'leftjoint_5', 'leftjoint_6',
        #                     'leftjoint_7']
        overwrite_joints = ['leftjoint_1', 'leftjoint_2', 'leftjoint_3', 'leftjoint_4', 'leftjoint_5', 'leftjoint_6',
                            'leftjoint_7']
        overwrite_joint_values = js.position[7:]
        # overwrite_joint_values[0] = c*0.3 +0.5
        overwrite_joint_values[1] = c*0.5 +0.5

        vals = [-1.8849555921538759, 1.16, -1.95, 1.92, -1.7, 1.78, -1.76]


        moveJoint(overwrite_joint_values,prefix='right')
        ls_msg = make_JointTrajectory_from_joints(overwrite_joint_values,overwrite_joints)
        left_js_pub.publish(ls_msg)
        rospy.sleep(0.1)
        # raw_input("input")
        counter+=stride
        continue



        xopt = relaxedIK.solve(goal_pos, goal_quat, overwrite_joints, overwrite_joint_values)
        # xopt = relaxedIK.solve(goal_pos, goal_quat)
        js = joint_state_define(xopt)
        # print("goal_pos",goal_pos)
        # print("xopt",xopt)
        if js == None:
            js = JointState()
            js.name = joint_ordering
            for x in xopt:
                js.position.append(x)
        now = rospy.Time.now()
        js.header.stamp.secs = now.secs
        js.header.stamp.nsecs = now.nsecs
        js_pub.publish(js)
        # print("js",js)

        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.header.seq = idx
        for i in range(num_ee):
            p = Pose()
            curr_goal_pos = goal_pos[i]
            curr_goal_quat = goal_quat[i]
            p.position.x = curr_goal_pos[0]
            p.position.y = curr_goal_pos[1]
            p.position.z = curr_goal_pos[2]

            p.orientation.w = curr_goal_quat[0]
            p.orientation.x = curr_goal_quat[1]
            p.orientation.y = curr_goal_quat[2]
            p.orientation.z = curr_goal_quat[3]
            ee_pose_goals.ee_poses.append(p)

        ee_pose_goals_pub.publish(ee_pose_goals)
        # print(ee_pose_goals)

        tf_pub.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             fixed_frame)

        idx += 1
        counter += stride
        rospy.sleep(0.1)
