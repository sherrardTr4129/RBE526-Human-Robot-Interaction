from ..Utils.colors import *

from ..GROOVE.GROOVE_Utils.objective import Objective, get_groove_global_vars, objective_master
from ..Utils import tf_fast as Tf
from ..Utils.geometry_utils import *
from ..Utils.joint_utils import *

# try:
#     from boost import objectives_ext
# except:
#     print 'ERROR when importing boost library extension.  Defaulting to python implementation (which will be slower).  ' \
#           'To get speed boost, please install and configure the boost python library: ' \
#           'https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html'


def objective_master_relaxedIK(x):
    vars = get_groove_global_vars()
    
    ################################# Overwrite Joints #################################
    # This is where the joints are actually overwritten
    # x is the hypothetical joint state vector which the solver has proposed
    # We are replacing the elements of x that we know to be fixed, or controlled by the gamepad
    # We know that the values of x are in joint_order order, so we find the name of the joints we need to replace, then replace the corresponding index of x
    for i,name in enumerate(vars.overwrite_joints):
        index = vars.joint_order.index(name)
        x[index] = vars.overwrite_joint_values[i]
    #####################################################################################
    
    vars.frames = vars.robot.getFrames(x)

    return objective_master(x)

########################################################################################################################
# Define our camera location goal objectives here

class Arm0_Look_At_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Look_At'

    def __call__(self, x, vars):
        arm0_chain = vars.frames[0]    # index determines which chain to get (arm0 = camera arm)
        arm0_positions = arm0_chain[0] # each chain is a tuple of (positions, rotations)
        arm0_rotations = arm0_chain[1]
        camPos = arm0_positions[-1]    # positions is an array of 1 x 3 arrays, each an [x,y,z] position of a link in the chain
                                       # The last link is the position of the camera 
                                       # on 9 Jan, found that it's a little further than right_hand. How is this calculated? 
                                       # I compared the value here at init_state with the value shown in RVIZ in urdf_viewer.launch. 
                                       # Should they be the same? They aren't.
        #print 'trying to look at location:'
        #print camPos
        
        focal_length = 1               # This is the length of the "viewing segment", which is the area we want the manipulation hand to be at
                                       # This objective returns the same score no matter where along this viewing segment the manipulation hand is located
                                       
        camFrame = arm0_rotations[-1]  # rotations is an array of 3x3 matrices that represent the rotation of a link in the chain w.r.t. the base
        
        # This endpoint defines the viewing segment as a segment of length focal_length extending along the z-axis of the camera link
        camEndpoint = camPos + focal_length*(camFrame[:,2]/np.linalg.norm(camFrame[:,2]))
        
        
        arm1_chain = vars.frames[1]    # index determines which chain to get (arm1 = manipulation arm)
        arm1_positions = arm1_chain[0] # each chain is a tuple of (positions, rotations)
        arm1_rotations = arm1_chain[1]
        manPos = arm1_positions[-1]    # positions is an array of 1 x 3 arrays, each an [x,y,z] position of a link in the chain
        
        gripper_length = 0.15               # This is the length of the "viewing segment", which is the area we want the manipulation hand to be at
                                       # This objective returns the same score no matter where along this viewing segment the manipulation hand is located
                                       
        manFrame = arm1_rotations[-1]  # rotations is an array of 3x3 matrices that represent the rotation of a link in the chain w.r.t. the base
        
        # This endpoint defines the viewing segment as a segment of length focal_length extending along the z-axis of the camera link
        manEndpoint = manPos + gripper_length*(manFrame[:,2]/np.linalg.norm(manFrame[:,2]))

        # Distance from a point to a line
        # Set A = camera position, B = manipulator position, C = endpoint of viewing segment
        # distance of point B from segment AC  = |AB X BC| / |AC|
        endpoint_dis = np.subtract(manEndpoint,camEndpoint)  
        inv_endpoint_dis = np.subtract(camEndpoint, manEndpoint)
        between_hands = np.subtract(manEndpoint,camPos)
        camera_axis = np.subtract(camEndpoint, camPos)
        inv_camera_axis = np.subtract(camPos, camEndpoint)

        x_val = np.linalg.norm(np.cross(between_hands, endpoint_dis))/ focal_length
        check_close = np.dot(camera_axis, between_hands)/(focal_length * np.linalg.norm(between_hands))
        check_far = np.dot(inv_camera_axis, endpoint_dis)/(focal_length * np.linalg.norm(inv_endpoint_dis))
        
        if check_close > 0 and check_far > 0:
            pass
        elif check_close <= 0 and check_far > 0:
            x_val =  np.linalg.norm(between_hands)
        elif check_close > 0 and check_far <= 0:
            x_val = np.linalg.norm(inv_endpoint_dis)
        
        # print x_val
        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g
        
class Arm0_High(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Arm_High'

    def __call__(self, x, vars):
        arm0_chain = vars.frames[0]    # index determines which chain to get (arm0 = camera arm)
        arm0_positions = arm0_chain[0] # each chain is a tuple of (positions, rotations)
        camPos = arm0_positions[-1]    # this is the [x,y,z] position of the camera
        
        x_val = camPos[2]
        # print x_val
        t = 0.5 # ideal value of x_val
        d = 2.0 # power of exponential numerator, keep at 2 
        c = .2 # exponential denominator, determines width of reward region
        f = 0.2 # determines width of transitional polynomial
        g = 2 # power of transitional polynomial
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g
        
class Roll_Limit(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Roll_Limit'

    def __call__(self, x, vars):    
        arm0_chain = vars.frames[0]    # index determines which chain to get (arm0 = camera arm)
        arm0_rotations = arm0_chain[1]
        camFrame = arm0_rotations[-1]
        if np.dot(camFrame[:,0], [0,0,1]) < 0:
            bonus_score = 100
        else:
            bonus_score = 0
        x_val = np.dot(camFrame[:,1], [0,0,1]) + bonus_score
        # print x_val
        t = 0.0 # ideal value of x_val
        d = 2.0 # power of exponential numerator, keep at 2 
        c = 0.1 # exponential denominator, determines width of reward region
        f = 1.0 # determines width of transitional polynomial
        g = 2 # power of transitional polynomial
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g    
        
class Away_From_Head(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Away_From_Head'

    def __call__(self, x, vars):    
        arm0_chain = vars.frames[0]    # index determines which chain to get (arm0 = camera arm)
        arm0_rotations = arm0_chain[1]
        camFrame = arm0_rotations[-1]
        if np.dot(camFrame[:,2], [0,1,0]) < 0:
            bonus_score = 100
        else:
            bonus_score = 0
        
        x_val = np.dot(camFrame[:,2], [0,1,0]) + bonus_score
        # print x_val
        t = 1.0 # ideal value of x_val
        d = 2.0 # power of exponential numerator, keep at 2 
        c = 0.1 # exponential denominator, determines width of reward region
        f = 1.0 # determines width of transitional polynomial
        g = 2 # power of transitional polynomial
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g    
        
class Dist_To_Target(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Distance_To_Target'

    def __call__(self, x, vars):    
        arm0_chain = vars.frames[0]    # index determines which chain to get (arm0 = camera arm)
        arm0_positions = arm0_chain[0] # each chain is a tuple of (positions, rotations)
        camPos = arm0_positions[-1]    # this is the [x,y,z] position of the camera
        
        arm1_chain = vars.frames[1]    # index determines which chain to get (arm1 = manipulation arm)
        arm1_positions = arm1_chain[0] # each chain is a tuple of (positions, rotations)
        arm1_rotations = arm1_chain[1]
        manPos = arm1_positions[-1]    # positions is an array of 1 x 3 arrays, each an [x,y,z] position of a link in the chain
        
        diff = manPos - camPos
        norm_ord = 2
        x_val = np.linalg.norm(diff, ord=norm_ord)
        t = 0.6 # ideal value of x_val
        d = 2.0 # power of exponential numerator, keep at 2 
        c = 0.1 # exponential denominator, determines width of reward region
        f = 10.0 # determines width of transitional polynomial
        g = 2 # power of transitional polynomial
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g 
#########################################################################################################################

class Position_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Position'

    def __call__(self, x, vars):
        # positions = vars.arm.getFrames(x)[0]
        arm_num = 1
        positions = vars.frames[arm_num][0] # This line returns an array of [x,y,z] positions, one position for each robot joint including origin and ee     
        eePos = positions[-1]
        goal_pos = vars.goal_positions[arm_num]
        diff = (eePos - goal_pos)
        norm_ord = 2
        x_val = np.linalg.norm(diff, ord=norm_ord)
        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Position_MultiEE_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Position_MultiEE'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.position_multiEE_obj(vars.frames, vars.goal_positions, [1.0, 1.0])
        else:
            x_val_sum = 0.0

            for i, f in enumerate(vars.frames):
                positions = f[0]
                eePos = positions[-1]
                goal_pos = vars.goal_positions[i]
                diff = (eePos - goal_pos)
                norm_ord = 2
                x_val = np.linalg.norm(diff, ord=norm_ord)
                x_val_sum += x_val

            x_val = x_val_sum

        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2

        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Orientation_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Orientation'

    def __call__(self, x, vars):
        arm_num = 1
        frames = vars.frames[arm_num][1]
        eeMat = frames[-1]

        goal_quat = vars.goal_quats[arm_num]
        new_mat = np.zeros((4, 4))
        new_mat[0:3, 0:3] = eeMat
        new_mat[3, 3] = 1

        ee_quat = Tf.quaternion_from_matrix(new_mat)

        q = ee_quat
        ee_quat2 = [-q[0],-q[1],-q[2],-q[3]]

        norm_ord = 2
        # start = time.time()
        disp = np.linalg.norm(Tf.quaternion_disp(goal_quat,ee_quat), ord=norm_ord)
        disp2 = np.linalg.norm(Tf.quaternion_disp(goal_quat,ee_quat2),ord=norm_ord)
        # after = time.time()
        # print after - start

        x_val = min(disp, disp2)
        # x_val = np.min(np.array([disp,disp2]))
        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Orientation_MultiEE_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Orientation_MultiEE'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.orientation_multiEE_obj(vars.frames, vars.goal_quats, [1.0, 1.0])
        else:
            x_val_sum = 0.0

            for i, f in enumerate(vars.frames):
                eeMat = f[1][-1]

                goal_quat = vars.goal_quats[i]
                new_mat = np.zeros((4, 4))
                new_mat[0:3, 0:3] = eeMat
                new_mat[3, 3] = 1

                ee_quat = Tf.quaternion_from_matrix(new_mat)

                q = ee_quat
                ee_quat2 = [-q[0], -q[1], -q[2], -q[3]]

                norm_ord = 2
                disp = np.linalg.norm(Tf.quaternion_disp(goal_quat, ee_quat), ord=norm_ord)
                disp2 = np.linalg.norm(Tf.quaternion_disp(goal_quat, ee_quat2), ord=norm_ord)

                x_val = min(disp, disp2)
                x_val_sum += x_val

            x_val = x_val_sum

        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g



class Min_Jt_Vel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_Jt_Vel'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.min_jt_vel_obj(x, vars.xopt)
        else:
            v = x - np.array(vars.xopt)
            x_val = np.linalg.norm(v)

        t = 0.0
        d = 2.0
        c = .1
        f = 10.0
        g = 2

        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g



class Min_EE_Vel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_EE_Vel'

    def __call__(self, x, vars):
        jtPt = vars.frames[0][-1]
        x_val = np.linalg.norm(vars.ee_pos - jtPt)
        t = 0.0
        d = 2.0
        c = .1
        f = 10.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Min_Jt_Accel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_Jt_Accel'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.min_jt_accel_obj(x, vars.xopt, vars.prev_state)
        else:
            prev_state_2 = np.array(vars.prev_state)
            prev_state = np.array(vars.xopt)

            v2 = prev_state - prev_state_2
            v1 = x - prev_state

            a = v2 - v1

            x_val = np.linalg.norm(a)

        t = 0.0
        d = 2.0
        c = .1
        f = 10.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Min_EE_Accel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_EE_Accel'

    def __call__(self, x, vars):
        jtPt = vars.frames[0][-1]
        prev_jtPt_2 = np.array(vars.prev_ee_pos)
        prev_jtPt = np.array(vars.ee_pos)

        v2 = prev_jtPt - prev_jtPt_2
        v1 = jtPt - prev_jtPt

        a = v2 - v1

        x_val = np.linalg.norm(a)
        t = 0.0
        d = 2.0
        c = .2
        f = 0.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Min_Jt_Jerk_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_Jt_Jerk'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.min_jt_jerk_obj(x, vars.xopt, vars.prev_state, vars.prev_state2)
        else:
            prev_state_3 = np.array(vars.prev_state2)
            prev_state_2 = np.array(vars.prev_state)
            prev_state = np.array(vars.xopt)

            v3 = prev_state_2 - prev_state_3
            v2 = prev_state - prev_state_2
            v1 = x - prev_state

            a2 = v2 - v3
            a1 = v1 - v2

            j = a1 - a2

            x_val = np.linalg.norm(j)

        t = 0.0
        d = 2.0
        c = .2
        f = 0.0
        g = 2

        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Min_EE_Jerk_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_EE_Jerk'

    def __call__(self, x, vars):
        jtPt = vars.frames[0][-1]
        prev_jtPt_3 = np.array(vars.prev_ee_pos2)
        prev_jtPt_2 = np.array(vars.prev_ee_pos)
        prev_jtPt = np.array(vars.ee_pos)

        v3 = prev_jtPt_2 - prev_jtPt_3
        v2 = prev_jtPt - prev_jtPt_2
        v1 = jtPt - prev_jtPt

        a2 = v2 - v3
        a1 = v1 - v2

        j = a1 - a2

        x_val = np.linalg.norm(j)
        t = 0.0
        d = 2.0
        c = .2
        f = 1.0
        g = 2
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2))) + f * (x_val - t) ** g


class Joint_Limit_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Joint_Limit'

    def __call__(self, x, vars):
        sum = 0.0
        penalty = 50.0
        d = 8
        joint_limits = vars.robot.bounds
        for i in xrange(vars.robot.numDOF):
            l = joint_limits[i][0]
            u = joint_limits[i][1]
            mid = (u + l) / 2.0
            a = penalty / (u - mid)**d
            sum += a*(x[i] - mid)**d

        vars.joint_limit_obj_value = sum

        x_val = sum
        t = 0
        d = 2
        c = 2.3
        f = .003
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Self_Collision_Avoidance_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Self_Collision_Avoidance'

    def __call__(self, x, vars):
        frames = vars.frames
        jt_pts = frames[0]

        x_val = vars.collision_graph.get_collision_score(frames)
        t = 0.0
        d = 2.0
        c = .08
        f = 1.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Collision_Avoidance_nn(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Collision_Avoidance_nn'

    def __call__(self, x, vars):
        frames = vars.frames
        out_vec = []
        for f in frames:
            jt_pts = f[0]
            for j in jt_pts:
                out_vec.append(j[0])
                out_vec.append(j[1])
                out_vec.append(j[2])

        val = vars.collision_nn.predict([out_vec])[0]

        # nn_stats = vars.nn_stats

        # x_val =  (val - nn_stats[0])/ nn_stats[1]
        x_val = val
        t = 0
        d = 2
        c = 1.85
        f = .004
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

        # return math.exp(x_val - 0.64) - 1
