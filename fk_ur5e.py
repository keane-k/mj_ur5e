import numpy as np
from types import SimpleNamespace
from robot_data import robot # we will extract robot details from here

# predefined matrices for different rotation axis
def axis_rotation(rotation_axis_id, angle):
    # assign your cos and sin values for easier callback; angle to be in rad
    c = np.cos(angle)
    s = np.sin(angle)

    # rotation about the x_axis; rotation_axis_id = 0
    if rotation_axis_id == 0:
        rotation_matrix = np.array([[1,0,0],
                                    [0,c,-s],
                                    [0,s,c]])

    # rotation about the y_axis; rotation_axis_id = 1
    elif rotation_axis_id == 1:
        rotation_matrix = np.array([[c,0,s],
                                    [0,1,0],
                                    [-s,0,c]])

    # rotation about the z_axis; rotation_axis_id = 2
    elif rotation_axis_id == 2:
        rotation_matrix = np.array([[c,-s,0],
                                    [s,c,0],
                                    [0,0,1]])

    # we conduct check if rotation_axis_id isn't 0, 1 or 2;
    else:
        raise ValueError("rotation_axis_id must only be 0(x), 1(y) or 2(z). No other values accepted.")

    return rotation_matrix


# convert quaternion to rotation
def quat_to_rotation(quat): 
    # extract the individual q0, q1, q2, q3 from <quat>
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]

    # compute quaternion rotation matrix; refer to <additional_notes.ipynb> for explanation
    quat_rot_mat = np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                             [2 * (q1 * q2 + q0 * q3), q0**2 - q1**2 + q2**2 - q3**2, 2 * (q2 * q3 - q0 * q1)],
                             [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0**2 - q1**2 - q2**2 + q3**2]])

    return quat_rot_mat

def forward_kinematics(angles):

    # what are some details we need to extract from robot_data?
    # selection: [body_id, parent, name, pos, quat, mass, ipos, iquat, inertia, joint_axis, joint_range]
    # <quat> to show how xyz changes from one frame to the other
    # <joint_axis> indicates which axis is the rotation axis
    for i in range(1, 7): # there's 6 bodies, starting from 1, ends at 6, rmbr <end + 1> 
                          # FYI: we start from 1 because MuJoCo sometimes define <ground> to be body[0]
        # extract <quat> for each body
        quat = robot.body[i].quat

        # extract <joint_axis> for each body
        joint_axis = robot.body[i].joint_axis
        # process <joint_axis> to reveal rotation axis
        rot_axis_id = np.argmax(np.abs(joint_axis)) # returns 0, 1 or 2; to be 'translated' to x, y or z

        # assign individual angle from angles (user-input in func) for each joint
        angle = angles[i-1] # rmbr that while body_id starts from 1-6, python(index) starts from 0-5

        # obtain the rotation matrix based on <rot_axis_id>, x, y or z rotation axis matrix; this would need to be combined with <quat> rotation later
        rot_mat = axis_rotation(rot_axis_id, angle)

        # set up storage point in <robot.body[body_id]> that stores:
        # obtain this link's (1) rotation and (2) position .wrt. its parent
        # we know <quat> is referenced .wrt. parent; do rmbr that we need to `quat @ rot_mat` to get the actual rotation matrix
        robot.body[i].rotation = quat_to_rotation(quat) @ rot_mat
        # position is just <pos> from xml file
        robot.body[i].position = robot.body[i].pos
        
        
        # create the homogeneous transform matrix (w/ rot and trans(pos))
        hom_transform = np.eye(4) # create a 4x4 identity (eye) matrix
        hom_transform[:3,:3] = robot.body[i].rotation
        hom_transform[:3,3] = robot.body[i].position.reshape(3)

        # rmbr to create a storage space to store all local <body> homogenous transform matrix
        robot.body[i].hom = hom_transform


    # =============================================================================
    # Do the same for <base>;
    # (1) extract <quat>, 
    # (2) obtain quat_to_rotation matrix,
    # (3) create the homoegenous tranf matrix for <base>
    # =============================================================================    
    # extract <quat>
    base_quat = robot.misc.base_quat

    # obtain quat_to_rot matrix
    base_rotation = quat_to_rotation(base_quat)

    # no <joint_axis> to handle, so we go straight to creating homogenous trf matrix
    # create the homogeneous transform matrix (w/ rot; there is no trans for <base>)
    base_hom_transform = np.eye(4) 
    base_hom_transform[:3,:3] = base_rotation

    # create a storage holder to store <base> homogeneous transform matrix (kinda optional but up to you)
    base_hom = base_hom_transform


    # =============================================================================
    # Combine what homogeneous transforms we have thus far 
    # to obtain the FINAL Homogeneous Transform (excluding end_effector)
    # <hom_0_1> @ <hom_1_2> @ <hom_2_3> @ <hom_3_4> @ <hom_4_5> @ <hom_5_6>
    # =============================================================================
    placeholder = base_hom # temp space to hold the current hom_transform matrix, will be updated later
    for i in range(1,7):
        robot.body[i].final_hom = placeholder @ robot.body[i].hom
        
        # update the placeholder to hold the latest homogeneous transform for the next wave
        placeholder = robot.body[i].final_hom 
    # at this point, <robot.body[6].final_hom> contains the Final Homogeneous Transform 
    

    # =============================================================================
    # Prep <site> (end effector) details; <quat> & <pos> 
    # Then obtain the FINAL Homogeneous Transform (including end_effector)
    # <hom_5_6> @ <hom_6_ee>
    # =============================================================================
    # extract <ee_quat>
    ee_quat = robot.misc.ee_quat

    # obtain <quat_to_rot> matrix; no <joint_axis> to handle
    ee_rotation = quat_to_rotation(ee_quat)

    # extract <ee_pos> 
    ee_pos = robot.misc.ee_pos

    # form the homogenous transformation matrix for <ee>
    ee_hom_transform = np.eye(4)
    ee_hom_transform[:3,:3] = ee_rotation
    ee_hom_transform[:3,3] = ee_pos
    
    # OPTIONAL, but I prefer to stick to the same naming convention of <name_hom>
    ee_hom = ee_hom_transform

    # Compute the Final Homogeneous Transform (including ee)
    final_hom_transform = robot.body[i].final_hom @ ee_hom

    # extract <rot> and <trans> from <final_hom_transform>
    final_hom_rot = final_hom_transform[:3,:3]
    final_hom_trans = final_hom_transform[:3,3]


    # using SimpleNamespace, return end-effector pose <final_hom_rot> & <final_hom_trans>
    soln = SimpleNamespace(
        final_hom_rot = final_hom_rot,
        final_hom_trans = final_hom_trans
        )   
    
    return soln, robot


