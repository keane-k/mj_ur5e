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
        rot_axis_id = np.argmax(joint_axis) # returns 0, 1 or 2; to be 'translated' to x, y or z

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
