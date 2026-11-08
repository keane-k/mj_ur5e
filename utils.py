# =============================================================================
# Containment for mathematical calculations (funcs)
# =============================================================================
import numpy as np

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


# compute <rot_to_quat>
def rot_to_euler(rot):
    # bridge math and python callouts for Rotation Matrix; I'll do only for the shortest math expressions avail
    R11 = rot[0,0] # = cos(ψ)cos(θ)
    R12 = rot[0,1] # = -cos(θ)sin(ψ)
    R13 = rot[0,2] # = sin(θ)
    R23 = rot[1,2] # = -cos(θ)sin(φ)
    R33 = rot[2,2] # = cos(φ)cos(θ)

    # following math convention, (phi: φ --> x), (theta: θ --> y), (psi: ψ --> z)
    # determine φ, θ and ψ by solving backwards
    # ------------------
    # solving for θ
    theta = np.arcsin(R13)
    # now we know θ, just need to sub cos(θ) into other eqns to find the other eulers 
    cos_theta = np.cos(theta)
    ############################################
    # solving for φ, using arcsin()
    # phi = np.arcsin(R23/-cos_theta)

    # solving for φ, using arctan2(sine, cosine)
    phi_arctan2 = np.arctan2(-R23,R33)
    ############################################
    # solving for ψ, using arcsin()
    # psi = np.arcsin(R12/-cos_theta)

    # solving for φ, using arctan2(sine, cosine)
    psi_arctan2 = np.arctan2(-R12,R11)
    ############################################

    # return theta, phi_arctan2, psi_arctan2
    return np.array([phi_arctan2, theta, psi_arctan2])


# compute <rot_to_quat>
def rot_to_quat(rot):
    # Determine the magn of each q; goal is to find the largest magn for later
    q0 = np.sqrt(rot[0,0] + rot[1,1] + rot[2,2] + 1) / 2
    q1 = np.sqrt(rot[0,0] - rot[1,1] - rot[2,2] + 1) / 2
    q2 = np.sqrt(- rot[0,0] + rot[1,1] - rot[2,2] + 1) / 2
    q3 = np.sqrt(- rot[0,0] - rot[1,1] + rot[2,2] + 1) / 2

    # Determine the max magnitude amongst q0, q1, q2 and q3
    max_magn = max(q0, q1, q2, q3)

    if max_magn == q0:      # if q0 is the largest
        q1 = (rot[2, 1] - rot[1, 2]) / (4 * q0)
        q2 = (rot[0, 2] - rot[2, 0]) / (4 * q0)
        q3 = (rot[1, 0] - rot[0, 1]) / (4 * q0)
    elif max_magn == q1:    # if q1 is the largest
        q0 = (rot[2, 1] - rot[1, 2]) / (4 * q1)
        q2 = (rot[1, 0] + rot[0, 1]) / (4 * q1)
        q3 = (rot[0, 2] + rot[2, 0]) / (4 * q1)
    elif max_magn == q2:    # if q2 is the largest
        q0 = (rot[0, 2] - rot[2, 0]) / (4 * q2)
        q1 = (rot[1, 0] + rot[0, 1]) / (4 * q2)
        q3 = (rot[2, 1] + rot[1, 2]) / (4 * q2)
    else:                   # if q3 is largest
        q0 = (rot[1, 0] - rot[0, 1]) / (4 * q3)
        q1 = (rot[0, 2] + rot[2, 0]) / (4 * q3)
        q2 = (rot[2, 1] + rot[1, 2]) / (4 * q3)

    return np.array([q0, q1, q2 ,q3])


# compute <euler_to_rot>; input <euler> should be np.array([φ,θ,ψ])
def euler_to_rot(euler):
    phi = euler[0]
    theta = euler[1]
    psi = euler[2]

    Rx_phi = axis_rotation(0, phi)
    Ry_theta = axis_rotation(1, theta)
    Rz_psi = axis_rotation(2, psi)

    rot = Rx_phi @ Ry_theta @ Rz_psi

    return rot


# compute <euler_to_quat>; combi of <euler_to_rot> & <rot_to_quat>
# input <euler> should be np.array([φ,θ,ψ])
def euler_to_quat(euler):
    rot = euler_to_rot(euler)
    quat = rot_to_quat(rot)

    return quat # <rot_to_quat> returns np.array([q0,q1,q2,3])