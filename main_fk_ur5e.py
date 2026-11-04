import numpy as np
from robot_data import robot # we will extract robot details from here
from fk_ur5e import forward_kinematics
from animation import animate

angles = np.array([-1.5708, -1.5708, -1.5708, -1.5708, -1.5708, 0]) # expect 6 angles

# call forward_kinematics()
soln, robot = forward_kinematics(angles)

# call soln.<name> and assign them as <ee_rot> or whtvr name you want
ee_rot = soln.final_hom_rot
ee_pos = soln.final_hom_trans


print('Rotation of End-Effector:\n', ee_rot)
print('Position of End-Effector: ', ee_pos)


# Optional: compute <rot_to_quat>
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


# Optional: compute <rot_to_quat>
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


print('Rot to Euler:\n',rot_to_euler(ee_rot))
print('Rot to Quat:\n',rot_to_quat(ee_rot))


animate(robot)
