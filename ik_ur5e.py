import numpy as np
from robot_data import robot # extract robot details from here
import utils as utils

def inverse_kinematics(angles, ref): # inputs are (1) angles and (2) <ref> points and orientation
    # extract <ref> details
    x_ref = ref[0]
    y_ref = ref[1]
    z_ref = ref[2]
    phi_ref = ref[3]
    theta_ref = ref[4]
    psi_ref = ref[5]

    # convert <euler> to <rot> and <quat>
    ref_euler = np.array([phi_ref, theta_ref, psi_ref]) # hold euler values
    ref_rot = utils.euler_to_rot(ref_euler)
    ref_quat = utils.euler_to_quat(ref_euler)

    