# =============================================================================
# Import your libraries
# =============================================================================
import numpy as np

# func to compute the norm for <quat> and normalized it to get <quat_normalized>
# scale a quaternion (w,x,y,z) to have its length be exactly 1
def quat_normalize(q):
    q = np.asarray(q) # 'convert' to np.array
    assert q.shape[-1] == 4 # check if there's 4 components

    norm = np.linalg.norm(q, axis = -1, keepdims=True)
    q_normalized = q / norm
    return q_normalized


# =============================================================================
# <class Robot> to store impt properties (ie. bodies) referenced from <ur5e.xml>, these will go on to be used in FK later
# Refer to <python_prep> for inputs on Classes and Objects
# =============================================================================
class Robot:

    # =============================================================================
    # Body: to contain properties/parameters of main links of the arm (to be stored in self.body)
    # =============================================================================
    class Body:
        # setup constructor for <class Body>; initialize it
        def __init__(self, parent, name, pos, quat, mass, ipos, iquat, inertia, joint_axis, joint_range):
            self.parent = parent
            self.name = name
            self.pos = np.array(pos)
            self.quat = np.array(quat)
            self.mass = mass
            self.ipos = np.array(ipos)
            self.iquat = np.array(iquat)
            self.inertia = np.array(inertia)
            self.joint_axis = np.array(joint_axis)
            self.joint_range = np.array(joint_range)

    # func to add a body (creates a Robot.Body and files it under [body_id])
    def add_body(self, body_id, parent, name, pos, quat, mass, ipos, iquat, inertia, joint_axis, joint_range):
        self.body[body_id] = Robot.Body(parent, name, pos, quat, mass, ipos, iquat, inertia, joint_axis, joint_range)

    # =============================================================================
    # Misc: to contain properties/parameters of (1) base & (2) end-effector/attachment site (to be stored in self.misc)
    # =============================================================================
    # What's left that hasn't been added via robot.body() is base & site
        # <!-- Joint 1 | Base -->
        # <body name="base" quat="0 0 0 -1" childclass="ur5e">
        #     <inertial mass="4.0" pos="0 0 0" diaginertia="0.00443333156 0.00443333156 0.0072"/>
        #     <geom mesh="base_0" material="black" class="visual"/>
        #     <geom mesh="base_1" material="jointgray" class="visual"/>
        # <!-- Site -->
        #     <site name="attachment_site" pos="0 0.1 0" quat="-1 1 0 0"/>
    class Misc:
        # setup constructor for <class Misc>; initialize it
        # we are to grab <quat> for base, <pos> for EE, <quat> for EE; work this first, add anything else after
        def __init__(self, base_quat, ee_pos, ee_quat):
            # self.base_quat = np.array(base_quat)
            # self.ee_pos = np.array(ee_pos)
            self.ee_quat = np.array(ee_quat)

            self.base_quat = quat_normalize(np.array(base_quat))
            self.ee_quat = quat_normalize(np.array(ee_quat))

    # func to add misc (no id like [body_id] req. since there's only 1 base and 1 ee for UR5e)
    def add_misc(self, base_quat, ee_pos, ee_quat):
        self.misc = Robot.Misc(base_quat, ee_pos, ee_quat)

    

    # setup constructor for <class Robot>
    def __init__(self):
        self.body = {}


# call the class; create an <object> from this <class>
robot = Robot()

# =============================================================================
# BASE & END-EFFECTOR
# Create our own 'Python' robot using data from MuJoCo's robot xml.
# Referencing back to <worldbody> in <ur5e.xml>, we simply copy and paste the values
# into our newly created <add_misc()> within the <class Robot>

# Recall from <python_prep>, to use a class: 
# arm = RobotArm(3)  # create a new RobotArm object called `arm` with 3 joints
# arm.set_angle(0, 45)  # set Joint 0 to 45 degrees  ---> self.angles = [45, 0, 0]
# =============================================================================
# <!-- Joint 1 | Base -->
    # <body name="base" quat="0 0 0 -1" childclass="ur5e">
    #     <inertial mass="4.0" pos="0 0 0" diaginertia="0.00443333156 0.00443333156 0.0072"/>
    #     <geom mesh="base_0" material="black" class="visual"/>
    #     <geom mesh="base_1" material="jointgray" class="visual"/>
    # <!-- Site -->
    #     <site name="attachment_site" pos="0 0.1 0" quat="-1 1 0 0"/>
# #########################
# translate to <base_quat, ee_pos, ee_quat>
robot.add_misc(base_quat=[0,0,0,-1], ee_pos = [0, 0.1, 0], ee_quat = [-1, 1, 0, 0])


# =============================================================================
# BODIES
# Create our own 'Python' robot using data from MuJoCo's robot xml.
# Referencing back to <worldbody> in <ur5e.xml>, we simply copy and paste the values
# into our newly created <add_body()> within the <class Robot>

# Recall from <python_prep>, to use a class: 
# arm = RobotArm(3)  # create a new RobotArm object called `arm` with 3 joints
# arm.set_angle(0, 45)  # set Joint 0 to 45 degrees  ---> self.angles = [45, 0, 0]
# =============================================================================

# <!-- Joint 2 -->
# <body name="shoulder_link" pos="0 0 0.163">
#     <inertial mass="3.7" pos="0 0 0" diaginertia="0.0102675 0.0102675 0.00666"/>
#     <joint name="shoulder_pan_joint" class="size3" axis="0 0 1"/>
#     <geom mesh="shoulder_0" material="urblue" class="visual"/>
#     <geom mesh="shoulder_1" material="black" class="visual"/>
#     <geom mesh="shoulder_2" material="jointgray" class="visual"/>
#     <geom class="collision" size="0.06 0.06" pos="0 0 -0.04"/>
# #########################
# translate to <body_id, parent, name, pos, quat,
#              mass, ipos, iquat, inertia,
#              joint_axis, joint_range>
robot.add_body(body_id = 1, parent='base', name='shoulder_link', pos=[0,0,0.163], quat=[1,0,0,0],
               mass=3.7, ipos=[0,0,0], iquat=[1,0,0,0], inertia=[0.0102675,0.0102675,0.00666], 
               joint_axis=[0,0,1], joint_range=[-6.2831,6.2831])


# <!-- Joint 3 -->
# <body name="upper_arm_link" pos="0 0.138 0" quat="1 0 1 0">
#     <inertial mass="8.393" pos="0 0 0.2125" diaginertia="0.133886 0.133886 0.0151074"/>
#     <joint name="shoulder_lift_joint" class="size3"/>
#     <geom mesh="upperarm_0" material="linkgray" class="visual"/>
#     <geom mesh="upperarm_1" material="black" class="visual"/>
#     <geom mesh="upperarm_2" material="jointgray" class="visual"/>
#     <geom mesh="upperarm_3" material="urblue" class="visual"/>
#     <geom class="collision" pos="0 -0.04 0" quat="1 1 0 0" size="0.06 0.06"/>
#     <geom class="collision" size="0.05 0.2" pos="0 0 0.2"/>
# #########################
# translate to <body_id, parent, name, pos, quat,
#              mass, ipos, iquat, inertia,
#              joint_axis, joint_range>
robot.add_body(body_id = 2, parent='shoulder_link', name='upper_arm_link', pos=[0,0.138,0], quat=[1,0,1,0],
               mass=8.393, ipos=[0,0,0.2125], iquat=[1,0,0,0], inertia=[0.133886,0.133886,0.0151074], 
               joint_axis=[0,1,0], joint_range=[-6.2831,6.2831])


# <!-- Joint 4 -->
# <body name="forearm_link" pos="0 -0.131 0.425">
#     <inertial mass="2.275" pos="0 0 0.196" diaginertia="0.0311796 0.0311796 0.004095"/>
#     <joint name="elbow_joint" class="size3_limited"/>
#     <geom mesh="forearm_0" material="urblue" class="visual"/>
#     <geom mesh="forearm_1" material="linkgray" class="visual"/>
#     <geom mesh="forearm_2" material="black" class="visual"/>
#     <geom mesh="forearm_3" material="jointgray" class="visual"/>
#     <geom class="collision" pos="0 0.08 0" quat="1 1 0 0" size="0.055 0.06"/>
#     <geom class="collision" size="0.038 0.19" pos="0 0 0.2"/>
# #########################
# translate to <body_id, parent, name, pos, quat,
#              mass, ipos, iquat, inertia,
#              joint_axis, joint_range>
robot.add_body(body_id = 3, parent='upper_arm_link', name='forearm_link', pos=[0,-0.131,0.425], quat=[1,0,0,0],
               mass=2.275, ipos=[0,0,0.196], iquat=[1,0,0,0], inertia=[0.0311796,0.0311796,0.004095], 
               joint_axis=[0,1,0], joint_range=[-3.1415,3.1415])


# <!-- Joint 5 -->
# <body name="wrist_1_link" pos="0 0 0.392" quat="1 0 1 0">
#     <inertial mass="1.219" pos="0 0.127 0" diaginertia="0.0025599 0.0025599 0.0021942"/>
#     <joint name="wrist_1_joint" class="size1"/>
#     <geom mesh="wrist1_0" material="black" class="visual"/>
#     <geom mesh="wrist1_1" material="urblue" class="visual"/>
#     <geom mesh="wrist1_2" material="jointgray" class="visual"/>
#     <geom class="collision" pos="0 0.05 0" quat="1 1 0 0" size="0.04 0.07"/>
# #########################
# translate to <body_id, parent, name, pos, quat,
#              mass, ipos, iquat, inertia,
#              joint_axis, joint_range>
robot.add_body(body_id = 4, parent='forearm_link', name='wrist_1_link', pos=[0,0,0.392], quat=[1,0,1,0],
               mass=1.219, ipos=[0,0.127,0], iquat=[1,0,0,0], inertia=[0.0025599,0.0025599,0.0021942], 
               joint_axis=[0,1,0], joint_range=[-6.2831,6.2831])


# <!-- Joint 6 -->
# <body name="wrist_2_link" pos="0 0.127 0">
#     <inertial mass="1.219" pos="0 0 0.1" diaginertia="0.0025599 0.0025599 0.0021942"/>
#     <joint name="wrist_2_joint" axis="0 0 1" class="size1"/>
#     <geom mesh="wrist2_0" material="black" class="visual"/>
#     <geom mesh="wrist2_1" material="urblue" class="visual"/>
#     <geom mesh="wrist2_2" material="jointgray" class="visual"/>
#     <geom class="collision" size="0.04 0.06" pos="0 0 0.04"/>
#     <geom class="collision" pos="0 0.02 0.1" quat="1 1 0 0" size="0.04 0.04"/>
# #########################
# translate to <body_id, parent, name, pos, quat,
#              mass, ipos, iquat, inertia,
#              joint_axis, joint_range>
robot.add_body(body_id = 5, parent='wrist_1_link', name='wrist_2_link', pos=[0,0.127,0], quat=[1,0,0,0],
               mass=1.219, ipos=[0,0,0.1], iquat=[1,0,0,0], inertia=[0.0025599,0.0025599,0.0021942], 
               joint_axis=[0,0,1], joint_range=[-6.2831,6.2831])


# <!-- Joint 7 -->
# <body name="wrist_3_link" pos="0 0 0.1">
#     <inertial mass="0.1889" pos="0 0.0771683 0" quat="1 0 0 1"
#     diaginertia="0.000132134 9.90863e-05 9.90863e-05"/>
#     <joint name="wrist_3_joint" class="size1"/>
#     <geom material="linkgray" mesh="wrist3" class="visual"/>
#     <geom class="eef_collision" pos="0 0.08 0" quat="1 1 0 0" size="0.04 0.02"/>
# #########################
# translate to <body_id, parent, name, pos, quat,
#              mass, ipos, iquat, inertia,
#              joint_axis, joint_range>
robot.add_body(body_id = 6, parent='wrist_2_link', name='wrist_3_link', pos=[0,0,0.1], quat=[1,0,0,0],
               mass=0.1889, ipos=[0,0.0771683,0], iquat=[1,0,0,1], inertia=[0.000132134,9.90863e-05,9.90863e-05], 
               joint_axis=[0,1,0], joint_range=[-6.2831,6.2831])




