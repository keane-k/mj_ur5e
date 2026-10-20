# =============================================================================
# Import your libraries
# =============================================================================
import numpy as np


# =============================================================================
# Using <class>, I will store impt properties referenced from <ur5e.xml>, these will go on to be used in FK later
# Refer to <python_prep> for my inputs on Classes and Objects
# =============================================================================
class Robot:
    class Body:
        # setup constructor for <class Body>; initialize it
        def __init__(self, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
            self.parent = parent
            self.name = name
            self.pos = np.array(pos)
            self.quat = np.array(quat)
            self.ipos = np.array(ipos)
            self.iquat = np.array(iquat)
            self.mass = mass
            self.inertia = np.array(inertia)
            self.joint_axis = np.array(joint_axis)
            self.joint_range = np.array(joint_range)

    # func to add body to robot
    def add_body(self, body_id, parent, name, pos, quat, mass, ipos, iquat, inertia, joint_axis, joint_range):
        self.body[body_id] = Robot.Body(parent, name, pos, quat, mass, ipos, iquat, inertia, joint_axis, joint_range)

    # class to contain (1) base and (2) end-effector/attachment site; anything that isn't a body
    class Misc:
        # setup constructor for <class Misc>; initialize it
        def __init__(self, ):

    # setup constructor for <class Robot>
    def __init__(self):
        self.body = {}





# =============================================================================
# Create our own 'Python' robot using data from MuJoCo's robot xml.
# Referencing back to <worldbody> in <ur5e.xml>, we simply copy and paste the values
# into our newly created <add_body()> within the <class Robot>

# Recall from <python_prep>, to use a class: 
# arm = RobotArm(3)  # create a new RobotArm object called `arm` with 3 joints
# arm.set_angle(0, 45)  # set Joint 0 to 45 degrees  ---> self.angles = [45, 0, 0]
# =============================================================================
# call the class; create an <object> from this <class>
robot = Robot()


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



# What's left that hasn't been added via robot.body() is base & site
# <!-- Joint 1 | Base -->
# <body name="base" quat="0 0 0 -1" childclass="ur5e">
#     <inertial mass="4.0" pos="0 0 0" diaginertia="0.00443333156 0.00443333156 0.0072"/>
#     <geom mesh="base_0" material="black" class="visual"/>
#     <geom mesh="base_1" material="jointgray" class="visual"/>
# <!-- Site -->
#     <site name="attachment_site" pos="0 0.1 0" quat="-1 1 0 0"/>
