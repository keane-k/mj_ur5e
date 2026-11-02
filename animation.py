import numpy as np
from matplotlib import pyplot as plt
from fk_ur5e import quat_to_rotation, forward_kinematics
from robot_data import robot # we will extract robot details from here

def animate(robot):
    # extract <ee_quat>; needs to be processed with <quat_to_rotation>
    ee_quat = robot.misc.ee_quat

    # obtain <quat_to_rot> matrix; no <joint_axis> to handle
    ee_rotation = quat_to_rotation(ee_quat)

    # extract <ee_pos> 
    ee_pos = robot.misc.ee_pos


    # Initialise the 3D plot
    fig = plt.figure()
    axis = fig.add_subplot(111, projection='3d')

    # Indicate the axis(x,y,z)
    axis.set_xlabel('x')
    axis.set_ylabel('y')
    axis.set_zlabel('z')

    # Set the axis limits
    # axis.set_xlim(-3,-3) # way off, dont use
    # axis.set_ylim(-3,-3) # way off, dont use
    # axis.set_zlim(-3,-3) # way off, dont use
    axis.set_xlim([-1,1])
    axis.set_ylim([-1,1])
    axis.set_zlim([-1,1])
    

    # Animation Title
    axis.set_title('UR5e Animation')

    # Optional: Grid setting
    axis.grid(True)


    # temp placeholder to hold points; it will hold previous point, then get updated to current point, rinse & repeat
    temp = np.array([0,0,0]) # <trans> portion contains only 3 items

    # Plot; callback to <robot.body.items()>
    for i, body in robot.body.items():
        # extract <trans> (aka. <pos>) from <robot.body[i].final_hom> for each <body>
        current_pos = robot.body[i].final_hom[:3,3]

        # plot the robot link; draws a line from prev point to current point
        axis.plot([temp[0],current_pos[0]], [temp[1],current_pos[1]], [temp[2],current_pos[2]],
                  linewidth=6, color='blue')

        # rmbr to update <temp> to become <current_pos>
        temp = current_pos


        # rmbr to include <site> | end_effector 
        if i == len(robot.body):
            # extract <rot> and <trans>; we may need to recompute the <final_hom_transform> in <fk_ur5e.py> in here
            # <final_hom_transform> is essentially the end-effector pose
            #########################################################
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


            current_pos = final_hom_trans

            # plot the link to end_effector
            axis.plot([temp[0],current_pos[0]], [temp[1],current_pos[1]], [temp[2],current_pos[2]],
                  linewidth=4, color='red')


    # Configure aspect ratio; equal scaling for x,y,z axes
    axis.set_box_aspect([1,1,1]) 
    # plt.gca().set_aspect('equal')

    axis.view_init(elev=25, azim=42)


    origin = final_hom_trans
    rot = final_hom_rot

    arrow_length = 0.2

    # config x-direction
    direction_x = np.array([1,0,0])
    direction_x = rot.dot(direction_x)

    # config y-direction
    direction_y = np.array([0,1,0])
    direction_y = rot.dot(direction_y)

    # config z-direction
    direction_z = np.array([0,0,1])
    direction_z = rot.dot(direction_z)


    axis.quiver(origin[0],origin[1],origin[2],direction_x[0],direction_x[1],direction_x[2],
                length=arrow_length, arrow_length_ratio = .1,normalize=True,color='red')
    axis.quiver(origin[0],origin[1],origin[2],direction_y[0],direction_y[1],direction_y[2],
                length=arrow_length, arrow_length_ratio = .1,normalize=True,color='green')
    axis.quiver(origin[0],origin[1],origin[2],direction_z[0],direction_z[1],direction_z[2],
                length=arrow_length, arrow_length_ratio = .1,normalize=True,color='blue')
    axis.scatter(origin[0], origin[1], origin[2], color='orange', s=50, label='EndEff')


    # display plot
    plt.show()