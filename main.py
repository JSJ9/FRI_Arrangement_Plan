#https://github.com/ChenEating716/pybullet-URDF-models --for table and random objects
#https://github.com/robot-descriptions/awesome-robot-descriptions?tab=readme-ov-file -- sawyer, we might not
#need it tho because we are only using pybullet to get valid pointclouds
#https://github.com/RethinkRobotics/sawyer_robot.git
# import pybullet as p
# from time import sleep
# import pybullet_data

# # def get_camera_params():
# #     width = 640  # Example width, adjust as needed
# #     height = 480  # Example height, adjust as needed
# #     view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0],
# #                                                       distance=2,
# #                                                       yaw=0,
# #                                                       pitch=-45,
# #                                                       roll=0,
# #                                                       upAxisIndex=2)
# #     projection_matrix = p.computeProjectionMatrixFOV(fov=60,  # Example FOV, adjust as needed
# #                                                      aspect=width / height,
# #                                                      nearVal=0.01,
# #                                                      farVal=100)
# #     return width, height, view_matrix, projection_matrix

# def main():
#     physicsClient = p.connect(p.GUI)
#     p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())


#     planeId = p.loadURDF("plane.urdf")
#     cubeStartPos = [0, 0, 1]
#     #cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
#     tableId = p.loadURDF("table/table.urdf")
#     boxID = p.loadURDF("cube.urdf", [0,0,5],  globalScaling=.1)
#     #cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#     p.setGravity(0, 0, -10)
#     useRealTimeSimulation = 0

#     if (useRealTimeSimulation):
#         p.setRealTimeSimulation(1)

#     while 1:
#         if (useRealTimeSimulation):
#             p.setGravity(0, 0, -10)
#             sleep(0.01)  # Time in seconds.
#         else:
#             p.stepSimulation()

# if __name__ == "__main__":
#     main()   
    
# """..................................................................
# ...................................................................
# ....................................................................."""
# import time
# import pybullet as p
# import pybullet_data
# import numpy as np
# import open3d as o3d

# def get_camera_params():
#     width = 640  # Example width, adjust as needed
#     height = 480  # Example height, adjust as needed
#     view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0],
#                                                       distance=1.5,
#                                                       yaw=0,
#                                                       pitch=-90,
#                                                       roll=0,
#                                                       upAxisIndex=2)
#     projection_matrix = p.computeProjectionMatrixFOV(fov=60,  # Example FOV, adjust as needed
#                                                      aspect=width / height,
#                                                      nearVal=0.01,
#                                                      farVal=100)
#     return width, height, view_matrix, projection_matrix


# def get_point_cloud(width, height, view_matrix, proj_matrix):
#     # Get a depth image
#     image_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
#     depth = image_arr[3]
#     print(depth)

#     # Create a grid with pixel coordinates and depth values
#     y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
#     y *= -1.
#     x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
#     h = np.ones_like(z)
#     pixels = np.stack([x, y, z, h], axis=1)
    
#     # Filter out "infinite" depths
#     pixels = pixels[z < 0.99]
#     pixels[:, 2] = 2 * pixels[:, 2] - 1

#     # Reshape matrices for matrix multiplication
#     proj_matrix = np.asarray(proj_matrix).reshape((4, 4))
#     view_matrix = np.asarray(view_matrix).reshape((4, 4))

#     # Transform pixels to world coordinates
#     tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))
#     points = np.matmul(tran_pix_world, pixels.T).T
#     points /= points[:, 3: 4]
#     points = points[:, :3]

#     # Create an Open3D point cloud
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     print(pcd)

#     return pcd

# # initialize the GUI and others
# def main():
#     p.connect(p.GUI)
#     p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
#     p.setTimeStep(1 / 240.)
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())
#     p.setGravity(0,0,-9.8)

#     # Load table and plane
#     # p.loadURDF("plane.urdf",[0,0,0])
#     # p.loadURDF("table/table.urdf", useFixedBase = True)
#     # p.loadURDF("cube.urdf", [0,0,5],  globalScaling=.1)
#     # p.loadURDF("xarm/xarm6_with_gripper.urdf", [.5,0,.5])

        
#     useFixedBase = True
#     flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

#     #plane_pos = [0,0,0]
#     #plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
#     table_pos = [0,0,-0.625]
#     table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
#     xarm = p.loadURDF("xarm/xarm6_with_gripper.urdf", flags = flags, useFixedBase=useFixedBase)
#     p.loadURDF("cube.urdf", [0,0,5],  globalScaling=.1)

#     jointIds = []
#     paramIds = []

#     for j in range(p.getNumJoints(xarm)):
#         p.changeDynamics(xarm, j, linearDamping=0, angularDamping=0)
#         info = p.getJointInfo(xarm, j)
#         #print(info)
#         jointName = info[1]
#         jointType = info[2]
#     if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
#         jointIds.append(j)
#         paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
    
#     skip_cam_frames = 10  

#     while (1):
#         p.stepSimulation()
#         for i in range(len(paramIds)):
#             c = paramIds[i]
#             targetPos = p.readUserDebugParameter(c)
#             p.setJointMotorControl2(xarm, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
#         skip_cam_frames -= 1
#         if (skip_cam_frames<0):
#             p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL )
#             skip_cam_frames = 10
#         # point_cloud = get_point_cloud(width, height, view_matrix, projection_matrix)
#         # print(type(point_cloud))
        
#         # # Visualize point cloud using Open3D
#         # o3d.visualization.draw_geometries([point_cloud])

#         time.sleep(1./240.)
	

    
    # # Simulation parameters
    # max_steps = 1000  # Number of simulation steps before displaying point cloud
    # current_step = 0

    # Main simulation loop
    # while True:
    # # Get camera parameters
    #     width, height, view_matrix, projection_matrix = get_camera_params()

    #     # Perform simulation step
    #     p.stepSimulation()

    #     current_step += 1

    #     # # Generate point cloud after a certain number of steps
    #     # if current_step >= max_steps:
    #     #     # Reset current step count
    #     #     current_step = 0

    #     # Generate point cloud
    #     point_cloud = get_point_cloud(width, height, view_matrix, projection_matrix)
    #     print(type(point_cloud))
        
    #     # Visualize point cloud using Open3D
    #     o3d.visualization.draw_geometries([point_cloud])

# if __name__ == "__main__":
#     main()

# ---------------------------------------------------------------------
# ----------------------------------------------------------------------
# -------------------------------------------------------------------------

# # NEW CODE BELOW
# import pybullet as p
# import pybullet_data
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# def get_camera_params():
#     width = 640  # Example width, adjust as needed
#     height = 480  # Example height, adjust as needed
#     view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0],
#                                                       distance=2,
#                                                       yaw=0,
#                                                       pitch=-45,
#                                                       roll=0,
#                                                       upAxisIndex=2)
#     projection_matrix = p.computeProjectionMatrixFOV(fov=60,  # Example FOV, adjust as needed
#                                                      aspect=width / height,
#                                                      nearVal=0.01,
#                                                      farVal=100)
#     return width, height, view_matrix, projection_matrix


# def get_point_cloud(width, height, view_matrix, proj_matrix):
#     # Get a depth image
#     image_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
#     depth = image_arr[3]

#     # Create a grid with pixel coordinates and depth values
#     y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
#     y *= -1.
#     x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
#     h = np.ones_like(z)

#     pixels = np.stack([x, y, z, h], axis=1)
#     # Filter out "infinite" depths
#     pixels = pixels[z < 0.99]
#     pixels[:, 2] = 2 * pixels[:, 2] - 1

#     # Reshape matrices for matrix multiplication
#     proj_matrix = np.asarray(proj_matrix).reshape((4, 4))
#     view_matrix = np.asarray(view_matrix).reshape((4, 4))

#     # Transform pixels to world coordinates
#     tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))
#     points = np.matmul(tran_pix_world, pixels.T).T
#     points /= points[:, 3: 4]
#     points = points[:, :3]

#     print(points) 

# def display_point_cloud(point_cloud):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     # Extract x, y, z coordinates from the point cloud
#     x = point_cloud[:, 0]
#     y = point_cloud[:, 1]
#     z = point_cloud[:, 2]

#     # Plot the point cloud
#     ax.scatter(x, y, z, c='b', marker='o')

#     ax.set_xlabel('X Label')
#     ax.set_ylabel('Y Label')
#     ax.set_zlabel('Z Label')

#     plt.show()

# def main():
#     p.connect(p.GUI)
#     p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
#     p.setTimeStep(1 / 240.)
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())
  
#     # Load table and plane
#     p.loadURDF("plane.urdf")
#     p.loadURDF("table/table.urdf")

#     # Simulation parameters
#     max_steps = 1000  # Number of simulation steps before displaying point cloud

#     # Get camera parameters
#     width, height, view_matrix, projection_matrix = get_camera_params()

#     # Main simulation loop
#     while True:
#         # Perform simulation step
#         p.stepSimulation()

#         # Generate point cloud after a certain number of steps
#         if p.getKeyboardEvents() or p.getConnectionInfo() or p.getConnectionInfo():
#             if p.getConnectionInfo()['connectionMethod'] == 3:
#                 print("We are connected to a remote GUI")
#             elif p.getConnectionInfo()['connectionMethod'] == 1:
#                 print("We are connected to a direct GUI")
#             else:
#                 print("This is an unknown connection method")

#         # Generate point cloud
#         point_cloud = get_point_cloud(width, height, view_matrix, projection_matrix)

#         # Display point cloud
#         display_point_cloud(point_cloud)
#         break

# if __name__ == "__main__":
#     main()
#--------------------------------------------------------------------------
#---------------------------------------------------------------------------

import pybullet as p
import pybullet_data as pd
import time
import open3d as o3d
import numpy as np

def get_camera_params():
    width = 640  # Example width, adjust as needed
    height = 480  # Example height, adjust as needed
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0],
                                                      distance=1,
                                                      yaw=0,
                                                      pitch=-90,
                                                      roll=0,
                                                      upAxisIndex=2)
    projection_matrix = p.computeProjectionMatrixFOV(fov=60,  # Example FOV, adjust as needed
                                                     aspect=width / height,
                                                     nearVal=0.01,
                                                     farVal=100)
    return width, height, view_matrix, projection_matrix


def get_point_cloud(width, height, view_matrix, proj_matrix):
    # Get a depth image
    image_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    depth = image_arr[3]
    print(depth)

    # Create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
    h = np.ones_like(z)
    pixels = np.stack([x, y, z, h], axis=1)
    
    # Filter out "infinite" depths
    pixels = pixels[z < 0.99]
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # Reshape matrices for matrix multiplication
    proj_matrix = np.asarray(proj_matrix).reshape((4, 4))
    view_matrix = np.asarray(view_matrix).reshape((4, 4))

    # Transform pixels to world coordinates
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    # Create an Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def main():
    p.connect(p.GUI)#, options="--background_color_red=1.0 --background_color_blue=1.0 --background_color_green=1.0")
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0,0,-9.8)

    useFixedBase = True
    flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

    #plane_pos = [0,0,0]
    #plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
    table_pos = [0,0,-0.625]
    plane_pos = [0,0,-0.625]
    plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
    table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
    xarm = p.loadURDF("xarm/xarm6_with_gripper.urdf", [.7,0,0], flags = flags, useFixedBase=useFixedBase)
    cube = p.loadURDF("cube.urdf", flags= flags, globalScaling=.1)

    jointIds = []
    paramIds = []

    for j in range(p.getNumJoints(xarm)):
        p.changeDynamics(xarm, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(xarm, j)
        #print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
            jointIds.append(j)
            paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
    
    skip_cam_frames = 10  

    max_steps = 1000  # Number of simulation steps before displaying point cloud
    current_step = 0

    while (1):
        width, height, view_matrix, projection_matrix = get_camera_params()
        p.stepSimulation()
        current_step += 1

        # # Generate point cloud after a certain number of steps
        if current_step >= max_steps:
            # Reset current step count
            current_step = 0

        # Generate point cloud
        point_cloud = get_point_cloud(width, height, view_matrix, projection_matrix)
        print(type(point_cloud))
        
        # Visualize point cloud using Open3D
        o3d.visualization.draw_geometries([point_cloud])

        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(xarm, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
        skip_cam_frames -= 1
        if (skip_cam_frames<0):
            p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL )
            skip_cam_frames = 10
        time.sleep(1./240.)
        
if __name__ == "__main__":
    main()