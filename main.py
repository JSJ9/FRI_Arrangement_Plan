# # #https://github.com/ChenEating716/pybullet-URDF-models --for table and random objects
# # #https://github.com/robot-descriptions/awesome-robot-descriptions?tab=readme-ov-file -- sawyer, we might not
# # #need it tho because we are only using pybullet to get valid pointclouds
# # import pybullet as p
# # import time
# # import pybullet_data

# # physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# # p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# # p.setGravity(0,0,-10)
# # planeId = p.loadURDF("plane.urdf")
# # startPos = [0,0,1]
# # startOrientation = p.getQuaternionFromEuler([0,0,0])
# # boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# # sawyer = p.loadMJCF("")
# # #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
# # for i in range (10000):
# #     p.stepSimulation()
# #     time.sleep(1./240.)
# # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# # print(cubePos,cubeOrn)
# # p.disconnect()
# """..................................................................
# ...................................................................
# ....................................................................."""
import os
import time
import pybullet as p
import pybullet_data
from urdf_models import models_data
import random
import numpy as np
import open3d as o3d

def get_camera_params():
    width = 640  # Example width, adjust as needed
    height = 480  # Example height, adjust as needed
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0],
                                                      distance=5,
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
    print(pcd)

    return pcd

# initialize the GUI and others
def main():
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setTimeStep(1 / 240.)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
  
    # load urdf data
    models = models_data.model_lib()

    # load model list
    namelist = models.model_name_list
    print("Look at what we have {}".format(namelist))

    # Load table and plane
    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf")

    # load the randomly picked model
    flags = p.URDF_USE_INERTIA_FROM_FILE
    # randomly get a model
    for i in range(2):
        random_model = namelist[random.randint(0, len(namelist)-1)] 
        p.loadURDF(models[random_model], [0., 0., 0.8 + 0.15*i], flags=flags)
        
    p.setGravity(0,0,-9.8)
    
    # Simulation parameters
    max_steps = 1000  # Number of simulation steps before displaying point cloud
    current_step = 0

    # Main simulation loop
    while True:
    # Get camera parameters
        width, height, view_matrix, projection_matrix = get_camera_params()

        # Perform simulation step
        p.stepSimulation()

        current_step += 1

        # Generate point cloud after a certain number of steps
        if current_step >= max_steps:
            # Reset current step count
            current_step = 0

            # Generate point cloud
            point_cloud = get_point_cloud(width, height, view_matrix, projection_matrix)
            print(type(point_cloud))
            
            # Visualize point cloud using Open3D
            o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    main()

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
