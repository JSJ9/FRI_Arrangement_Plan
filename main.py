#https://github.com/ChenEating716/pybullet-URDF-models --for table and random objects
#https://github.com/robot-descriptions/awesome-robot-descriptions?tab=readme-ov-file -- sawyer, we might not
#need it tho because we are only using pybullet to get valid pointclouds
#https://github.com/RethinkRobotics/sawyer_robot.git
#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pointCloudFromCameraImage.py

import time
import pybullet as p
import pybullet_data
from urdf_models import models_data
import random
import math 
import numpy as np

# initialize the GUI and others
p.connect(p.GUI)
p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
p.setTimeStep(1 / 240.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.getCameraImage(640,480)
# Set camera parameters
width = 640
height = 480
fov = 60
aspect = width / height
nearPlane = 0.1
farPlane = 100

# Define camera position and orientation for bird's eye view
camera_target_position = [0, 0, 1]  # Look at the origin (center of the scene)
camera_distance = 5  # Distance from the origin
camera_yaw = 0  # Look straight down
camera_pitch = -90  # Look straight down

# Set the camera parameters
p.computeViewMatrixFromYawPitchRoll(camera_target_position, camera_distance, camera_yaw, camera_pitch, 0, 2)
p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)
# load urdf data
models = models_data.model_lib()

# load model list
namelist = models.model_name_list
print("Look at what we have {}".format(namelist))

flags = p.URDF_USE_INERTIA_FROM_FILE
# Load table and plane
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf")
p.loadURDF(models[namelist[len(namelist)-2]], [0,.3,.65], flags=flags)
p.loadURDF(models[namelist[len(namelist)-2]],[-.5, 0,0.65], flags=flags)
p.loadURDF(models[namelist[len(namelist)-2]], [.5, 0, 0.65], flags=flags)
p.loadURDF(models[namelist[len(namelist)-2]],[0,-.3,0.65], flags=flags)

# load the randomly picked model

# randomly get a model
#for i in range(8):
#    random_model = namelist[random.randint(0, len(namelist))] 
#    p.loadURDF(models[random_model], [0., 0., 0.8 + 0.15*i], flags=flags)

#stacking plates!!! :D
#for i in range(4):
#    p.loadURDF(models[namelist[1]], [0., 0., 0.8 + 0.15*i], flags=flags)


#more random location
unique_coord = []
valid = []  # Initialize valid as an empty list
for i in range(100):
  #contains coords [x,y,z] for each plate in the plane
    obj_coord = []
    plates = []
    
    for j in range(4):
        random_val = random.randint(-5, 5)
        random_val2 = random.randint(-5, 5)
        x, y, z = 0.15 * random_val, 0.1 * random_val2, 0.65
        coord = [x, y, z]
        if coord not in obj_coord:
            obj_coord.append(coord)
            plate = p.loadURDF(models[namelist[1]], coord, flags=flags)
            plates.append(plate)
    for k in range(len(plates)-1):
        collisions = p.getOverlappingObjects(p.getAABB(plates[k]), p.getAABB(plates[k+1]))
        print(collisions)
        if not collisions:
            valid.append(obj_coord)

    # Remove bodies
    for plate in plates:
        p.removeBody(plate)
    print("valid pos: ")
    print(valid)


    
    


    

    # valid[True] += obj_coord
  
print(unique_coord[1])  # valid[False] += col_coord



    
p.setGravity(0, 0, -9.8)

# print(obj_coord)


while 1:
    p.stepSimulation()
    time.sleep(1./240)