# Student: Clarrisa Mata, Janine Jimenez, Linh Tran
# Course: FRI 309
# 
# Date:05/06/2024
# Description of Program: Pybullet simulation for rearrangement learning

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


# Set camera parameters
camera_distance = 2 # Distance from the origin
camera_yaw = 0  # Yaw angle (rotation around the vertical axis)
camera_pitch = -80  # Pitch angle (rotation around the horizontal axis)
camera_target_position = [0.0, 0.0, 0.0]  # Camera target position (look at position)

# Reset the debug visualizer camera with the specified parameters
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
# p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
# load urdf data
models = models_data.model_lib()

def simulation():
    # load model list
    namelist = models.model_name_list
    print("Look at what we have {}".format(namelist))

    flags = p.URDF_USE_INERTIA_FROM_FILE
    # Load table and plane
    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf")
    existing_obj = [
    p.loadURDF(models[namelist[len(namelist)-2]], [0,.3,.65], flags=flags),
    p.loadURDF(models[namelist[len(namelist)-2]],[-.5, 0,0.65], flags=flags),
    p.loadURDF(models[namelist[len(namelist)-2]], [.5, 0, 0.65], flags=flags),
    p.loadURDF(models[namelist[len(namelist)-2]],[0,-.3,0.65], flags=flags)
    ]


    valid = [] #lis of valid poses
    col = [] #list of invalid poses
    for i in range(100):
    #contains coords [x,y,z] for each plate in the plane
        obj_coord = []
        plates = []
        for j in range(4):
            random_val = random.uniform(-4.9, 4.9)
            random_val2 = random.uniform(-4.9, 4.9)
            x, y, z = 0.1 * random_val, 0.1 * random_val2, 0.65
            coord = [x, y, z]
            if coord not in obj_coord:
                obj_coord.append(coord)
                plate = p.loadURDF(models[namelist[1]], coord, flags=flags)
                plates.append(plate)

        valid_flag = True
        for k in range(len(plates)-1):
            collisions = p.getClosestPoints(plates[k],(plates[k+1]), distance=.02)
            if collisions:
                valid_flag = False
                break
            for obj in existing_obj:
                collisions2 = p.getClosestPoints(plates[k], obj, distance=.02)
                if collisions2:
                    valid_flag = False
                    break
        print(valid_flag)
        if valid_flag:
            valid.append(obj_coord)  
            print("valid pos: ")
            print(valid)
        else: 
            col.append(obj_coord)
      
        # remove bodies
        for plate in plates:
            p.removeBody(plate)

       
    p.setGravity(0, 0, -9.8)

    while 1:
        p.stepSimulation()
        time.sleep(1./220)
    

if __name__ == "__main__":
    simulation()
        