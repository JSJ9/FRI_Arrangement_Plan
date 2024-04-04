# #https://github.com/ChenEating716/pybullet-URDF-models --for table and random objects
# #https://github.com/robot-descriptions/awesome-robot-descriptions?tab=readme-ov-file -- sawyer, we might not
# #need it tho because we are only using pybullet to get valid pointclouds
# import pybullet as p
# import time
# import pybullet_data

# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
# startPos = [0,0,1]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# sawyer = p.loadMJCF("")
# #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
# p.disconnect()

import os
import time
import pybullet as p
import pybullet_data
from urdf_models import models_data
import random

# initialize the GUI and others
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
for i in range(8):
    random_model = namelist[random.randint(0, len(namelist))] 
    p.loadURDF(models[random_model], [0., 0., 0.8 + 0.15*i], flags=flags)

p.setGravity(0, 0, -9.8)

while 1:
    p.stepSimulation()
    time.sleep(1./240)

