#https://github.com/ChenEating716/pybullet-URDF-models --for table and random objects
#https://github.com/robot-descriptions/awesome-robot-descriptions?tab=readme-ov-file -- sawyer, we might not
#need it tho because we are only using pybullet to get valid pointclouds
import pybullet as p
import time
import pybullet_data
# import module
import urdf_models.models_data as md

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
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

# create model library
models_lib = md.model_lib()

# get the name list of all models
print(models_lib.model_name_list)

# get the absolute path of all models in your computer
print(models_lib.model_path_list)

# find the corresponding abs path for desired model
print(models_lib['knife'])

# get random model file path
print(models_lib.random)
# print(cubePos,cubeOrn)
p.disconnect()
