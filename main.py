#https://github.com/ChenEating716/pybullet-URDF-models --for table and random objects
#https://github.com/robot-descriptions/awesome-robot-descriptions?tab=readme-ov-file -- sawyer, we might not
#need it tho because we are only using pybullet to get valid pointclouds
#https://github.com/RethinkRobotics/sawyer_robot.git
#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pointCloudFromCameraImage.py

# import pybullet as p
# import pybullet_data as pd
# import time
# import numpy as np
# import math

# def getRayFromTo(mouseX, mouseY):
#     width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
#     )
#     camPos = [
#         camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
#         camTarget[2] - dist * camForward[2]
#     ]
#     farPlane = 10000
#     rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
#     lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
#                         rayForward[2] * rayForward[2])
#     invLen = farPlane * 1. / lenFwd
#     rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
#     rayFrom = camPos
#     oneOverWidth = float(1) / float(width)
#     oneOverHeight = float(1) / float(height)

#     dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
#     dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
#     rayToCenter = [
#         rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
#     ]
#     ortho = [
#         -0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
#         -0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
#         -0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
#     ]

#     rayTo = [
#         rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
#         rayFrom[2] + rayForward[2] + ortho[2]
#     ]
#     lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
#     alpha = math.atan(lenOrtho / farPlane)
#     return rayFrom, rayTo, alpha


#     width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
#     )
#     camPos = [
#         camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
#         camTarget[2] - dist * camForward[2]
#     ]
#     farPlane = 10000
#     rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
#     lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
#                     rayForward[2] * rayForward[2])
#     oneOverWidth = float(1) / float(width)
#     oneOverHeight = float(1) / float(height)
#     dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
#     dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]

#     lendHor = math.sqrt(dHor[0] * dHor[0] + dHor[1] * dHor[1] + dHor[2] * dHor[2])
#     lendVer = math.sqrt(dVer[0] * dVer[0] + dVer[1] * dVer[1] + dVer[2] * dVer[2])

#     cornersX = [0, width, width, 0]
#     cornersY = [0, 0, height, height]
#     corners3D = []

#     imgW = int(width / 10)
#     imgH = int(height / 10)

#     img = p.getCameraImage(imgW, imgH, renderer=p.ER_BULLET_HARDWARE_OPENGL)
#     rgbBuffer = np.reshape(img[2], (imgH, imgW, 4))
#     # NOTE: this depth buffer's reshaping does not match the [w, h] convention for
#     # OpenGL depth buffers.  See getCameraImageTest.py for an OpenGL depth buffer
#     depthBuffer = np.reshape(img[3], [imgH, imgW])
#     print("rgbBuffer.shape=", rgbBuffer.shape)
#     print("depthBuffer.shape=", depthBuffer.shape)


# def main():
#     p.connect(p.GUI)#, options="--background_color_red=1.0 --background_color_blue=1.0 --background_color_green=1.0")
#     p.setAdditionalSearchPath(pd.getDataPath())
#     p.setGravity(0,0,-9.8)

#     useFixedBase = True
#     flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

#     #plane_pos = [0,0,0]
#     #plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
#     table_pos = [0,0,-0.625]
#     plane_pos = [0,0,-0.625]
#     plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
#     table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
#     xarm = p.loadURDF("xarm/xarm6_with_gripper.urdf", [.65,0,0], flags = flags, useFixedBase=useFixedBase)
#     cube = p.loadURDF("cube.urdf", flags= flags, globalScaling=.1)

#     jointIds = []
#     paramIds = []

#     for j in range(p.getNumJoints(xarm)):
#         p.changeDynamics(xarm, j, linearDamping=0, angularDamping=0)
#         info = p.getJointInfo(xarm, j)
#         #print(info)
#         jointName = info[1]
#         jointType = info[2]
#         if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
#             jointIds.append(j)
#             paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
    
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
#         time.sleep(1./240.)
        
# if __name__ == "__main__":
#     main()

#---------------------------------------------------------------------
#---------------------------------------------------------------------
import pybullet as p
import math
import numpy as np
import pybullet_data
import open3d as o3d

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# plane = p.loadURDF("plane100.urdf")
# cube = p.loadURDF("cube.urdf", [0, 0, 1])

useFixedBase = True
flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

table_pos = [0,0,-0.625]
plane_pos = [0,0,-0.625]
plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
xarm = p.loadURDF("xarm/xarm6_with_gripper.urdf", [.65,0,0], flags = flags, useFixedBase=useFixedBase)
cube = p.loadURDF("cube.urdf", [0,0, 1], flags= flags, globalScaling=.1)


def getRayFromTo(mouseX, mouseY):
  width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
  )
  camPos = [
      camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
      camTarget[2] - dist * camForward[2]
  ]
  farPlane = 10000
  rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
  lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
                     rayForward[2] * rayForward[2])
  invLen = farPlane * 1. / lenFwd
  rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
  rayFrom = camPos
  oneOverWidth = float(1) / float(width)
  oneOverHeight = float(1) / float(height)

  dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
  dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
  rayToCenter = [
      rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
  ]
  ortho = [
      -0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
      -0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
      -0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
  ]

  rayTo = [
      rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
      rayFrom[2] + rayForward[2] + ortho[2]
  ]
  lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
  alpha = math.atan(lenOrtho / farPlane)


  return rayFrom, rayTo, alpha


width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
)
camPos = [
    camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
    camTarget[2] - dist * camForward[2]
]
farPlane = 10000
rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
                   rayForward[2] * rayForward[2])
oneOverWidth = float(1) / float(width)
oneOverHeight = float(1) / float(height)
dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]

lendHor = math.sqrt(dHor[0] * dHor[0] + dHor[1] * dHor[1] + dHor[2] * dHor[2])
lendVer = math.sqrt(dVer[0] * dVer[0] + dVer[1] * dVer[1] + dVer[2] * dVer[2])

cornersX = [0, width, width, 0]
cornersY = [0, 0, height, height]
corners3D = []

imgW = int(width / 10)
imgH = int(height / 10)

img = p.getCameraImage(imgW, imgH, renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgbBuffer = np.reshape(img[2], (imgH, imgW, 4))
# NOTE: this depth buffer's reshaping does not match the [w, h] convention for
# OpenGL depth buffers.  See getCameraImageTest.py for an OpenGL depth buffer
depthBuffer = np.reshape(img[3], [imgH, imgW])
print("rgbBuffer.shape=", rgbBuffer.shape)
print("depthBuffer.shape=", depthBuffer.shape)

#disable rendering temporary makes adding objects faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
collisionShapeId = -1  #p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="duck_vhacd.obj", collisionFramePosition=shift,meshScale=meshScale)

for i in range(4):
  w = cornersX[i]
  h = cornersY[i]
  rayFrom, rayTo, _ = getRayFromTo(w, h)
  rf = np.array(rayFrom)
  rt = np.array(rayTo)
  vec = rt - rf
  l = np.sqrt(np.dot(vec, vec))
  newTo = (0.01 / l) * vec + rf
  #print("len vec=",np.sqrt(np.dot(vec,vec)))

  p.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
  corners3D.append(newTo)
count = 0

stepX = 5
stepY = 5
pcd = []
for w in range(0, imgW, stepX):
  for h in range(0, imgH, stepY):
    count += 1
    if ((count % 100) == 0):
      print(count, "out of ", imgW * imgH / (stepX * stepY))
    rayFrom, rayTo, alpha = getRayFromTo(w * (width / imgW), h * (height / imgH))
    rf = np.array(rayFrom)
    rt = np.array(rayTo)
    vec = rt - rf
    l = np.sqrt(np.dot(vec, vec))
    depthImg = float(depthBuffer[h, w])
    far = 1000.
    near = 0.01
    depth = far * near / (far - (far - near) * depthImg)
    depth /= math.cos(alpha)
    newTo = (depth / l) * vec + rf
    p.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
    mb = p.createMultiBody(baseMass=0,
                           baseCollisionShapeIndex=collisionShapeId,
                           baseVisualShapeIndex=visualShapeId,
                           basePosition=newTo,
                           useMaximalCoordinates=True)
    color = rgbBuffer[h, w]
    color = [color[0] / 255., color[1] / 255., color[2] / 255., 1]
    p.changeVisualShape(mb, -1, rgbaColor=color)
    pcd.append(newTo)


  
p.addUserDebugLine(corners3D[0], corners3D[1], [1, 0, 0])
p.addUserDebugLine(corners3D[1], corners3D[2], [1, 0, 0])
p.addUserDebugLine(corners3D[2], corners3D[3], [1, 0, 0])
p.addUserDebugLine(corners3D[3], corners3D[0], [1, 0, 0])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
print("ready\n")


#p.removeBody(plane)
#p.removeBody(cube)
while (1):
  pcd_np = np.asarray(pcd)  # Convert pcd to numpy array
  # Create an Open3D point cloud object
  point_cloud_o3d = o3d.geometry.PointCloud()
  point_cloud_o3d.points = o3d.utility.Vector3dVector(pcd_np)
  # Visualize the point cloud
  o3d.visualization.draw_geometries([point_cloud_o3d])
  p.setGravity(0, 0, -10)