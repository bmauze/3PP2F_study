import numpy as np
import matplotlib.pyplot as plt
#from math import np.sin, np.cos, sqrt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation

def HomogeneousTransform(transVec,rotVec):
    """3D Homogeneous transformation
        Return the 3D homogeneous transformation from a translation and rotation vector (rotation order is ZYX)
        The order for the vector is transX,transY,transZ ; angleRotX, angleRotY, angleRotZ
    """
    
    return np.array([  [np.cos(rotVec[0,2])*np.cos(rotVec[0,1]), np.cos(rotVec[0,2])*np.sin(rotVec[0,1])*np.sin(rotVec[0,0])-np.sin(rotVec[0,2])*np.cos(rotVec[0,0]),   np.cos(rotVec[0,2])*np.sin(rotVec[0,1])*np.cos(rotVec[0,0]) + np.sin(rotVec[0,2])*np.sin(rotVec[0,0]), transVec[0,0] ],
                       [np.sin(rotVec[0,2])*np.cos(rotVec[0,1]), np.sin(rotVec[0,2])*np.sin(rotVec[0,1])*np.sin(rotVec[0,0])+np.cos(rotVec[0,2])*np.cos(rotVec[0,0]),   np.sin(rotVec[0,2])*np.sin(rotVec[0,1])*np.cos(rotVec[0,0]) - np.cos(rotVec[0,2])*np.sin(rotVec[0,0]), transVec[0,1] ],
                       [                   -np.sin(rotVec[0,1]),                                                             np.cos(rotVec[0,1])*np.sin(rotVec[0,0]),                                                                 np.cos(rotVec[0,1])*np.cos(rotVec[0,0]), transVec[0,2] ],
                       [                                     0.,                                                                                                  0.,                                                                                                      0.,            1. ] ])

lengthFiber=80.0
numberElement=7
ds = lengthFiber/(numberElement)
platformRadius=20
actuatorImplementationRadius=55.0

anglePlat1z = np.pi/2.0
anglePlat2z = 5*np.pi/6.0
anglePlat3z = np.pi/6.0

angleAct1zP = -np.pi/6.0 #0.0
angleAct2zP =  -5*np.pi/6.0# - np.pi/3.0
angleAct3zP =  np.pi/2.0# - 2.0*np.pi/3.0

angleAct1z =  -5*np.pi/6.0
angleAct2z = -np.pi/6.0
angleAct3z = np.pi/2.0

demiCorde = np.cos(np.pi/6.0)*platformRadius
hP        = np.sin(np.pi/6.0)*platformRadius
lP        = np.sqrt(lengthFiber*lengthFiber-demiCorde*demiCorde)

# height = -lengthFiber*np.sin(np.pi/3.0)
positionPlatform=[0.003423230215944049, 0.006022613451855615, 0, -3.265265483357694e-05, 3.183590147299464e-05, -7.225301700406276e-06, 0.9999999989340372]

PosClampPlat1 = np.array([ [platformRadius*np.cos(anglePlat1z), platformRadius*np.sin(anglePlat1z), 0.] ])
PosClampPlat2 = np.array([ [platformRadius*np.cos(anglePlat2z), platformRadius*np.sin(anglePlat2z), 0.] ])
PosClampPlat3 = np.array([ [platformRadius*np.cos(anglePlat3z), platformRadius*np.sin(anglePlat3z), 0.] ])

PosClampAct1 = np.array([ [ 64.89,  37.46, -50.0, 0.0, 0.0, 0.0, 1.0] ])
PosClampAct2 = np.array([ [-64.89,  37.46, -50.0, 0.0, 0.0, 0.0, 1.0] ])
PosClampAct3 = np.array([ [  0.0 , -74.93, -50.0, 0.0, 0.0, 0.0, 1.0] ])


angleZ21=-(np.arccos( (actuatorImplementationRadius-hP)/lP ))
angleZ22= -(np.arccos( (actuatorImplementationRadius-hP)/lP ))
angleZ23=-(np.arccos( (actuatorImplementationRadius-hP)/lP ))

angleX=0.0

angleFibre1 = np.arcsin(demiCorde / lengthFiber)
angleFibre2 = - np.arcsin(demiCorde / lengthFiber)

rotVec_Act1_F1 = np.array([ [angleX, angleZ21, angleAct1z] ])
rot1_F1 = Rotation.from_euler('zyx', [angleAct1z, angleZ21, angleX], degrees=False)

rotVec_Act1_F2 = np.array([ [angleX, angleZ21, angleAct1z] ])
rot1_F2 = Rotation.from_euler('zyx', [angleAct1z, angleZ21, angleX], degrees=False)

rotVec_Act2_F1 = np.array([ [angleX, angleZ22, angleAct2z] ])
rot2_F1 = Rotation.from_euler('zyx', [angleAct2z, angleZ22, angleX], degrees=False)

rotVec_Act2_F2 = np.array([ [angleX, angleZ22, angleAct2z] ])
rot2_F2 = Rotation.from_euler('zyx', [angleAct2z, angleZ22, angleX], degrees=False)

rotVec_Act3_F1 = np.array([ [angleX, angleZ23, angleAct3z] ])
rot3_F1 = Rotation.from_euler('zyx', [angleAct3z, angleZ23, angleX], degrees=False)

rotVec_Act3_F2 = np.array([ [angleX, angleZ23, angleAct3z] ])
rot3_F2 = Rotation.from_euler('zyx', [angleAct3z, angleZ23, angleX], degrees=False)

HT_Act1_F1 = HomogeneousTransform(PosClampAct1,rotVec_Act1_F1)
HT_Act1_F2 = HomogeneousTransform(PosClampAct1,rotVec_Act1_F2)
HT_Act2_F1 = HomogeneousTransform(PosClampAct2,rotVec_Act2_F1)
HT_Act2_F2 = HomogeneousTransform(PosClampAct2,rotVec_Act2_F2)
HT_Act3_F1 = HomogeneousTransform(PosClampAct3,rotVec_Act3_F1) 
HT_Act3_F2 = HomogeneousTransform(PosClampAct3,rotVec_Act3_F2)
# print(HT_Act1_F1)

matrixRotz1= np.array([ [np.cos(angleFibre1),-np.sin(angleFibre1),0,0], [np.sin(angleFibre1),np.cos(angleFibre1),0,0], [0,0,1,0], [0,0,0,1] ])
matrixRotz2= np.array([ [np.cos(angleFibre2),-np.sin(angleFibre2),0,0], [np.sin(angleFibre2),np.cos(angleFibre2),0,0], [0,0,1,0], [0,0,0,1] ])

matrixRotz= np.array([ [np.cos(angleZ21),-np.sin(angleZ21),0,0], [np.sin(angleZ21),np.cos(angleZ21),0,0], [0,0,1,0], [0,0,0,1] ])
HT_Act1_F1 = HT_Act1_F1.dot(matrixRotz1)
HT_Act1_F2 = HT_Act1_F2.dot(matrixRotz2)
matrixRotz= np.array([ [np.cos(angleZ22),-np.sin(angleZ22),0,0], [np.sin(angleZ22),np.cos(angleZ22),0,0], [0,0,1,0], [0,0,0,1] ])
HT_Act2_F1 = HT_Act2_F1.dot(matrixRotz1)
HT_Act2_F2 = HT_Act2_F2.dot(matrixRotz2)
matrixRotz= np.array([ [np.cos(angleZ23),-np.sin(angleZ23),0,0], [np.sin(angleZ23),np.cos(angleZ23),0,0], [0,0,1,0], [0,0,0,1] ])
HT_Act3_F1 = HT_Act3_F1.dot(matrixRotz1)
HT_Act3_F2 = HT_Act3_F2.dot(matrixRotz2)
# print(HT_Act1_F1)

# cal= Rotation.from_dcm([[0.79,-0.19,-0.60],[-0.28,0.79,-0.54],[0.54,0.60,0.60]])
# elcalrad=cal.as_euler('zyx',degrees=False)
# elcaldeg=cal.as_euler('zyx',degrees=True)
# quatcal=cal.as_quat()
# print(elcaldeg,elcalrad)
# HT_Act1_F1 =np.array([[0.79,-0.13,-0.60,PosClampAct1[0,0]],[-0.28,0.79,-0.54,PosClampAct1[0,1]],[0.54,0.60,0.60,PosClampAct1[0,2]],[0,0,0,1] ])
# print(HT_Act1_F1)

rot1_F1= Rotation.from_dcm(HT_Act1_F1[:-1,:-1])
rot_quat1_F1 = rot1_F1.as_quat()
# print(rot_quat1_F1)
rot1_F2=Rotation.from_dcm(HT_Act1_F2[:-1,:-1])
rot_quat1_F2 = rot1_F2.as_quat()
rot2_F1=Rotation.from_dcm(HT_Act2_F1[:-1,:-1])
rot_quat2_F1 = rot2_F1.as_quat()
rot2_F2=Rotation.from_dcm(HT_Act2_F2[:-1,:-1])
rot_quat2_F2 = rot2_F2.as_quat()
rot3_F1=Rotation.from_dcm(HT_Act3_F1[:-1,:-1])
rot_quat3_F1 = rot3_F1.as_quat()
rot3_F2=Rotation.from_dcm(HT_Act3_F2[:-1,:-1])
rot_quat3_F2 = rot3_F2.as_quat()

# rot_quat2_F2[0]=-rot_quat2_F2[0]
# rot_quat3_F1[0]=-rot_quat3_F1[0]
# rot_quat2_F1[0]=-rot_quat2_F1[0]
# rot_quat3_F2[0]=-rot_quat3_F2[0]
# print(rot_quat1_F1)
# print(rot_quat1_F2)
# print(rot_quat2_F1)
# print(rot_quat2_F2)
# print(rot_quat3_F1)
# print(rot_quat3_F2)


T_Act1_F1 = np.zeros((3, numberElement+1))
T_Act1_F2 = np.zeros((3, numberElement+1))
T_Act2_F1 = np.zeros((3, numberElement+1))
T_Act2_F2 = np.zeros((3, numberElement+1))
T_Act3_F1 = np.zeros((3, numberElement+1))
T_Act3_F2 = np.zeros((3, numberElement+1))
for i in range(0, numberElement+1):
    matrixDisplacement= np.array([ [1,0,0,i*ds], [0,1,0,0], [0,0,1,0], [0,0,0,1] ])
    interCalMatrix = HT_Act1_F1.dot(matrixDisplacement)
    T_Act1_F1[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act1_F2.dot(matrixDisplacement)
    T_Act1_F2[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act2_F1.dot(matrixDisplacement)
    T_Act2_F1[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act2_F2.dot(matrixDisplacement)
    T_Act2_F2[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act3_F1.dot(matrixDisplacement)
    T_Act3_F1[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act3_F2.dot(matrixDisplacement)
    T_Act3_F2[0:3,i]=interCalMatrix[0:3,3] 

varr=numberElement+3
T_Act1_F1b = np.zeros((3, varr))
T_Act1_F2b = np.zeros((3, varr))
T_Act2_F1b = np.zeros((3, varr))
T_Act2_F2b = np.zeros((3, varr))
T_Act3_F1b = np.zeros((3, varr))
T_Act3_F2b = np.zeros((3, varr))
for i in range(0, varr):
    matrixDisplacement= np.array([ [1,0,0,i*ds], [0,1,0,0], [0,0,1,0], [0,0,0,1] ])
    interCalMatrix = HT_Act1_F1.dot(matrixDisplacement)
    T_Act1_F1b[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act1_F2.dot(matrixDisplacement)
    T_Act1_F2b[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act2_F1.dot(matrixDisplacement)
    T_Act2_F1b[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act2_F2.dot(matrixDisplacement)
    T_Act2_F2b[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act3_F1.dot(matrixDisplacement)
    T_Act3_F1b[0:3,i]=interCalMatrix[0:3,3] 
    interCalMatrix = HT_Act3_F2.dot(matrixDisplacement)
    T_Act3_F2b[0:3,i]=interCalMatrix[0:3,3] 

# print(ds)
# HTverif = HomogeneousTransform(np.array([ [0.,0., 0.] ]),np.array([ [0.,0., 0.] ]))
# print(HTverif.dot(matrixDisplacement))
# print(matrixDisplacement)
# print(HTverif)

height=0
T_Act1_F1[2,:]=height-T_Act1_F1[2,:]
T_Act1_F2[2,:]=height-T_Act1_F2[2,:]
T_Act2_F1[2,:]=height-T_Act2_F1[2,:]
T_Act2_F2[2,:]=height-T_Act2_F2[2,:]
T_Act3_F1[2,:]=height-T_Act3_F1[2,:]
T_Act3_F2[2,:]=height-T_Act3_F2[2,:]

height=T_Act1_F1[2,-1]
T_Act1_F1[2,:]=height-T_Act1_F1[2,:]
T_Act1_F2[2,:]=height-T_Act1_F2[2,:]
T_Act2_F1[2,:]=height-T_Act2_F1[2,:]
T_Act2_F2[2,:]=height-T_Act2_F2[2,:]
T_Act3_F1[2,:]=height-T_Act3_F1[2,:]
T_Act3_F2[2,:]=height-T_Act3_F2[2,:]

# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# ax.scatter(T_Act1_F1[0], T_Act1_F1[1], zs=T_Act1_F1[2], zdir='z', c='b', marker='1', label='FLA1')
# ax.scatter(T_Act1_F2[0], T_Act1_F2[1], zs=T_Act1_F2[2], zdir='z', c='b', marker='1', label='SLA1')
# ax.scatter(T_Act2_F1[0], T_Act2_F1[1], zs=T_Act2_F1[2], zdir='z', c='r', marker='1', label='FLA2')
# ax.scatter(T_Act2_F2[0], T_Act2_F2[1], zs=T_Act2_F2[2], zdir='z', c='r', marker='1', label='SLA2')
# ax.scatter(T_Act3_F1[0], T_Act3_F1[1], zs=T_Act3_F1[2], zdir='z', c='k', marker='1', label='FLA3')
# ax.scatter(T_Act3_F2[0], T_Act3_F2[1], zs=T_Act3_F2[2], zdir='z', c='k', marker='1', label='SLA3')
# # Make legend, set axes limits and labels
# ax.legend()
# ax.set_xlim(-70, 70)
# ax.set_ylim(-70, 70)
# ax.set_zlim(-70,0)
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show()


edgesList=[]
for i in range(0, numberElement-1):
    edgesList.append([1 + i , i + 2])
edgesList.append([numberElement, 0])
edgesList.append([1, numberElement + 1])

for  i in range(numberElement + 1, 2*(numberElement)-1):
    edgesList.append([i , i +1])
edgesList.append([i+1, 0])

for  i in range(2*(numberElement), 3*(numberElement)-1):
    edgesList.append([i , i + 1])
edgesList.append([i + 1, 0])
edgesList.append([2*(numberElement),i+2])

for  i in range(3*(numberElement), 4*numberElement-2):
    edgesList.append([i , i +1])
edgesList.append([i + 1, 0])
for  i in range(4*(numberElement)-1, 5*numberElement -2):
    edgesList.append([i , i + 1])
edgesList.append([ i+1,  0])

edgesList.append([4*numberElement-1 , 5*numberElement -1])

for  i in range(5*numberElement -1, 6*numberElement -3):
    edgesList.append([i , i + 1])
edgesList.append([i + 1, 0])

# Adaptation of the position of the rod into the position list
OrA1F1_i=np.array(rot_quat1_F1)
OrA1F1=OrA1F1_i
for  i in range(0, numberElement):
    OrA1F1=np.vstack((OrA1F1,OrA1F1_i))
interCalMatrix=np.transpose(T_Act1_F1)
InterMatrix=np.vstack((T_Act1_F1,np.transpose(OrA1F1)))
InterMatrixTA1F1=np.transpose(InterMatrix)
OrA1F2_i=np.array(rot_quat1_F2)
OrA1F2=OrA1F2_i
for  i in range(0, numberElement):
    OrA1F2=np.vstack((OrA1F2,OrA1F2_i))
interCalMatrix=np.transpose(T_Act1_F2)
InterMatrix=np.vstack((T_Act1_F2,np.transpose(OrA1F2)))
InterMatrixTA1F2=np.transpose(InterMatrix)

OrA2F1_i=np.array(rot_quat2_F1)
OrA2F1=OrA2F1_i
for  i in range(0, numberElement):
    OrA2F1=np.vstack((OrA2F1,OrA2F1_i))
interCalMatrix=np.transpose(T_Act2_F1)
InterMatrix=np.vstack((T_Act2_F1,np.transpose(OrA2F1)))
InterMatrixTA2F1=np.transpose(InterMatrix)

OrA2F2_i=np.array(rot_quat2_F2)
OrA2F2=OrA2F2_i
for  i in range(0, numberElement):
    OrA2F2=np.vstack((OrA2F2,OrA2F2_i))
interCalMatrix=np.transpose(T_Act2_F2)
InterMatrix=np.vstack((T_Act2_F2,np.transpose(OrA2F2)))
InterMatrixTA2F2=np.transpose(InterMatrix)

OrA3F1_i=np.array(rot_quat3_F1)
OrA3F1=OrA3F1_i
for  i in range(0, numberElement):
    OrA3F1=np.vstack((OrA3F1,OrA3F1_i))
interCalMatrix=np.transpose(T_Act3_F1)
InterMatrix=np.vstack((T_Act3_F1,np.transpose(OrA3F1)))
InterMatrixTA3F1=np.transpose(InterMatrix)

OrA3F2_i=np.array(rot_quat3_F2)
OrA3F2=OrA3F2_i
for  i in range(0, numberElement):
    OrA3F2=np.vstack((OrA3F2,OrA3F2_i))
interCalMatrix=np.transpose(T_Act3_F2)
InterMatrix=np.vstack((T_Act3_F2,np.transpose(OrA3F2)))
InterMatrixTA3F2=np.transpose(InterMatrix)

# positionLista=np.array([[0., 0., T_Act3_F2[2,0], 0., 0., 0., 1.]])
positionLista=np.array([[0., 0., 0, 0., 0., 0., 1.]])

# positionLista=np.vstack((positionList,InterMatrixTA1F1[0,:]))
# positionLista=np.vstack((positionLista,InterMatrixTA2F1[0,:]))
# positionLista=np.vstack((positionLista,InterMatrixTA3F1[0,:]))
positionLista=np.vstack((positionLista,InterMatrixTA1F1[:-1,:]))
positionLista=np.vstack((positionLista,InterMatrixTA1F2[1:-1,:]))
positionLista=np.vstack((positionLista,InterMatrixTA2F1[:-1,:]))
positionLista=np.vstack((positionLista,InterMatrixTA2F2[1:-1,:]))
positionLista=np.vstack((positionLista,InterMatrixTA3F1[:-1,:]))
positionLista=np.vstack((positionLista,InterMatrixTA3F2[1:-1,:]))
positionList=positionLista

positionList[1,3:]=[0.,0.,0.,1.]
positionList[2*numberElement,3:]=[0.,0.,0.,1.]
positionList[4*numberElement-1,3:]=[0.,0.,0.,1.]

DOF0TransformNode0_i=np.transpose([0.,0.,0., 0.,0.,0.,1.])
DOF0TransformNode0=np.transpose([0.,0.,0., 0.,0.,0.,1.])
for  i in range(0, (numberElement)*6-1):
    DOF0TransformNode0=np.vstack((DOF0TransformNode0, DOF0TransformNode0_i))


DOF0TransformNode0[0,3:]=rot_quat1_F1
DOF0TransformNode0[numberElement,3:]=rot_quat1_F2
DOF0TransformNode0[2*numberElement,3:]=rot_quat2_F1
DOF0TransformNode0[3*numberElement,3:]=rot_quat2_F2
DOF0TransformNode0[4*numberElement,3:]=rot_quat3_F1
DOF0TransformNode0[5*numberElement,3:]=rot_quat3_F2


matrixZ=np.zeros((6,numberElement-1))
matrixO=np.ones((1,numberElement-1))
NullDof=np.transpose(np.vstack((matrixZ,matrixO)))
DOF1TransformNode1=NullDof

DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act1_F1[0:2,-1]), np.hstack((np.array([0.]), rot_quat1_F1)) )) ))
# DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(InterMatrixTA1F1[-1,0:3]),np.array([0.,0.,-0.603,0.,0.798]))) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,NullDof))
# DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act1_F2[0:2,-1]),np.array([0.,0.,-0.798,0.,0.603]))) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act1_F2[0:2,-1]), np.hstack((np.array([0.]), rot_quat1_F2)) )) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,NullDof))
# DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act2_F1[0:2,-1]),np.array([0.,-0.302,-0.522,-0.399,0.691]))) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act2_F1[0:2,-1]), np.hstack((np.array([0.]), rot_quat2_F1)) )) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,NullDof))
# DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act2_F2[0:2,-1]),np.array([0.,-0.399,-0.691,-0.302,0.522]))) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act2_F2[0:2,-1]), np.hstack((np.array([0.]), rot_quat2_F2)) )) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,NullDof))
# DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act3_F1[0:2,-1]),np.array([0.,-0.522,-0.302,-0.691,0.399]))) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act3_F1[0:2,-1]), np.hstack((np.array([0.]), rot_quat3_F1)) )) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,NullDof))
# DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act3_F2[0:2,-1]),np.array([0.,-0.691,-0.399,-0.522,0.302]))) ))
DOF1TransformNode1=np.vstack((DOF1TransformNode1,np.hstack((np.transpose(T_Act3_F2[0:2,-1]), np.hstack((np.array([0.]), rot_quat3_F2)) )) ))

PPi=positionList
positionList=positionList.tolist()
DOF0TransformNode0=DOF0TransformNode0.tolist()
DOF1TransformNode1=DOF1TransformNode1.tolist()

# print("edgesList = ", edgesList)
# print('positionListTest=',positionList)
# print("DOF0TransformNode0 = ", DOF0TransformNode0)
# print('DOF1TransformNode1=',DOF1TransformNode1)
print("end")