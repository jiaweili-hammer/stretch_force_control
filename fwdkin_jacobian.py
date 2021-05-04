from general_robotics_toolbox import Robot, fwdkin, robotjacobian

import sys, time
import numpy as np


# forward kinematics & Jacobian setup
ex=np.array([[1],[0],[0]])
ey=np.array([[0],[1],[0]])
ez=np.array([[0],[0],[1]])

H=np.concatenate((ez,-1*ey),axis=1)
p01=np.array([[0],[0],[0]])
p12=np.array([[0],[0],[0]])
p2T=np.array([[0],[0],[0]])

num_joints=2
P=np.concatenate((p01,p12,p2T),axis=1)
joint_type=np.ones(num_joints)
theta = [0,0]

robot_def=Robot(H,np.transpose(P),joint_type)
print(robotjacobian(robot_def,theta))