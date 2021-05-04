import sys, time
import numpy as np

q_B = 1
q_3 = 1
q_2 = 1
P23_0 = 1
P3T_y = 1

Jp = np.array([ [np.cos(q_B), -1*np.cos(q_B)*P23_0 - np.cos(q_B)*q_2 - np.cos(q_B+q_3)*P3T_y, 0, -1*np.sin(q_B), -1*np.cos(q_B+q_3)*P3T_y],
				[np.sin(q_B), -1*np.sin(q_B)*P23_0 - np.sin(q_B)*q_2 - np.sin(q_B+q_3)*P3T_y, 0, np.cos(q_B), -1*np.sin(q_B+q_3)*P3T_y],
				[0, 0, 1, 0, 0]])
Jr = np.array([[0, 1, 0, 0, 1]])

Jp1 = np.array([[np.cos(q_B), -1*np.cos(q_B)*P23_0 - np.cos(q_B)*q_2 - np.cos(q_B+q_3)*P3T_y, 0, -1*np.sin(q_B)], 
				[np.sin(q_B), -1*np.sin(q_B)*P23_0 - np.sin(q_B)*q_2 - np.sin(q_B+q_3)*P3T_y, 0, np.cos(q_B)],
				[0, 0, 1, 0]])
Jr1 = np.array([[0, 1, 0, 0]])

J1 = np.concatenate((Jr1,Jp1), axis=0)
extra_zero = np.zeros((2,4))
J1 = np.concatenate((extra_zero,J1), axis=0)

J1_T = np.transpose(J1)

J = np.concatenate((Jr, Jp), axis=0)
#print(J)
print(J1_T)