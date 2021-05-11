from general_robotics_toolbox import Robot, fwdkin, robotjacobian
from RobotRaconteur.Client import *     #import RR client library
import sys, time
import numpy as np
from scipy.signal import filtfilt
from scipy import stats
import matplotlib.pyplot as plt
import scipy


url='rr+tcp://192.168.1.64:23232/?service=stretch'

def bandpassfilter(signal):
	fs = 25.0
	lowcut = 2
	#highcut = 50.0

	nyq = 0.5*fs
	low = lowcut / nyq
	#high = highcut / nyq

	order = 6
	b,a = scipy.signal.butter(order, low, btype='low', analog=False)
	y = scipy.signal.filtfilt(b,a,signal, axis=0)

	return y

#Startup, connect, and pull out different objects from robot object   
robot=RRN.ConnectService(url)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()
base=robot.get_base()

#Connect to arm status RR wire
arm_status=arm.status_rr.Connect()
lift_status=lift.status_rr.Connect()
base_status=base.status_rr.Connect()
time.sleep(2)

'''
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
robot_def=Robot(H,np.transpose(P),joint_type)
'''

# generalized damper setting, dim:6X4
D_des = 500*np.eye(4)

# start the robot
lift.move_to(0.5) 
arm.move_to(0.3)
base.translate_by(0.05)
robot.push_command()
time.sleep(3)
print('controller starts')

while True:
	try:
		print('Reading status')
		q_B = base_status.InValue['theta']
		#q_3 = end_of_arm.InValue['theta']
		q_3 = 3.14
		q_2 = arm_status.InValue['pos']
		P23_0 = -0.28
		P3T_y = -0.05
		
		
		print('Reading force')

		#dim: 4X1
		#FT = np.array([[0],[arm_status.InValue['force']],[-lift_status.InValue['force']]])
		#FT_raw = np.array([[base_status.InValue['rotation_torque']], [-base_status.InValue['translation_force']],[arm_status.InValue['force']],[-lift_status.InValue['force']]])
		
		count = 0
		base_torque = [];
		base_force = [];
		arm_force = [];
		lift_force = [];
		while count < 25:
			print('checkpoint',count)
			base_torque.append(base_status.InValue['rotation_torque'])
			base_force.append(-base_status.InValue['translation_force'])
			arm_force.append(arm_status.InValue['force'])
			lift_force.append(-lift_status.InValue['force'])
			count += 1
		base_torque_filtered = np.array(bandpassfilter(base_torque))
		base_force_filtered = np.array(bandpassfilter(base_force))
		arm_force_filtered = np.array(bandpassfilter(arm_force))
		lift_force_filtered = np.array(bandpassfilter(lift_force))

		base_torque_mean = np.mean(base_torque_filtered)
		base_force_mean = np.mean(base_force_filtered)
		arm_force_mean = np.mean(arm_force_filtered)
		lift_force_mean = np.mean(lift_force_filtered)

		FT_filtered = np.array([[base_torque_mean], [base_force_mean], [arm_force_mean], [lift_force_mean]])
		D_des_inv = np.linalg.inv(D_des)

		print('D_des_inv:',D_des_inv)
		
		v_eef = np.dot(D_des_inv,FT_filtered)
		#sudo_J = np.linalg.pinv(robotjacobian(robot_def,theta)[3:]) #sudo inverse of J_translate

		print('check1')
		# J_translation
		Jp = np.array([[np.cos(q_B), -1*np.cos(q_B)*P23_0 - np.cos(q_B)*q_2 - np.cos(q_B+q_3)*P3T_y, 0, -1*np.sin(q_B), -1*np.cos(q_B+q_3)*P3T_y],
				[np.sin(q_B), -1*np.sin(q_B)*P23_0 - np.sin(q_B)*q_2 - np.sin(q_B+q_3)*P3T_y, 0, np.cos(q_B), -1*np.sin(q_B+q_3)*P3T_y],
				[0, 0, 1, 0, 0]])
		print('check2')
		# J_rotation
		Jr = np.array([[0, 1, 0, 0, 1]])

		#J = J_translation + J_rotation
		J = np.concatenate((Jr, Jp), axis=0)

		#sudo inverse of J
		sudo_J = np.linalg.pinv(J)
		print('sudo_J:',sudo_J.shape)

		q =np.dot(sudo_J,v_eef)
		print('dim of q', q.shape)
		print('Push command')
		base.rotate_by(q[0]*0.25)
		base.translate_by(q[1]*0.25)
		lift.move_by(q[2]*0.025)
		arm.move_by(q[3]*0.025)

		robot.push_command()
		print(q)
		time.sleep(0.1)
		# print(fwdkin(robot_def, theta))
		# print(robotjacobian(robot_def, theta))
	except:
		break

print ('Program ending...')
lift.move_to(0.5)
arm.move_to(0.0)
end_of_arm.move_to('wrist_yaw',0.)
end_of_arm.move_to('stretch_gripper',0.)
base.translate_by(-0.05)
robot.push_command( )
time.sleep(3)