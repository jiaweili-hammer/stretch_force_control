
from RobotRaconteur.Client import *     #import RR client library
import sys, time
import numpy as np
url='rr+tcp://192.168.1.64:23232/?service=stretch'

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
#end_of_arm_status = end_of_arm.status_rr.Connect()

#Go to initial position first
lift.move_to(0.5) 
arm.move_to(0.3)
end_of_arm.move_to('wrist_yaw',0)
end_of_arm.move_to('stretch_gripper',25) ##(50 to -100)
base.translate_by(0.1)		
robot.push_command()
time.sleep(3)
now=time.time()
data = np.zeros(500)
count = 0
while True:
	try:
		#Command joint to move sinusoidal motion
		#print force reading from arm
		#time.sleep(0.1)
		#print(base_status.InValue['translation_force']) #base torque
		#print(end_of_arm_status.InValue['theta'])
		#print(base_status.InValue['theta'])
		#print(arm_status.InValue['force'])
		#print(arm_status.InValue['motor']['current'])
		
		if count<500:
			data[count] = lift_status.InValue['force']
			count = count+1
		else:
			break
		time.sleep(0.05)
	except:
		break

print ('Retracting...')
np.save('sensor_data.npy',data)
lift.move_to(0.3)
arm.move_to(0.0)
end_of_arm.move_to('wrist_yaw',0.)
end_of_arm.move_to('stretch_gripper',0.)
robot.push_command( )
time.sleep(3)