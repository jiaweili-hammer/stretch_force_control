from RobotRaconteur.Client import *
import numpy as np 
import sys, time, traceback
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import stretch_body.robot
import matplotlib.pyplot as plt

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
arm_motor_status=arm.motor_status_rr.Connect()
time.sleep(2)


lift.move_to(0.5) 
arm.move_to(0.3)
end_of_arm.move_to('wrist_yaw',0)
end_of_arm.move_to('stretch_gripper',25)		##(50 to -100)
robot.push_command()
time.sleep(3)



L = 2.5/1000
V = 3.
R = 0.9
K = 54/1000
b = 3.5/1000000
J = 3.3/1000000


kf = KalmanFilter(dim_x=3, dim_z=1,dim_u=1)

# define initial state
kf.x = np.array([[0.], [0.], [3.]])

# define A matrix
kf.F = np.array([[0., 1., 0,],
				 [0., -b/J, K/J],
				 [0., -K/L, -R/L]])

# define B matrix
kf.B = np.array([[0.],
				 [0.],
				 [1/L]])

# define C matrix
kf.H = np.array([[0., 0., 1.]])

# define cov matrix for initial state
kf.P = kf.P*10

# define cov matrix for measurement noise
kf.R = kf.R*5

kf.Q = Q_discrete_white_noise(dim=3,dt=0.05, var=2.5)

count = 0
xs = []
cov = []
raw_current = []
while True:
	try:
		u = np.array([[3.]])
		kf.predict(u)
		z = np.array([[arm_motor_status.InValue['current']]])
		kf.update(z)
		xs.append(kf.x.item((2,0))) #
		cov.append(kf.P)
		raw_current.append(z.item((0,0)))
		count += 1
	except:
		break
filtered_current = np.array(xs)
cov = np.array(cov)
raw_current = np.array(raw_current)
print(np.shape(raw_current))
print('')
print(raw_current)
print('')
print(filtered_current)



print ('Retracting...')
lift.move_to(0.3)
arm.move_to(0.0)
end_of_arm.move_to('wrist_yaw',0.)
end_of_arm.move_to('stretch_gripper',0.)
robot.push_command( )
time.sleep(3)


fig,ax = plt.subplots()
number = np.linspace(0,count,num=count)
ax.plot(number,raw_current,label='Raw')
ax.plot(number,filtered_current,label='KF')
ax.set_ylim(-0.8,0.8)
ax.legend()
plt.show()
