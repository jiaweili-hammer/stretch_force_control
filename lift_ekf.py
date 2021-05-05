from RobotRaconteur.Client import *
import numpy as np 
from filterpy.kalman import ExtendedKalmanFilter
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
time.sleep(2)


ekf = ExtendedKalmanFilter(dim_x=3, dim_z=1,dim_u=1)

# define initial state
kf.x = np.array([[0., 0., 3.]])

# define A matrix
kf.F = np.array([[0.]])

# define B matrix
kf.B = np.array([[75.]])

# define C matrix
kf.H = np.array([[1.]])

# define cov matrix for initial state
kf.P = kf.P*100

# define cov matrix for measurement noise
kf.R = np.array([[5.]])

kf.Q = Q_discrete_white_noise(dim=1,dt=0.05, var=0.13)

count = 0
xs = []
cov = []
raw = []
while count < 500:
	u = np.array([[lift_status.InValue['motor']['current']-0.54]])
	kf.predict(u)
	z = lift_status.InValue['force']
	kf.update(z)
	xs.append(kf.x)
	cov.append(kf.P)
	raw.append(z)
	count += 1
xs = np.array(xs)
cov = np.array(cov)
raw = np.array(raw)

fig,ax = plt.subplots()
num = np.linspace(0,count,count+1)
ax.plot(num,xs,label='KF')
ax.plot(num,raw,label='Raw')
ax.legend()
