
import stretch_body.robot
import sys, time, traceback
import numpy as np

#Startup, connect, and pull out different objects from robot object   
robot=stretch_body.robot.Robot()
robot.startup()
#Go to initial position first
robot.lift.move_to(0.5) 
robot.arm.move_to(0.3)
robot.end_of_arm.move_to('wrist_yaw',0)
robot.end_of_arm.move_to('stretch_gripper',25)		##(50 to -100)
robot.push_command()
time.sleep(3)
now=time.time()
while True:
	try:
		#Command joint to move sinusoidal motion
		robot.lift.move_to(x_m=0.5+0.2*np.sin((time.time()-now)/2.),stiffness = 0.1)
		#arm.move_to(0.3+0.28*np.cos((time.time()-now)/2.))
		#end_of_arm.move_to('wrist_yaw',np.sin((time.time()-now)/2.))
		#end_of_arm.move_to('stretch_gripper',25+25*np.sin((time.time()-now)/2.))
		#base.translate_by(0.01*np.sin((time.time()-now)/2.))
		robot.push_command()
		#print force reading from arm
		time.sleep(0.05)
	except:
		traceback.print_exc()
		break

print ('Retracting...')
robot.lift.move_to(0.3)
robot.arm.move_to(0.0)
robot.end_of_arm.move_to('wrist_yaw',0.)
robot.end_of_arm.move_to('stretch_gripper',0.)
robot.push_command( )
time.sleep(3)
robot.stop()