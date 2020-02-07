#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import socket
import struct
import time
import math as m
import numpy as np
from simple_pid import PID
import pyrealsense2 as rs
import transformations as tf
import pickle
import select
import datetime
#import matplotlib.pyplot as plt

def DriveWheels(rpmR, rpmL):
    udpPacket = struct.pack('ff', rpmR, rpmL)  # input float values
    moab_sock.sendto(udpPacket, (MOAB_COMPUTER, MOAB_PORT))

############################# UDP connection #############################

moab_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
plot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Send to MOAB
MOAB_COMPUTER = "192.168.8.20"
ROBOT_IP = "127.0.0.1"
GUI_IP = "192.168.8.181"

MOAB_PORT = 12346
SBUS_PORT = 31338
GUI_PORT = 5555
PLOT_PORT = 24688

moab_sock.bind(('0.0.0.0', 0))
#sbus_sock.bind(('0.0.0.0',SBUS_PORT))

#drive_flag_sock.bind((ROBOT_IP, DRIVE_FLAG_PORT))

#drive_flag_sock.setblocking(0)  # set to non-blocking mode

# set rover to zeros
DriveWheels(0.0, 0.0)
print("					wait a sec to start....")
print("					switch ch5 to auto-mode")
time.sleep(1.0)

########################### T265 Initialization ###########################
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

################################ Parameters ###############################
Rwheel = 0.1534  # meters
sensorOffset = 0.49 # meter, from sensor to center of the wheels
distLong = 50.0
distShort = 1.0
x_target = [0, distShort, distShort, 0]
y_target = [distLong-sensorOffset, distLong-sensorOffset, -sensorOffset, -sensorOffset]
heading_sp = [90.0, 180.0, -90.0, 0.0]
constSpeed = 30.0   # use 40 it will has position error of 11cm
tol = 0.005 #  tolerance
once = True
k = 0
TURN_FLAG = False
GO_FLAG = True

################################ PID config ###############################
Kp_Line = 90.0  
Ki_Line = 25.0
Kd_Line = 380.0

maxExtraSpeed = 10.0

pid_Line = PID(Kp_Line, Ki_Line, Kd_Line, setpoint=0.0)
pid_Line.sample_time = 0.005
pid_Line.output_limits = (-maxExtraSpeed, maxExtraSpeed)

#################################################################
Kp_Turn = 8.0  
Ki_Turn = 0.0
Kd_Turn = 1.50

maxTurningSpeed = 20.0

pid_turn = PID(Kp_Turn, Ki_Turn, Kd_Turn)
pid_turn.sample_time = 0.005
pid_turn.output_limits = (-maxTurningSpeed, maxTurningSpeed)



################################ Loop ###############################
#allSockets = [sbus_sock]
try:

	while True:

		startTime = time.time()

		# Wait for the next set of frames from the camera
		frames = pipe.wait_for_frames()

		# Fetch pose frame
		pose = frames.get_pose_frame()

		# If there is any data comes
		if pose:
			# Print some of the pose data to the terminal
			data = pose.get_pose_data()

			position = data.translation
			velocity = data.velocity

			H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
			# transform to aeronautic coordinates (body AND reference frame!)
			H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
			rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz') ) # Rz(yaw)*Ry(pitch)*Rx(roll) body w.r.t. reference frame

			rpy_deg = rpy_rad*180/m.pi

			#print("position: ", position)  # to call is separtely position.x  position.y  position.z
			#print("rpy_deg: ",rpy_deg)  # rpy_deg[0] = roll, rpy_deg[1] = pitch, rpy_deg[2] = yaw
			
			currentHeading = rpy_deg[2]

			xc = position.x
			yc = (position.z)*(-1.0)

			xOffset = sensorOffset*m.sin(rpy_rad[2])
			yOffset = sensorOffset*m.cos(rpy_rad[2])

			xu = xc - xOffset
			yu = yc - yOffset

			if once:
				ugv_origin = [xu,yu]
				once = False	

			packet = struct.pack("ff",xu,yu)
			plot_sock.sendto(packet,(GUI_IP, PLOT_PORT))
			print("xu: " + str(round(xu,4)) + "   yu:" + str(round(yu,4)) + "   head:"+ str(round(currentHeading,2)))
			'''
			#diff_x = x_target[0] - xu
			#diff_y = y_target[0] - yu

			dist_error = [y_target[0]-yu, x_target[1]-xu, y_target[2]-yu, x_target[3]-xu]
			cross_track_error = [x_target[0]-xu, y_target[1]-yu, x_target[2]-xu, y_target[3]-yu]
			control_var = [xu, yu, xu, yu]
			target_var = [x_target[0], y_target[1], x_target[2], y_target[3]]


			if GO_FLAG == True:

				#print("GO_FLAG",GO_FLAG)
				#print("TURN_FLAG", TURN_FLAG)
				#print("dist_error[k]", round(dist_error[k],4))
				if abs(dist_error[k]) > 0.05:

					extraSpeed = pid_Line(cross_track_error[k])
					
					if (control_var[k] < target_var[k]) and (abs(cross_track_error[k]) > tol):

						if k == 0 or k == 3:
							# reduce right rpm speed
							rpmL = constSpeed 
							rpmR = constSpeed - (-extraSpeed)
						else:
							# reduce left rpm speed
							rpmL = constSpeed - (-extraSpeed)
							rpmR = constSpeed 

					elif (control_var[k] > target_var[k]) and (abs(cross_track_error[k]) > tol):
						if k == 0 or k == 3:
							# redice left rpm speed
							rpmL = constSpeed - (extraSpeed)
							rpmR = constSpeed 
						else:
							# redice left rpm speed
							rpmL = constSpeed 
							rpmR = constSpeed - (extraSpeed)
					else:
						rpmL = constSpeed
						rpmR = constSpeed

					DriveWheels(rpmR, rpmL)

				else:
					DriveWheels(0.0, 0.0)
					#print("DONE GO")
					time.sleep(1)
					TURN_FLAG = True
					GO_FLAG = False


			if TURN_FLAG == True:
				# In case of turning to 180deg the normal currentHeading cannot go beyond 180
				# so if it becomes negative number -179deg just change it to 181deg and etc.
				#print("currentHeading: " + str(round(currentHeading,2)))
				#print("heading setpoint: " + str(round(heading_sp[k],2)))
				if k==1:
					if currentHeading < 0.0:
						# target angle of this case is 180, we convert the angle back to plus zone
						currentHeading = 360.0 + currentHeading
				elif k==2:
					if currentHeading > 0.0:
						# target angle of this case is -90, we convert the angle back to minus zone
						currentHeading = currentHeading - 360.0

				if abs(currentHeading-heading_sp[k]) > 0.5:
					pid_turn.setpoint = heading_sp[k]
					outputPID = pid_turn(currentHeading)
					rpmL = outputPID
					rpmR = -rpmL
					DriveWheels(rpmR, rpmL)
				else:
					#print("DONE TURN")
					DriveWheels(0.0, 0.0)
					TURN_FLAG = False
					GO_FLAG = True
					time.sleep(1)
					
					k = k+1
					if k == 4:
						k = 0
						GO_FLAG = True
						TURN_FLAG = False

				#print("GO_FLAG",GO_FLAG)
				#print("TURN_FLAG", TURN_FLAG)
				#print("k: ",k)
			
			print("k: "+str(k)+"  xu: "+"{:.4f}".format(xu)+"  yu:"+"{:.4f}".format(yu)+\
				"  x_target: "+"{:.2f}".format(x_target[k])+"  y_target:"+"{:.2f}".format(y_target[k])+\
				"  rpmL: "+"{:.2f}".format(rpmL)+"  rpmR: "+"{:.2f}".format(rpmR)+\
				"  cur_head: "+"{:.2f}".format(currentHeading)+"  sp_head: "+"{:.2f}".format(heading_sp[k]))
			'''
			'''
			for i in range(5):
				print("")

			print("xu: " + str(round(xu,4)) + "   yu:" + str(round(yu,4)))
			print("x_target: " + str(x_target[k]) + "		y_target:" + str(y_target[k]))
			print("rpmL: " + str(round(rpmL,4)) + "		rpmR: " + str(round(rpmR,4)))
			print("currentHeading: " + str(round(currentHeading,2)))
			print("heading setpoint: " + str(round(heading_sp[k],2)))
			print("k: ", k)
			'''
		period = time.time() - startTime
		#print("period", period)

finally:
	print("				End of rover control")
	DriveWheels(0.0,0.0)
	pipe.stop()
