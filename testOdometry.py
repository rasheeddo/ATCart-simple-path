#!/bin/bash/python

import pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time

H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

try:
	while(True):
		# Wait for the next set of frames from the camera
		frames = pipe.wait_for_frames()

		# Fetch pose frame
		pose = frames.get_pose_frame()

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
			for i in range(10):
				print("")
			print("position: ", position)  # to call is separtely position.x  position.y  position.z
			print("rpy_deg: ",rpy_deg)  # rpy_deg[0] = roll, rpy_deg[1] = pitch, rpy_deg[2] = yaw
			print("velocity", velocity)
			

			#posX = position.x
			#posY = position.y
			#posZ = position.z
			#print("posX", posX)
			#print("posY", posY)
			#print("posZ", posZ)
			time.sleep(0.1)

finally:
	pipe.stop()