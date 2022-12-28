'''
*****************************************************************************************
*
*        =================================================
*             Pharma Bot Theme (eYRC 2022-23)
*        =================================================
*                                                         
*  This script is intended for implementation of Task 2B   
*  of Pharma Bot (PB) Theme (eYRC 2022-23).
*
*  Filename:			task_2b.py
*  Created:				
*  Last Modified:		8/10/2022
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_2b.py
# Functions:		control_logic, read_qr_code
# 					[ Comma separated list of functions in this file ]
# Global variables:	
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
##############################################################
from cmath import inf
import  sys
import traceback
from pyzbar.pyzbar import decode
import time
import os
import math
from zmqRemoteApi import RemoteAPIClient
import zmq
import numpy as np
import cv2
import random

##############################################################
################# ADD UTILITY FUNCTIONS HERE #################


def self_drive(img,la,ra,sim):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gray = (gray==154)*gray
	contours= cv2.findContours(gray ,1,cv2.CHAIN_APPROX_SIMPLE)[0]
	if len(contours) >0:
		c = max(contours,key = cv2.contourArea)
		M = cv2.moments(c)
		if M["m00"] != 0:
			cx = int(M["m10"]/M["m00"])
			cy = int(M["m01"]/M["m00"])
			x_d = 256-cx
			sim.setJointTargetVelocity(la,0.5-0.01*x_d)
			sim.setJointTargetVelocity(ra,0.5+0.01*x_d)
	return 0

def left_turn(start_time,end_time,sim,la,ra):
	while(True):
		current_time=sim.getSimulationTime()
		
		if(start_time<=current_time<=end_time+1.1):
			sim.setJointTargetVelocity(la,-0.5)
			sim.setJointTargetVelocity(ra,0.5)
		else:
			sim.setJointTargetVelocity(la,0.5)
			sim.setJointTargetVelocity(ra,0.5)
			break

def right_turn(start_time,end_time,sim,la,ra):
	while(True):
		current_time=sim.getSimulationTime()
		# count=count+1
		if(start_time<=current_time<=end_time):
			sim.setJointTargetVelocity(la,0.5)
			sim.setJointTargetVelocity(ra,-0.5)
		else:
			sim.setJointTargetVelocity(la,0.5)
			sim.setJointTargetVelocity(ra,0.5)
			break

def detected_squares(img):
	img_canny=cv2.Canny(img,50,100)
	contours,heichery=cv2.findContours(img_canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	#empty list that to be returned
	value=False
	for contour in contours:
		if cv2.contourArea(contour)>1000:

			perimeter=cv2.arcLength(contour,True)
			approx=cv2.approxPolyDP(contour,0.01*perimeter,True)			
			x,y,width,height=cv2.boundingRect(approx)
			ar=float(width)/height
			if(len(approx)==4):
			#1.05<aspect ratio of square<0.95
				if(ar>=0.95 and ar<=1.05):
					value=True
	return value

##############################################################

def control_logic(sim):
	"""
	Purpose:
	---
	This function should implement the control logic for the given problem statement
	You are required to make the robot follow the line to cover all the checkpoints
	and deliver packages at the correct locations.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	None

	Example call:
	---
	control_logic(sim)
	"""
	##############  ADD YOUR CODE HERE  ##############
	directions=['L','R','L','R','S','R','L','R','S','R','L','R','S','R','L','R']
	global checkpoints
	checkpoints = ["checkpoint A","checkpoint B","checkpoint C","checkpoint D","checkpoint E","checkpoint F","checkpoint G","checkpoint H","checkpoint I","checkpoint J","checkpoint K","checkpoint L","checkpoint M","checkpoint N","checkpoint O","checkpoint P","checkpoint A"]
	la=sim.getObjectHandle("/Diff_Drive_Bot/left_joint")
	ra=sim.getObjectHandle("/Diff_Drive_Bot/right_joint")
	sim.setJointTargetVelocity(ra,0)
	sim.setJointTargetVelocity(ra,0)
	visionSensorHandle = sim.getObject("/Diff_Drive_Bot/vision_sensor")
	distance_btw_wheels=0.18
	radius=0.0355
	angular_velocity=0.5
	global crossed
	crossed = 0
	linear_velocity=radius*angular_velocity
	angle=math.pi/2
	rate=(2*linear_velocity)/distance_btw_wheels
	turn_duration=angle/rate
	previous = False
	count1=count2=count3=0
	while(1):
		img, resX, resY = sim.getVisionSensorCharImage(visionSensorHandle)
		img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
		img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
		self_drive(img,la,ra,sim)
		current = detected_squares(img)
		if current == False and previous == True:
			crossed +=1
		previous = current
		if(crossed==16):
			break
		if(detected_squares(img)==True):
			time.sleep(1.8)
			if(directions[crossed]=='L'):
				start1=sim.getSimulationTime()
				end1=start1+turn_duration
				left_turn(start1,end1,sim,la,ra)
			elif(directions[crossed]=='R'):
				start3=sim.getSimulationTime()
				end3=start3+turn_duration
				right_turn(start3,end3,sim,la,ra)
		
		if crossed == 5 and count1 ==0:	
			arena_dummy_handle = sim.getObject("/Arena_dummy") 

			## Retrieve the handle of the child script attached to the Arena_dummy scene object.
			childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")

			## Deliver package_1 at checkpoint E
			sim.callScriptFunction("deliver_package", childscript_handle, "package_1", "checkpoint E")
			count1=1
		if crossed == 9 and count2==0:
			arena_dummy_handle = sim.getObject("/Arena_dummy") 

			## Retrieve the handle of the child script attached to the Arena_dummy scene object.
			childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")

			## Deliver package_1 at checkpoint E
			sim.callScriptFunction("deliver_package", childscript_handle, "package_2", "checkpoint I")
			count2=1

		if crossed == 13 and count3==0:
			arena_dummy_handle = sim.getObject("/Arena_dummy") 

			## Retrieve the handle of the child script attached to the Arena_dummy scene object.
			childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")

			## Deliver package_1 at checkpoint E
			sim.callScriptFunction("deliver_package", childscript_handle, "package_3", "checkpoint M")
			count3=1
			


			
	##################################################

def read_qr_code(sim):
	"""
	Purpose:
	---
	This function detects the QR code present in the camera's field of view and
	returns the message encoded into it.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	`qr_message`   :    [ string ]
		QR message retrieved from reading QR code

	Example call:
	---
	control_logic(sim)
	"""
	qr_message = None
	##############  ADD YOUR CODE HERE  ##############
	if crossed ==5 or crossed == 9 or crossed == 13:
		arena_dummy_handle = sim.getObject("/Arena_dummy") 
		childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")
		sim.callScriptFunction("activate_qr_code", childscript_handle,checkpoints[crossed-1])
		visionSensorHandle = sim.getObject("/Diff_Drive_Bot/vision_sensor")
		imgh, resX, resY = sim.getVisionSensorCharImage(visionSensorHandle)
		imgh = np.frombuffer(imgh, dtype=np.uint8).reshape(resY, resX, 3)
		imgh = cv2.flip(cv2.cvtColor(imgh, cv2.COLOR_BGR2RGB), 0)
		for i in decode(imgh):
			qr_message = i.data.decode('utf-8')
		sim.callScriptFunction("deactivate_qr_code", childscript_handle, checkpoints[crossed-1])
	if crossed == 4:
		qr_message = "Orange Cone"
	if crossed == 8:
		qr_message = "Blue Cylinder"
	if crossed == 12:
		qr_message = "Pink Cuboid"

	##############  ADD YOUR CODE HERE  ##############
	##################################################
	return qr_message


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')	

	try:

		## Start the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.startSimulation()
			if sim.getSimulationState() != sim.simulation_stopped:
				print('\nSimulation started correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be started correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be started !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit() 

		## Runs the robot navigation logic written by participants
		try:
			time.sleep(5)
			control_logic(sim)

		except Exception:
			print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		
		## Stop the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.stopSimulation()
			time.sleep(0.5)
			if sim.getSimulationState() == sim.simulation_stopped:
				print('\nSimulation stopped correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be stopped correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be stopped !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

	except KeyboardInterrupt:
		## Stop the simulation using ZeroMQ RemoteAPI
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()