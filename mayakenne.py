'''
	Made by Kenneth Quinones 
'''

##### DEPENDENCIES #####

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
import time
import socket
import math
import exceptions
import argparse
from pymavlink import mavutil

##### FUNCTIONS #####

##### Copter Connections #####
def connectionCopter():

	connection_string = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A4008qL9-if00-port0'
	#connection_string = 'tcp:127.0.0.1:5760'
	vehicle = connect(connection_string,wait_ready=True)
	return vehicle

##### Simple Take Off #####

def arm_takeoff(how_far):
	while vehicle.is_armable != True:
		print("Waiting for vehicle to become armable")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode != 'GUIDED':
		print("Waiting for the drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE")
	vehicle.armed = True

	while vehicle.armed==False:
		print("Waiting to vehicle to become armed")
		time.sleep(1)
	print("Must be turn and spinning!!")

	vehicle.simple_takeoff(how_far) ### Height in meters change also in below *change in the future to a variable 

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*how_far:
			break
		time.sleep(1)
	print("Target altitude reached")
	return None

##### Main #####

vehicle = connectionCopter()
homewp=vehicle.location.global_relative_frame
altura = 2

cmd1=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,0,0,0,0,0,homewp.lat,homewp.lon,altura)
cmd2=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,homewp.lat,homewp.lon,altura)

cmd3=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,0,0,0,0,0,18.212402,-67.144740,altura)
cmd4=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,18.212402,-67.144740,altura)

cmd5=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,0,0,0,0,0,18.212353,-67.144783,altura-1)
cmd6=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,18.212353,-67.144783,altura-1)


##Donwload current list of commands FORM the drone were connected to
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

###Clear the current list of commands
cmds.clear()

###Add in our new commands
cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)
cmds.add(cmd5)
cmds.add(cmd6)


###Upload our commands to the drone
vehicle.commands.upload()

##### Save Home Location #####
homewp=vehicle.location.global_relative_frame

arm_takeoff(5)

print("After arm and Takeoff")
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode !="AUTO":
	time.sleep(.2)

while vehicle.location.global_relative_frame.alt>2:
	print ("Global Location: %s" % vehicle.location.global_frame)
	time.sleep(2)
	
vehicle.mode = VehicleMode("LAND")
while(vehicle.mode != 'LAND'):
	print("Landing Drone Please Wait")
	time.sleep(1)
print("Drone on floor")
while True:
	time.sleep(1)

vehicle.mode = VehicleMode("STABILIZE")
while vehicle.mode.name == 'STABILIZE':
	time.sleep(1)
print("Disarm Motors")
vehicle.armed =False
while vehicle.armed:
	time.sleep(1)
print("Crash")
vehicle.close()

