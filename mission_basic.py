#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
from communications import *
from helperFunctions import *




def adds_square_mission(aLocation, aSize, altitude):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    cmds.clear() 
    
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, altitude))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_offset_meters(aLocation, aSize, -aSize, 0)
    point2 = get_location_offset_meters(aLocation, aSize, aSize, 0)
    point3 = get_location_offset_meters(aLocation, -aSize, aSize, 0)
    point4 = get_location_offset_meters(aLocation, -aSize, -aSize, 0)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, altitude))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, altitude))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, altitude))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, altitude))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, altitude))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, altitude))    

    print(" Upload new commands to vehicle")
    cmds.upload()
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).

    vehicle.armed = True

    print("Starting mission")
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0

    # Set mode to AUTO to start mission
    # vehicle.mode = VehicleMode("AUTO")


    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.
    PX4setMode(vehicle, MAV_MODE_AUTO)
    while True:
        nextwaypoint=vehicle.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint(vehicle)))
    
        # if nextwaypoint==4: #Skip to next waypoint
        #     print('Skipping to Waypoint 5 when reach waypoint 3')
        #     vehicle.commands.next = 5
        if nextwaypoint==7: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)

    print('Return to launch')
    vehicle.mode = VehicleMode("RTL")



def execMission():
    #arm_and_takeoff(altitude)
    vehicle.armed=True
    # monitor mission execution
    follow_waypoints(vehicle)

    vehicle.armed = False
    time.sleep(1)

def PX4mission(radius, altitude):

    PX4setMode(vehicle, MAV_MODE_AUTO)
    time.sleep(1)
    cmds = vehicle.commands
    cmds.clear() 
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, altitude);
    radius_to_pass_by = 5
    acceptance_radius = 5


    cmds.add(takeoff_command(wp))
    # move north
    wp = get_location_offset_meters(wp, radius, 0, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move east
    wp = get_location_offset_meters(wp, 0, radius, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move south
    wp = get_location_offset_meters(wp, -2*radius, 0, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move west
    wp = get_location_offset_meters(wp, 0, -2*radius, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    wp = get_location_offset_meters(wp, 2*radius, 0, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    wp = get_location_offset_meters(wp, 0, radius, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))
    # land
    wp = get_location_offset_meters(home, 0, 0, altitude);
    cmds.add(land_command(wp))

    # Upload mission
    cmds.upload()
    time.sleep(1)

    execMission()


def takeoff_land(altitude):
    cmds = vehicle.commands
    cmds.clear() 
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, altitude)
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, 0, 0, 5, 0, 3, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 2, 0, 0, 0, 0, wp.lat, wp.lon, 0)
    cmds.add(cmd)
    cmds.upload()
    time.sleep(1)
    execMission()


def testAutoMode():
    cmds = vehicle.commands
    cmds.clear() 
    vehicle.armed = True
    vehicle.commands.next=0
    vehicle.mode = VehicleMode("AUTO")
    time.sleep(5)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = False
    time.sleep(1)


def simple_goto(aTargetAltitude, radius):
    #NOT WORKING ON PX4
    vehicle.simple_takeoff(aTargetAltitude)

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, radius, radius, 0);
    vehicle.simple_goto(wp, groundspeed=20)
    time.sleep(15)
    wp = get_location_offset_meters(home, radius, -radius, 0);
    vehicle.simple_goto(wp, groundspeed=10)
    time.sleep(15)
    vehicle.mode = VehicleMode("RTL")

vehicle = connectPixhawk()

vehicle.groundspeed = 20
altitude = 8
radius = 20
PX4mission(radius, altitude)
#simple_goto(altitude, radius)
#RTL wznosi sie na 15 m w symulatorze
#adds_square_mission(vehicle.location.global_relative_frame,radius, altitude)
#takeoff_land(altitude)
#testAutoMode()

# Close vehicle object before exiting script
vehicle.close()
# time.sleep(1)

