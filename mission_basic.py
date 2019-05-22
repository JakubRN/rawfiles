#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import math, threading, time, readtr1w
from pymavlink import mavutil
from communications import *
from helperFunctions import *
import numpy as np

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

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def havePriority(myPos, enemyPos, myDir):
    x= enemyPos.lat - myPos.lat
    y= enemyPos.lon - myPos.lon
    length = math.sqrt((x*x) + (y*y))
    enemyPosVector = [x, y]
    myDirVector = [math.cos(math.radians(myDir)), math.sin(math.radians(myDir))]
    angle = math.degrees(angle_between(myDirVector, enemyPosVector))
    #print("My heading:", myDir, "angle between us: ", angle)
    crossProduct = x*myDirVector[1] - y * myDirVector[0]
    # if(crossProduct < 0):
    #     print("on the right")
    # else:
    #     print("on the left")
    if(angle < 45): return False
    if(crossProduct < 0 and angle < 135):
        #print("airplane to the right and within no-go zone")
        return False
    return True
    
def calculateClosestDistance(ref, my_pos, enemy_pos, my_heading, enemy_heading, my_vel, enemy_vel):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    multiplicatorX = earth_radius / (180/math.pi)
    multiplicatorY = (earth_radius*math.cos(math.pi*ref.lat/180)) / (180/math.pi)
    x0t = (my_pos.lat - ref.lat)*multiplicatorX
    y0t = (my_pos.lon - ref.lon)*multiplicatorY
    x0j = (enemy_pos.lat - ref.lat) * multiplicatorX
    y0j = (enemy_pos.lon - ref.lon) * multiplicatorY
    print("my coords: ", x0t, ", ", y0t, ", enemy's coords:", x0j, ", ", y0j)
    a = (my_vel * math.cos(math.radians(my_heading)) - enemy_vel * math.cos(math.radians(enemy_heading)))
    b = (my_vel * math.sin(math.radians(my_heading)) - enemy_vel * math.sin(math.radians(enemy_heading)))
    t = (-(x0t - x0j)*a - (y0t - y0j)*b) / (a*a + b*b)
    print("minimum for time:", t)
    d = math.sqrt(math.pow(x0j - x0t - t * a, 2) + math.pow(y0j - y0t - t*b, 2))
    return d
    



def checkAirplanesDistance(run_event):
    MAX_RADIUS = 20
    while run_event.is_set():
        vehicle_pos = vehicle.location.global_relative_frame
        readtr1w.checkingAirplanes = True
        for ICAO, airplaneData in readtr1w.airplanes.items():
            loc = LocationGlobalRelative(airplaneData["latitude"], airplaneData["longitude"])
            dist = get_distance_metres(vehicle_pos, loc)
            #print("Distance: ", dist)
            if(not havePriority(vehicle_pos, loc, vehicle.heading) and dist < MAX_RADIUS):
                velValue = np.linalg.norm(vehicle.velocity)
                #print("My velocity:", velValue)
                dist = calculateClosestDistance(vehicle.home_location, vehicle_pos, loc, vehicle.heading,
                             airplaneData["dir"], velValue, airplaneData["h_velocity"])
                print("Closest calculated distance: ", dist)

        readtr1w.checkingAirplanes = False
        time.sleep(0.2)

vehicle = connectPixhawk()
vehicle.groundspeed = 20
altitude = 4
radius = 10

readtr1w.addFakeAirPlane(vehicle.location.global_relative_frame, radius + 5)

run_event = threading.Event()
run_event.set()
tr1wDataThread = threading.Thread(target = checkAirplanesDistance, args = (run_event,))
tr1wDataThread.start()
try:
    PX4mission(radius, altitude)
except KeyboardInterrupt:
    run_event.clear()
    tr1wDataThread.join()
    print("Thread closed")
#simple_goto(altitude, radius)
#RTL wznosi sie na 15 m w symulatorze
#adds_square_mission(vehicle.location.global_relative_frame,radius, altitude)
#takeoff_land(altitude)
#testAutoMode()
run_event.clear()
tr1wDataThread.join()
print("Thread closed")
# Close vehicle object before exiting script
vehicle.close()
# time.sleep(1)

