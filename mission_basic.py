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

        
def modifyMission():
    print("Hello mission modifier")
    vehicle.home_location = LocationGlobal(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon,vehicle.home_location.lat)
    cmds = vehicle.commands
    currentCollisionWaypoint = vehicle.commands.next - 1
    missionlist=[]
    missionlist.append(nav_command(LocationGlobalRelative(collisionHandleCoordinates[0], collisionHandleCoordinates[1], altitude)))
    for index, cmd in enumerate(cmds):
        if(index > currentCollisionWaypoint):
            missionlist.append(cmd)
    print("Old mission size: ", len(cmds))
    cmds.clear()
    vehicle.commands.next = 0
    # print(missionlist)
    print("New mission size: ",len(missionlist))
    print("not inserted commands: ",currentCollisionWaypoint)
    print("next command",vehicle.commands.next )
    # time.sleep(0.2)
    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    time.sleep(1)

def follow_waypoints(vehicle):
    global collisionPending
    global collisionHandling
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands):
        if(collisionPending):
            modifyMission()
            nextwaypoint = vehicle.commands.next
            print(nextwaypoint)
            collisionPending = False
        elif vehicle.commands.next > nextwaypoint:
            if(collisionHandling is not None) :
                readtr1w.checkingAirplanes = True
                del readtr1w.airplanes[collisionHandling]
                readtr1w.checkingAirplanes = False
                collisionHandling = None
            print("Moving to waypoint %s" % vehicle.commands.next)
            nextwaypoint = vehicle.commands.next
        # if(nextwaypoint == 3):
        #     vehicle.commands.clear()
        #     PX4RTL(vehicle)
        #     break
        time.sleep(1)
    while vehicle.commands.next > 0: #last command
        time.sleep(1)

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

    # move west
    wp = get_location_offset_meters(wp, 0, -radius, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move south
    wp = get_location_offset_meters(wp, -2*radius, 0, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move east
    wp = get_location_offset_meters(wp, 0, 2*radius, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    wp = get_location_offset_meters(wp, 2*radius, 0, 0);
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    wp = get_location_offset_meters(wp, 0, -radius, 0);
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
    cmds.add(takeoff_command(wp)) 
    cmds.add(loiter_command(wp,5))
    cmds.add(land_command(wp))
    cmds.upload()
    time.sleep(1)
    execMission()


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
    x = enemyPos.lat - myPos.lat
    y = enemyPos.lon - myPos.lon
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
    

def avoidCollision(ref, my_pos, enemy_pos, my_heading, enemy_heading, my_vel, enemy_vel, ICAO):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    multiplicatorX = earth_radius / (180/math.pi)
    multiplicatorY = (earth_radius*math.cos(math.pi*ref.lat/180)) / (180/math.pi)
    x0t = (my_pos.lat - ref.lat)*multiplicatorX
    y0t = (my_pos.lon - ref.lon)*multiplicatorY
    x0j = (enemy_pos.lat - ref.lat) * multiplicatorX
    y0j = (enemy_pos.lon - ref.lon) * multiplicatorY
    #print("my coords: ", x0t, ", ", y0t, ", enemy's coords:", x0j, ", ", y0j)
    ##derivative to find closest point between our paths
    a = (my_vel * math.cos(math.radians(my_heading)) - enemy_vel * math.cos(math.radians(enemy_heading)))
    b = (my_vel * math.sin(math.radians(my_heading)) - enemy_vel * math.sin(math.radians(enemy_heading)))
    t = (-(x0t - x0j)*a - (y0t - y0j)*b) / (a*a + b*b)
    #print("minimum for time:", t)
    d = math.sqrt(math.pow(x0j - x0t - t * a, 2) + math.pow(y0j - y0t - t*b, 2))
    if(d < 15 and  t > 0):
        ##avoid collision
        R=15
        alpha = 0.0
        AC = 0.0
        AB = math.sqrt(math.pow(x0t - x0j, 2) + math.pow(y0t - y0j, 2))
        if(AB < R): 
            AC = R
            alpha = 90.0
        else: 
            AC = math.sqrt(AB*AB - R*R) 
            alpha = math.degrees(math.asin(R/AB))
        print("angle to turn right: ", alpha)

        x= enemy_pos.lat - my_pos.lat
        y= enemy_pos.lon - my_pos.lon
        length = math.sqrt((x*x) + (y*y))
        enemyPosVector = [x, y]
        myDirVector = [math.cos(math.radians(my_heading)), math.sin(math.radians(my_heading))]
        angleBetweenUs = math.degrees(angle_between(myDirVector, enemyPosVector))
        crossProduct = x*myDirVector[1] - y * myDirVector[0]
        if(crossProduct < 0):
            finalAngle = my_heading + angleBetweenUs + alpha
        else:
            finalAngle = my_heading - angleBetweenUs + alpha
        #print("angle between us ", angleBetweenUs)
        if(finalAngle > 360): finalAngle = finalAngle - 360
        print("displacement length: ", AC)
        collisionAvoidX =(math.cos(finalAngle) * AC) + x0t
        collisionAvoidY = (math.sin(finalAngle) * AC) + y0t
        finalCoordinateLat = collisionAvoidX / multiplicatorX + ref.lat
        finalCoordinateLon = collisionAvoidY / multiplicatorY + ref.lon
        print("collisionAvoidCoordinates: ", collisionAvoidX - x0t, ", ", collisionAvoidY - y0t)
        print("collisionAvoidCoordinates relative to home: ", collisionAvoidX, ", ", collisionAvoidY )
        #print("my coordinates: ", my_pos.lat, ", ", my_pos.lon)
        #print("collision avoidance coordinates: ", finalCoordinateLat, ", ", finalCoordinateLon)
        #mdify mission
        scheduleModification((finalCoordinateLat, finalCoordinateLon), ICAO)


def scheduleModification(latlon, ICAO):
    global collisionHandling
    global collisionHandleCoordinates
    global collisionPending
    if(collisionHandling is None):
        print("Hello modification scheduler")
        collisionHandleCoordinates = latlon
        collisionPending = True
        collisionHandling = ICAO


def checkAirplanesDistance(run_event):
    MAX_RADIUS = 20
    global collisionHandling
    global currentCollisionWaypoint
    while run_event.is_set():
        vehicle_pos = vehicle.location.global_relative_frame
        time.sleep(0.5)
        if(collisionHandling is not None ):
            continue
        readtr1w.checkingAirplanes = True
        for ICAO, airplaneData in readtr1w.airplanes.items():
            loc = LocationGlobalRelative(airplaneData["latitude"], airplaneData["longitude"])
            dist = get_distance_metres(vehicle_pos, loc)
            #print("Distance: ", dist)


            if(dist < 10): 
                print("Game Over")
                continue
            if(dist < MAX_RADIUS and not havePriority(vehicle_pos, loc, vehicle.heading)):
                print("collision detected")
                avoidCollision(vehicle.home_location, vehicle_pos, loc, vehicle.heading,
                    airplaneData["dir"], np.linalg.norm(vehicle.velocity), airplaneData["h_velocity"], ICAO)
                # print("Closest calculated distance: ", dist)

        readtr1w.checkingAirplanes = False

collisionPending = False
collisionHandling = None
collisionHandleCoordinates = None


vehicle = connectPixhawk()
vehicle.groundspeed = 5
altitude = 4
radius = 20

readtr1w.addFakeAirPlane(vehicle.location.global_relative_frame, radius + 10)

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
except:
    run_event.clear()
    tr1wDataThread.join()
    print("Thread closed")
    raise
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

