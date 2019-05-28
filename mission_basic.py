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

collisionPending = False
collisionHandling = None
        
def modifyMission(vehicle, collisionHandleCoordinates):
    print("Hello mission modifier")
    cmds = vehicle.commands
    currentCollisionWaypoint = vehicle.commands.next - 1
    missionlist=[]
    missionlist.append(nav_command(LocationGlobalRelative(collisionHandleCoordinates[0], collisionHandleCoordinates[1], vehicle.location.global_relative_frame.alt)))
    for index, cmd in enumerate(cmds):
        if(index >= currentCollisionWaypoint):
            missionlist.append(cmd)
    print("Old mission size: ", len(cmds))
    cmds.clear()
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    vehicle.commands.next = 0
    print("New mission size: ",len(missionlist))
    print("not inserted commands: ",currentCollisionWaypoint)
    print("next command",vehicle.commands.next )

def follow_waypoints(vehicle):
    global collisionPending
    global collisionHandling
    nextwaypoint = vehicle.commands.next
    while (nextwaypoint < len(vehicle.commands)) or collisionPending:
        if(collisionPending):
            nextwaypoint = vehicle.commands.next
            print(nextwaypoint)
            collisionPending = False
        elif vehicle.commands.next > nextwaypoint:
            if(collisionHandling is not None) :
                while (readtr1w.checkingAirplanes):
                    time.sleep(0.05)
                # for debugging purpose only
                readtr1w.checkingAirplanes = True
                readtr1w.airplanes.pop(collisionHandling)
                readtr1w.checkingAirplanes = False
                collisionHandling = None
            print("Moving to waypoint %s" % vehicle.commands.next)
            nextwaypoint = vehicle.commands.next
        time.sleep(1)
    while vehicle.commands.next > 0: #last command
        time.sleep(1)

def execMission(vehicle, altitude):
    #arm_and_takeoff(altitude)
    arm_and_takeoff(vehicle, altitude)
    vehicle.mode = VehicleMode("AUTO")
    # monitor mission execution
    follow_waypoints(vehicle)
    #only disarm 
    if(vehicle.location.global_relative_frame.alt < 1):
        vehicle.armed = False
    time.sleep(1)


def ArdupilotMission(vehicle, radius, altitude):

    cmds = vehicle.commands
    cmds.clear() 
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, altitude)
    radius_to_pass_by = 1
    acceptance_radius = 1

    cmds.add(takeoff_command(wp))
    # move north
    wp = get_location_offset_meters(wp, radius, 0, 0)
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move west
    wp = get_location_offset_meters(wp, 0, -radius, 0)
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move south
    wp = get_location_offset_meters(wp, -2*radius, 0, 0)
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    # move east
    wp = get_location_offset_meters(wp, 0, 2*radius, 0)
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    wp = get_location_offset_meters(wp, 2*radius, 0, 0)
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))

    wp = get_location_offset_meters(wp, 0, -radius, 0)
    cmds.add(nav_command(wp, acceptance_radius, radius_to_pass_by))
    # land
    wp = get_location_offset_meters(home, 0, 0, altitude)
    cmds.add(land_command(wp))

    # Upload mission
    cmds.upload()
    execMission(vehicle, altitude)


def takeoff_land(vehicle, altitude):
    cmds = vehicle.commands
    cmds.clear() 
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, altitude)
    cmds.add(takeoff_command(wp)) 
    cmds.add(loiter_command(wp,5))
    cmds.add(land_command(wp))
    cmds.upload()

    execMission(vehicle, altitude)

def simple_go_and_move_back(vehicle, altitude, radius):
    cmds = vehicle.commands
    cmds.clear() 
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, altitude)
    cmds.add(takeoff_command(wp)) 
    wp = get_location_offset_meters(wp, 0, -radius, 0)
    cmds.add(nav_command(wp,1,1))
    wp = get_location_offset_meters(wp, 0, radius, 0)
    cmds.add(nav_command(wp,1,1))
    wp = get_location_offset_meters(home, 0, 0, altitude)
    cmds.add(land_command(wp))
    cmds.upload()

    execMission(vehicle, altitude)


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
    

def avoidCollision(vehicle, enemy_pos, enemy_heading, my_vel, enemy_vel, ICAO):
    ref = vehicle.home_location
    my_pos = vehicle.location.global_relative_frame
    my_heading = vehicle.heading
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
        #print("angle to turn right: ", alpha)

        x= enemy_pos.lat - my_pos.lat
        y= enemy_pos.lon - my_pos.lon
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
        # print("displacement length: ", AC)
        #print("final angle: ", finalAngle)
        collisionAvoidX =(math.cos(math.radians(finalAngle)) * AC) + x0t
        collisionAvoidY = (math.sin(math.radians(finalAngle)) * AC) + y0t
        finalCoordinateLat = collisionAvoidX / multiplicatorX + ref.lat
        finalCoordinateLon = collisionAvoidY / multiplicatorY + ref.lon
        # print("collisionAvoidCoordinates: ", collisionAvoidX - x0t, ", ", collisionAvoidY - y0t)
        # print("collisionAvoidCoordinates relative to home: ", collisionAvoidX, ", ", collisionAvoidY )
        #print("my coordinates: ", my_pos.lat, ", ", my_pos.lon)
        #print("collision avoidance coordinates: ", finalCoordinateLat, ", ", finalCoordinateLon)
        #mdify mission
        scheduleModification(vehicle, (finalCoordinateLat, finalCoordinateLon), ICAO)


def scheduleModification(vehicle, latlon, ICAO):
    global collisionHandling
    global collisionPending
    if(collisionHandling is None):
        print("Hello modification scheduler")
        modifyMission(vehicle, latlon)
        collisionHandling = ICAO
        collisionPending = True


def checkAirplanesDistance(run_event, vehicle):
    MAX_RADIUS = 25
    while run_event.is_set():
        vehicle_pos = vehicle.location.global_relative_frame
        time.sleep(0.5)
        if(collisionHandling is not None ):
            continue
        while(readtr1w.checkingAirplanes):
            time.sleep(0.05)
        readtr1w.checkingAirplanes = True
        for ICAO, airplaneData in readtr1w.airplanes.items():
            loc = LocationGlobalRelative(airplaneData["latitude"], airplaneData["longitude"])
            dist = get_distance_metres(vehicle_pos, loc)

            if(dist < 10): 
                print("Game Over")
                continue
            if(dist < MAX_RADIUS and not havePriority(vehicle_pos, loc, vehicle.heading)):
                print("collision detected")
                avoidCollision(vehicle,loc, airplaneData["dir"], np.linalg.norm(vehicle.velocity), airplaneData["h_velocity"], ICAO)
                # print("Closest calculated distance: ", dist)

        readtr1w.checkingAirplanes = False


def main():
    altitude = 4
    radius = 20
    # Parse connection argument
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    parser.add_argument("-m", "--mission", help="mission_to_play")
    parser.add_argument("-alt", "--altitude", help="default altitude")
    parser.add_argument("-r", "--radius", help="default radius")
    args = parser.parse_args()
    connection_string=args.connect
    currentMission=args.mission

    if(args.radius):
        radius = args.radius
    if(args.altitude):
        altitude = args.altitude
    vehicle = connectPixhawk(connection_string)
    vehicle.groundspeed = 3


    try:
        run_event = threading.Event()
        run_event.set()
        tr1wGatherThread = threading.Thread(target = readtr1w.readTransponder, args = (run_event,'/dev/ttyUSB0'))
        tr1wGatherThread.start()
        tr1wDataThread = threading.Thread(target = checkAirplanesDistance, args = (run_event,vehicle))
        tr1wDataThread.start()
        if not currentMission:
            takeoff_land(vehicle, altitude)
        elif currentMission == "simple":
            simple_go_and_move_back(vehicle, altitude, radius)
        elif currentMission == "long":
            readtr1w.addFakeAirPlane(vehicle.location.global_relative_frame, radius)
            ArdupilotMission(vehicle, radius, altitude)
    except KeyboardInterrupt:
        run_event.clear()
        tr1wGatherThread.join()
        tr1wDataThread.join()
        print("Thread closed")
        exit
    except:
        run_event.clear()
        tr1wGatherThread.join()
        tr1wDataThread.join()
        print("Thread closed")
        raise
    #simple_goto(altitude, radius)
    #RTL wznosi sie na 15 m w symulatorze
    #adds_square_mission(vehicle.location.global_relative_frame,radius, altitude)
    #takeoff_land(altitude)
    #testAutoMode()
    run_event.clear()
    tr1wGatherThread.join()
    tr1wDataThread.join()
    print("Thread closed")
    # Close vehicle object before exiting script
    vehicle.close()
    # time.sleep(1)

if __name__ == "__main__":
   main()
