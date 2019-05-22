
import dronekit
from dronekit import connect
from dronekit import Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
from communications import connectPixhawk, PX4setMode



def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)





################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True



################################################################################################
# Start mission example
################################################################################################

vehicle = connectPixhawk()

# Load commands
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame

# takeoff
wp = get_location_offset_meters(home, 0, 0, 3);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
wp = get_location_offset_meters(home, 0, 0, 3);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

cmds.upload()
time.sleep(1)
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)


vehicle.armed = True

# monitor mission execution
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print( "Moving to waypoint %s" % display_seq)
        nextwaypoint = vehicle.commands.next
    time.sleep(1)

# wait for the vehicle to land
while vehicle.commands.next > 0:
    time.sleep(1)


# Disarm vehicle
vehicle.armed = False
time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)
