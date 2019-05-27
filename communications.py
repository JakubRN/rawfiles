import dronekit
from dronekit import connect
from dronekit import Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math


home_position_set = False
MAV_MODE_AUTO   = 4
MAV_MODE_STABILIZED = 16
def connectPixhawk():
    connection_string       = '127.0.0.1:14540'
    serial_connection 		= '/dev/ttyTHS1'
    baud_rate 				= 921600

    # https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
    
    # Parse connection argument
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    parser.add_argument("-b", "--baud", help="baud rate")
    args = parser.parse_args()
    connection_string=args.connect
    # Connect to the Vehicle
    if not connection_string:
        print( "Connecting UART")
        vehicle = connect(serial_connection, wait_ready=True, baud=baud_rate)
        print( "connected")
        init(vehicle)     
        return vehicle
    else:
        print( "Connecting SITL")
        vehicle = connect(connection_string, wait_ready=True)
        print( "connected")
        init(vehicle)     
        return vehicle
    
def init(vehicle):
    udp_conn = dronekit.mavlink.MAVConnection('udpin:0.0.0.0:15667', source_system=1)
    vehicle._handler.pipe(udp_conn)
    udp_conn.master.mav.srcComponent = 1
    udp_conn.start
    print('udp launched')
    #Create a message listener for home position fix
    @vehicle.on_message('HOME_POSITION')
    def listener(self, name, home_position):
        global home_position_set
        home_position_set = True

    while not home_position_set:
        print( "Waiting for home position...")
        time.sleep(1)

    # Display basic vehicle state
    print( " Type: %s" % vehicle._vehicle_type)
    print( " Armed: %s" % vehicle.armed)
    print( " System status: %s" % vehicle.system_status.state)
    print( " GPS: %s" % vehicle.gps_0)
    print( " Alt: %s" % vehicle.location.global_relative_frame.alt)
    print("Battery: ", vehicle.battery.level, ", voltage: ", vehicle.battery.voltage)




def PX4setMode(vehicle, mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)
                                               
def PX4RTL(vehicle):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                                               0,
                                               0, 0, 0, 0, 0, 0)


