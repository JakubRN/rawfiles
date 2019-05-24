import serial, sys
import io, time
from dronekit import LocationGlobalRelative
from helperFunctions import get_location_offset_meters, get_distance_metres
airplanes = {}
checkingAirplanes = False

def addFakeAirPlane(vehicle_pos, distance):
    wp = get_location_offset_meters(vehicle_pos, 0, distance, 0)
    aircraftData = {
    "flags":0x02,
    "latitude":wp.lat,
    "longitude":wp.lon,
    "altitude":wp.alt,
    "v_velocity":0.0,
    "h_velocity":3.0,
    "dir":270
    }
    airplanes["0"] = aircraftData 
    wp = get_location_offset_meters(vehicle_pos, 0, -distance, 0)
    aircraftData = {
    "flags":0x02,
    "latitude":wp.lat,
    "longitude":wp.lon,
    "altitude":wp.alt,
    "v_velocity":0.0,
    "h_velocity":3.0,
    "dir":90
    }
    airplanes["1"] = aircraftData
    # print("fake airplanes added: ", airplanes)



def processTo(data, fnctn):
    if(data != ''):	
        return fnctn(data)
    else:
        return fnctn("0");

def readTransponder(dev):
    try:
        ser = serial.Serial(dev, 115200, timeout=1)
        sio = io.TextIOWrapper(io.BufferedReader(ser), newline="\r\n")
        ser.reset_input_buffer()
    except:
        print("Couldn't open tr1w device, anti-collision is not working")
        return
    try:
        while True:
            line = sio.readline()
            if(len(line) == 0): continue
            data = line[3:].strip('\r\n').split(',')
            if(line[1] == 'S'):
                print("Status: CPU load: ", data[0])	
            elif(line[1] == 'A'):
                ICAO = data[0]	
                FLAGS = data[1]
                GPS_LAT = processTo(data[4], float)
                GPS_LON = processTo(data[5], float)
                ALTITUDE = processTo(data[6], float)
                horizontalVelocity = processTo(data[8], float)*1.852
                verticalVelocity = processTo(data[9], float) * 0.00508
                directionAzimuth = processTo(data[7], int)
                aircraftData = {
                    "flags":FLAGS,
                    "latitude":GPS_LAT,
                    "longitude":GPS_LON,
                    "altitude":ALTITUDE,
                    "v_velocity":verticalVelocity,
                    "h_velocity":horizontalVelocity,
                    "dir":directionAzimuth
                }
                while(checkingAirplanes):
                    time.sleep(0.05)
                airplanes[ICAO] = aircraftData 
                #print(aircraftData)
                #print("Aircraft, ICAO: ", data[0], ", lat: ", data[4], ", lon: ", data[5], ", alt: ", data[6], ", track: ", data[7], ", VELH: ", float(data[8]) * 1.852)
            else:
                print("unknown: ", line)
    except:
        print("error: ", sys.exc_info()[0], " while reading tr1w, anticollision stopped working") 
            

