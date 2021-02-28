

from subprocess import PIPE
import subprocess
import sys
import re
import os


from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
import math

# getting arguments of connection string, input lat, and input lon from command line call
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
parser.add_argument('--destinationLat')
parser.add_argument('--destinationLong')
args = parser.parse_args()

# getting connection string from input arguments
#connection_string = args.connect
connection_string = "/dev/ttyS0"
# getting and setting target lat and lon from input arguments
finalLat = float(args.destinationLat)
finalLon = float(args.destinationLong)

#Connect to the vehicle
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, wait_ready=True)

#----------------------------dronekit functions----------------------------
def arm_and_takeoff():
    print("Arming motors")

    # wating for vehicle to be armable
    while not vehicle.is_armable:
        time.sleep(1)

    # once armable vehile set to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    print("--Bot in Guided Mode--")

    # write True to arm the vehicle
    vehicle.armed = True

    # if not armed wait
    while not vehicle.armed: time.sleep(1)

    print("Takeoff")

def get_lat(i):
    print(">>Getting new lat for obstacle")
    currentLat = vehicle.location.global_frame.lat # current Lat from bot
    if i % 2 == 0:
        newLat = currentLat - 0.0002 # creating new lat
    else:
        newLat = currentLat + 0.0001 # creating new lat
    #print("New Latitude: ", newLat)
    return newLat


# this function will represent getting a new longitude for the bot to go to when obstacle
def get_long(i):
    print(">>Getting new long for obstacle")
    currentLon = vehicle.location.global_frame.lon # current lon from bot
    #print("i = ", i)
    if i % 2 == 0:
        newLong = currentLon + 0.00002 # creating new lon
    else:
        newLong = currentLon - 0.0002 # creating new lon
    #print("New Longitude: ", newLong)
    return newLong


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# Gets distance in metres to the current waypoint
def distance_to_current_waypoint(wp):

    distancetopoint = get_distance_metres(vehicle.location.global_frame, wp)
    return distancetopoint

def get_lat2_lon2(currentLat, currentLon, dist, theta):
    R = 6378.1 #Radius of the Earth

    #lat2  52.20444 - the lat result I'm hoping for
    #lon2  0.36056 - the long result I'm hoping for.

    currentLat = float(currentLat)
    currentLon = float(currentLon)
    dist = float(dist)
    theta = float(theta)

    lat1 = math.radians(currentLat) #Current lat point converted to radians
    lon1 = math.radians(currentLon) #Current long point converted to radians

    lat2 = math.asin( math.sin(lat1)*math.cos(dist/R) + math.cos(lat1)*math.sin(dist/R)*math.cos(theta))

    lon2 = lon1 + math.atan2(math.sin(theta)*math.sin(dist/R)*math.cos(lat1),math.cos(dist/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    list = [lat2,lon2]
    return list


#----------------------------Main Program----------------------------
#dronekit setup

arm_and_takeoff()

#set default speed
vehicle.groundspeed = 3
gndSpeed = 3

print("^^Setting Launch Location to Current Coordinates^^\n")
vehicle.home_location = vehicle.location.global_frame

 # starting mission to destination 
## edit to right lat and long variables: print('Going to Coordinates: ' + str(lat) + ", " + str(long))

#end dronekit setup
# functions I want vehicle.simple_goto(wp2),how drone understands waypoint= LocationGlobalRelative(newLat, newLong, 10)

next_loc = vehicle.location.global_frame

j = 0

#waypoint tree that saves the lists of waypoints detected in each locations by the lidar
wptree = {}

final_loc =  LocationGlobalRelative(finalLat, finalLong, 10)

if distance_to_current_waypoint(final_loc)>2:

    while(distance_to_current_waypoint(next_loc)<2):
        proc = subprocess.Popen(
            ["./ultra_simple"," /dev/ttyUSB0"],
            stderr=subprocess.STDOUT,  # Merge stdout and stderr
            stdout=subprocess.PIPE,
            shell=True)


        stdoutdata, stderrdata = proc.communicate()
        print("stdoutdata: ",stdoutdata)
        print("stderrdata: ",stderrdata)

        match = re.findall("theta: (?P<theta>[\d.]+(?= )) , distance: (?P<distance>[\d.]+(?= )) , Waypoint (?P<waypoint>[\d.]+(?= ))", stdoutdata.decode('utf-8'))

        #if waypoint is found
        if len(match)>0:

            #add new location's waypoints to waypoint tree
            wptree["loc"+str(j)] = match
            theta = match[0][0]
            dist = match[0][1]
            print(theta)
            print(dist)
            currentLat = vehicle.location.global_frame.lat
            currentLon = vehicle.location.global_frame.lon

            #converts the distance and theta into a lattitude and longitude point relative to the current location
            list = get_lat2_lon2(currentLat, currentLon, dist, theta)
            nextLat = list[0]
            nextLon = list[1]

            #sets next_loc to the next waypoint in a way that the pixhawks can understand
            next_loc = LocationGlobalRelative(nextLat, nextLong, 10)
            j=j+1
            print(wptree)

            vehicle.simple_goto(next_loc)
    
else:
    print("Arrived at Destination")






#result = subprocess.run(["./ultra_simple"," /dev/ttyUSB0"], stdout=PIPE, stderr=PIPE)
#print("result from c++: ",result.stdout.decode('utf-8'))


