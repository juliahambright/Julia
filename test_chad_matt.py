

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
    print("Arming motors--2 sec wait")

    time.sleep(2)

    # wating for vehicle to be armable
    #while not vehicle.is_armable:
        #time.sleep(1)

    # once armable vehile set to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    print("--Bot Mode-- %s" %vehicle.mode)

    # write True to arm the vehicle
    vehicle.armed = True

    # if not armed wait
    while not vehicle.armed: time.sleep(1)

    print(vehicle.mode)
    print("Vehicle Armed: %s"%vehicle.armed)

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

arm_and_takeoff(0)

#set default speed
vehicle.groundspeed = 1

print("^^Setting Launch Location to Current Coordinates^^ -- wait 3 secs \n")
vehicle.home_location = vehicle.location.global_frame

print("Home Current Location %s" % vehicle.location.global_frame)
time.sleep(3)


 # starting mission to destination 
## edit to right lat and long variables: print('Going to Coordinates: ' + str(lat) + ", " + str(long))

#end dronekit setup
# functions I want vehicle.simple_goto(wp2),how drone understands waypoint= LocationGlobalRelative(newLat, newLong, 10)

next_loc = vehicle.location.global_frame
print("Next Location: %s" % next_loc)
j = 0

#waypoint tree that saves the lists of waypoints detected in each locations by the lidar
wptree = {}

final_loc =  LocationGlobalRelative(finalLat, finalLong, 10)
print("Final Location: %s" % final_loc)

x = time.time()
while distance_to_current_waypoint(final_loc)>2:
    elapsed = time.time() - x
    if elapsed > 20:
        break
    print("Distance to Final WP: %s" % distance_to_current_waypoint(final_loc))
    if distance_to_current_waypoint(next_loc)<2:
        elapsed = time.time() - x
        if elapsed > 20:
            break
        print("--Bot Mode-- %s" %vehicle.mode)
        while not vehicle.mode == "HOLD":
            vehicle.mode = VehicleMode("HOLD")
            time.sleep(1)
        print("--Bot Mode while getting next WP-- %s" %vehicle.mode)

        print("Distance to Next WP: %s" % distance_to_current_waypoint(next_loc))
        print("<<<<<<<---------- Finding Next WP -------->>>>>>>")
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
            next_loc = LocationGlobalRelative(nextLat, nextLong, 0)
            print("Next Waypoint: %s" % next_loc)
            j=j+1

            print("WP Tree is Below")
            print(wptree)
            print("\n")


            print("--Bot Mode-- %s" %vehicle.mode)
            while not vehicle.mode == "GUIDED":
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)
            print("--Bot Mode before going to next WP-- %s" %vehicle.mode)

            print("Going to next Waypoint: %s" % next_loc)
            vehicle.simple_goto(next_loc)
    else:
        time.sleep(1)
    

print("--Bot Mode-- %s" %vehicle.mode)
while not vehicle.mode == "HOLD":
    vehicle.mode = VehicleMode("HOLD")
    time.sleep(1)
print("--Bot Mode before Close-- %s" %vehicle.mode)

vehicle.armed = False
    # if armed wait
    while vehicle.armed: 
        time.sleep(1)
        vehicle.armed = False

print("Vehicle Armed: %s" % vehicle.armed)
vehicle.close()
print("Vehicle Closed -- Mission Over")
time.sleep(5)


#result = subprocess.run(["./ultra_simple"," /dev/ttyUSB0"], stdout=PIPE, stderr=PIPE)
#print("result from c++: ",result.stdout.decode('utf-8'))


