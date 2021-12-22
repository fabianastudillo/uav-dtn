#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Data collection in two waypoints
"""

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera

# Marker parameters
# Defines the dictionary for the type of marker used in this case 6X6 to 250
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
arucomeasure = 4 # known marker measure
foclenght = 1225 # measured in pixels of the side or bottom distance of the marker in the captured frame


# GO PRO Camera parameters
hres = 618
vres = 378
# Camera calibration parameters
mtx=np.array([[ 470.79433195, 0, 461.270146  ], [0, 467.90739423, 267.78259298], [0, 0, 1]])
dista=np.array([[-0.26629651,  0.07608818 , 0.00428972,  0.0117036,  -0.00256195]])
cap = cv2.VideoCapture(0)
time.sleep(0.1)

# General parameters
cal = hres/vres #relación de aspecto de cámara

# Functions

def searchmarker():
    i=0
    marker=False
    while (True):
        # Camara GO PRO
        ret, image =cap.read()
        image= cv2.rotate(image, cv2.ROTATE_180)
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dista,(w,h),1,(w,h))
        image = cv2.undistort(image, mtx, dista, None, newcameramtx)
        x,y,w,h = roi
        image = image[y:y+h, x:x+w]

        corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        numcorn=len(corners)

        if numcorn > 0:
            marker=True
            break
        else:
            marker=False

        #print('marker=',marker)
        i=i+1
        if i==15:
            break
    return marker

def yawactual():
    refyaw=math.degrees(vehicle.attitude.yaw)
    # Corrects yaw angle with reference to north
    if refyaw>0:
        actualyaw = refyaw
    else:
        actualyaw = 360 + refyaw
    return actualyaw

def getyawcorrection(refyaw):
    i=0

    while (True):
        # GO PRO Camera
        ret, image =cap.read()
        image= cv2.rotate(image, cv2.ROTATE_180)
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dista,(w,h),1,(w,h))
        image = cv2.undistort(image, mtx, dista, None, newcameramtx)
        x,y,w,h = roi
        image = image[y:y+h, x:x+w]

        corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        numcorn=len(corners)

        if numcorn > 0:
            print('marker')
            p1x=corners[0][0][0][0]
            p2x=corners[0][0][1][0]
            p3x=corners[0][0][2][0]
            p4x=corners[0][0][3][0]
            # Posicion en Y de cada vertice del marcador
            p1y=corners[-1][0][0][1]
            p2y=corners[-1][0][1][1]
            p3y=corners[-1][0][2][1]
            p4y=corners[-1][0][3][1]
            # Calculo de distancia al origen para cada punto
            d1 = math.sqrt((p1x)**2+(p1y)**2)
            d2 = math.sqrt((p2x)**2+(p2y)**2)
            d3 = math.sqrt((p3x)**2+(p3y)**2)
            d4 = math.sqrt((p4x)**2+(p4y)**2)

            distl = []
            distl.append(d1)
            distl.append(d2)
            distl.append(d3)
            distl.append(d4)
            # Calculate the minimum distance and determine the position of the point closest to the origin
            minposition = distl.index(min(distl))+1
            # Slope calculation between P1 and P4
            m=(p1y-p4y)/(p1x-p4x)
            # Slope angle calculation
            anglem= ((math.atan(m))*180/math.pi)
            # Calculate the angle correction to obtain it with respect to the Y axis (+)
            if anglem<0:
                if minposition == 1 or minposition == 4:
                    angle = 90+anglem
                    print('Q1')
                if minposition == 2 or minposition == 3:
                    angle = 270+anglem
                    print('Q3')
            else:
                if minposition == 1 or minposition == 2:
                    angle = 270+anglem
                    print('Q2')

                if minposition == 3 or minposition == 4:
                    angle = 90+anglem
                    print('Q4')
            print('rotacion = ', angle)
            newyaw = refyaw+angle
            if newyaw>360:
                newyawf=newyaw-360
            else:
                newyawf=newyaw
            print('newyawf = ',newyawf)
            break
        else:
            print('Error. No hay marcador')
            newyawf=refyaw
        i=i+1
        if i==20:
            break
    return newyawf

def getcentervelocvector():
    i=0
    velcent=0.1
    while (True):
        # GO PRO Camera
        ret, image =cap.read()
        image= cv2.rotate(image, cv2.ROTATE_180)
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dista,(w,h),1,(w,h))
        image = cv2.undistort(image, mtx, dista, None, newcameramtx)
        x,y,w,h = roi
        image = image[y:y+h, x:x+w]
        corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        numcorn=len(corners)

        if numcorn > 0:
            # X position of each vertex of the marker
            p2x=corners[0][0][1][0]
            p4x=corners[0][0][3][0]
            # Y position of each vertex of the marker
            p2y=corners[-1][0][1][1]
            p4y=corners[-1][0][3][1]
            # Marker center point calculation
            centroX=((p4x)+(p2x))/2
            centroY=((p4y)+(p2y))/2
            if centroX <= hres/2:
                if centroY <= vres/2:
                    print('Q2')
                    vx=velcent/cal
                    vy=-velcent
                else:
                    print('Q3')
                    vx=-velcent/cal
                    vy=-velcent
            else:
                if centroY <= vres/2:
                    print('Q1')
                    vx=velcent/cal
                    vy=velcent
                else:
                    print('Q4')
                    vx=-velcent/cal
                    vy=velcent
        else:
            vertices=[[[[0,0],[0,0],[0,0],[0,0]]]]
            vx=0
            vy=0
            centroX=0
            centroY=0
        i=i+1
        if i==7:
            break
    return vx, vy, centroX, centroY

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-assembly check")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for the vehicle to start...")
        time.sleep(1)

    print("Arming engines")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting armed...")
        time.sleep(1)

    print("¡Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.75:
            print("Target altitude reached")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    """
    send_global_velocity(0,0,0,1)
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    if heading >= 180:
        direction = 1
    else:
        direction = -1
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        print("  Comando enviado...")
        time.sleep(1)

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='DTN Data collection using UAV')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified (Dronekit-SITL do not run on RPi)
if not connection_string:
    print('Starting DRONEKIT-SITL...')
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to the vehicle in: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for HOME location...")
# Home
print("\n Location of HOME: %s" % vehicle.home_location)

home = LocationGlobalRelative(vehicle.home_location.lat,vehicle.home_location.lon,vehicle.home_location.alt)

flightalt = 2.3

homewp = LocationGlobalRelative(vehicle.home_location.lat,vehicle.home_location.lon, flightalt)
#SITL BalzayTesis
WP1 = LocationGlobalRelative(-2.8924465, -79.038765, flightalt)
WP2 = LocationGlobalRelative(-2.8923802, -79.038745, flightalt)

#WPlist = [WP1, WP2, WP3]
WPlist = [WP1, WP2]
#WPlist = [WP1]

#Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
ADELANTE = 0.5
ATRAS = -0.5

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
DERECHA = 0.5
IZQUIERDA = -0.5

print("Set default/target airspeed to 0.5")
vehicle.airspeed = 1.2

# Altura minima para aterrizar
altland=1
# Velocidad de descenso
veldesc=0.2
vx=0
vy=0
vx_ant=0
vy_ant=0

nodoID = ['ipn:300.2', 'ipn:400.2']
valnodo = 0

for wp in WPlist:
    print('Addressing to waypoint',wp)
    arm_and_takeoff(flightalt)
    vehicle.simple_goto(wp)
    while True:
        #Estimate distance betweewn current location and wp1
        ds=get_distance_metres(vehicle.location.global_frame, wp)
        print(" Distance to waypoint: %s" % ds)
        if ds<=1: #Continue with the program
            print("\n ¡Waypoint reached!")
            break;
        time.sleep(1)
    print('Localization and detection started...')
    marker=searchmarker()
    time.sleep(1)
    while marker == 0:
        print('There is no marker')
        dist = [2, 1, 2, 1]
        velx = [DERECHA,ATRAS,IZQUIERDA,ADELANTE]
        vely = [0, 0]
        distance = 0
        i = 0
        pref = vehicle.location.global_frame
        for ds in dist:
            print('Locating marker...')
            velacx = velx[i]
            velacy = vely[i]
            print('velacx = ',velacx)
            print('velacy = ',velacy)
            while distance < ds:
                send_ned_velocity(velacx,velacy,0,1)
                #Estimate distance betweewn current location and pref
                distance=get_distance_metres(vehicle.location.global_frame, pref)
                print('distance = ', distance)
                time.sleep(1)
            marker=searchmarker()
            time.sleep(1)
            if marker == True:
                print('¡Localized marker!')
                break
            else:
                print('There is no marker')
            print('Zero')
            pref = vehicle.location.global_frame
            i = i + 1
            distance = 0
    print('Orientation started...')
    yawref= yawactual()
    print('yawref=',yawref)
    newyaw=getyawcorrection(yawref)
    #Porcentaje de error permitido en el angulo de orientacion
    yawtol=0.1
    orientcorrect=1
    while orientcorrect==1:
        print('Correcting orientation')
        condition_yaw(int(newyaw))
        time.sleep(1)
        actualyaw=yawactual()
        if (actualyaw>(newyaw-(yawtol*newyaw))) and (actualyaw<(newyaw+(yawtol*newyaw))):
            print('Corrected orientation')
            orientcorrect=0
        else:
            orientcorrect=1

    yawfinal= yawactual()
    print('yawfinal=',yawfinal)
    print('Centering and descent started...')
    time.sleep(2)
    while vehicle.location.global_relative_frame.alt>=altland:
        vx, vy, cx, cy = getcentervelocvector()
        print('cx=',cx)
        print('cy=',cy)
        print('vx=',vx)
        print('vy=',vy)
        if (vx==0) and (vy==0) and (cx==0) and (cy==0):
            print('Marker lost. Reversing last move')
            time.sleep(1)
            send_ned_velocity(-vx_ant/2,-vy_ant/2,0,1)
        if (cy<=((vres/2)+90)) and (cy>=((vres/2)-90)):
            if (cx<=((hres/2)+90)) and (cx>=((hres/2)-90)):
                print('Comes down')
                time.sleep(1)
                send_ned_velocity(0,0,veldesc,1)
                print('descend 0.2m')
                #send_ned_velocity(0,0,0,1)
                time.sleep(1)
            else:
                print('Horizontally')
                send_ned_velocity(0,vy,0,1)
                #time.sleep(1)
        else:
            print('Vertically')
            send_ned_velocity(vx,0,0,1)
            #time.sleep(1)
        time.sleep(1)
        print('Altitude =', vehicle.location.global_relative_frame.alt)
        vx_ant=vx
        vy_ant=vy
    print("Descending on platform")
    time.sleep(2)
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        print(" Waiting unarmed...")
        time.sleep(1)
    print(" Disassembled vehicle")
    print('Pairing and transmission started...')
    time.sleep(3)
    print('Pairing and transmission started...')
    time.sleep(3)
    print('\n')
    print('Node:', nodoID[valnodo])
    print('Start flag transmission')
    print('\n')
    print('Retransmission started...')
    print('Waiting for packages...')
    time.sleep(10)
    print('Data: b')
    print('Transmission from node completed. Going to the next node...')
    valnodo = valnodo + 1

print('Addressing to HOME',home)
arm_and_takeoff(flightalt)
vehicle.simple_goto(homewp)
while True:
    #Estimate distance betweewn current location and wp1
    ds=get_distance_metres(vehicle.location.global_frame, home)
    print(" Distance to HOME: %s" % ds)
    if ds<=1: #Continue with the program
        print("\n ¡HOME reached!")
        break;
    time.sleep(1)


print("Landing in HOME")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Waiting unarmed...")
    time.sleep(1)
print(" Disassembled vehicle")

# Close vehicle object before exiting script
print(" Closing the vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
