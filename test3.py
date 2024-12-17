from vpython import *
import numpy as np
import math
import firebase_admin
from firebase_admin import credentials, db
import serial
import threading

# Firebase setup
cred = credentials.Certificate('key.json')
try:
    sHandler = serial.Serial('com4',115200)
except Exception as e:
    print(e)

firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/'
})
ref = db.reference('/SensorData')

# Scene settings
scene.title = "Quadcopter Simulation"
scene.width = 1080
scene.height = 720

body = box(pos=vector(0, 0, 0), size=vector(0.8, 0.2, 0.8), color=color.blue)

# Simulation axis 3D axis
z_axis = arrow(pos = vector(0.051 ,2 ,0), axis=vector(1, 0, 0), opacity=0.5, shininess=0.2, color=color.green)
x_axis = arrow(pos = vector(0 ,1.95 ,0), axis=vector(0, 1, 0), opacity=0.5, shininess=0.2, color=color.red)
y_axis = arrow(pos = vector(0 ,2 ,0.051), axis=vector(0, 0, 1), opacity=0.5, shininess=0.2, color=color.blue)

# Arms
arm_length = 2
arm_thickness = 0.1
arm1 = box(pos=vector(0, 0, 0), size=vector(arm_length, arm_thickness, arm_thickness), color=color.red)
arm2 = box(pos=vector(0, 0, 0), size=vector(arm_thickness, arm_thickness, arm_length), color=color.green)

# Propellers
propeller_radius = 0.3
propeller_thickness = 0.05

# Create propellers at the ends of the arms
prop1 = cylinder(pos=vector(arm_length/2, 0, 0), axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.red)
prop2 = cylinder(pos=vector(-arm_length/2, 0, 0), axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.red)
prop3 = cylinder(pos=vector(0, 0, arm_length/2), axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.green)
prop4 = cylinder(pos=vector(0, 0, -arm_length/2), axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.green)

# Group all components into a single quadcopter object
quadcopter = compound([body, arm1, arm2, prop1, prop2, prop3, prop4])

scene.camera.pos = vector(3, 2, 5)
scene.camera.axis = vector(-3, -2, -5)

def fetch_db_data():
    try:
        data = ref.get()  # Get data from Firebase
        if data:
            q0 = data.get('QuatW', 0)
            q1 = data.get('QuatX', 0)
            q2 = data.get('QuatY', 0)
            q3 = data.get('QuatZ', 0)
            temp = data.get('Temp', 0)
            return q0, q1, q2, q3, temp
        print("No data found in Firebase.")
    except Exception as e:
        print(f"Error fetching data: {e}")
    return 0, 0, 0, 0, 0

def cvtQuat2YPR(q0: float, q1: float, q2: float, q3: float):
    norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    if norm == 0:
        print("Quaternion normalization failed: Zero norm.")
        return 0, 0, 0  # Default values for invalid quaternion
    q0, q1, q2, q3 = q0 / norm, q1 / norm, q2 / norm, q3 / norm

    # Compute roll, pitch, yaw with clamping for pitch
    roll = math.atan2(2 * ((q2 * q3) + (q0 * q1)), q0**2 - q1**2 - q2**2 + q3**2)
    pitch = math.asin(max(-1, min(1, 2 * ((q1 * q3) - (q0 * q2)))))  # Clamp to [-1, 1]
    yaw = math.atan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2)

    return yaw, pitch, roll

def main_loop():
    while True:
        # while (sHandler.inWaiting()==0):
        #     pass
        # dataPacket=sHandler.readline()
        # dataPacket=str(dataPacket,'utf-8')
        # splitPacket=dataPacket.split(",")
        # q0=float(splitPacket[0])
        # q1=float(splitPacket[1])
        # q2=float(splitPacket[2])
        # q3=float(splitPacket[3])
        
        q0, q1, q2, q3, temp = fetch_db_data()
        # print(temp)
        try: 
            yaw, pitch, roll = cvtQuat2YPR(q0, q1, q2, q3)
        except Exception as e:
            print(e)
            pass
        
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)
 
        quadcopter.axis=k
        quadcopter.up=vrot

        # print(yaw, pitch, roll)
        # print(q0, q1, q2, q3)
        
        rate(50)  # Animation rate

# Start the animation
main_loop()

