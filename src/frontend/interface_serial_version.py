from vpython import *
import numpy as np
import math
import serial

# Firebase setup
try:
    sHandler = serial.Serial('com4',115200)
except Exception as e:
    print(e)

# Scene settings
scene.title = "Quadcopter Simulation"
scene.width = 1080
scene.height = 720

body = box(pos=vector(0, 0, 0), size=vector(0.8, 0.2, 0.8), color=color.blue)

# Simulation axis 3D axis
z_axis = arrow(pos = vector(0.051 ,2 ,0), axis=vector(1, 0, 0), opacity=0.5, shininess=0.2, color=color.green)
x_axis = arrow(pos = vector(0 ,1.95 ,0), axis=vector(0, 1, 0), opacity=0.5, shininess=0.2, color=color.red)
y_axis = arrow(pos = vector(0 ,2 ,0.051), axis=vector(0, 0, 1), opacity=0.5, shininess=0.2, color=color.blue)

# Arm parameters
arm_length = 1.2
arm_thickness = 0.05

corner_offset = body.size.x / 2
diagonal_offset = corner_offset / np.sqrt(2) # Offset for 45-degree diagonal placement

# Arms positioned diagonally
arm1 = box(pos=vector(diagonal_offset, 0, diagonal_offset), size=vector(arm_length, arm_thickness, arm_thickness),
           axis=vector(1, 0, 1), up=vector(0, 1, 0), color=color.red)
arm2 = box(pos=vector(-diagonal_offset, 0, diagonal_offset), size=vector(arm_length, arm_thickness, arm_thickness),
           axis=vector(-1, 0, 1), up=vector(0, 1, 0), color=color.green)
arm3 = box(pos=vector(-diagonal_offset, 0, -diagonal_offset), size=vector(arm_length, arm_thickness, arm_thickness),
           axis=vector(-1, 0, -1), up=vector(0, 1, 0), color=color.red)
arm4 = box(pos=vector(diagonal_offset, 0, -diagonal_offset), size=vector(arm_length, arm_thickness, arm_thickness),
           axis=vector(1, 0, -1), up=vector(0, 1, 0), color=color.green)

# Propeller parameters
propeller_radius = 0.3
propeller_thickness = 0.05

# Create propellers at the ends of the arms
prop1 = cylinder(pos=arm1.pos + arm1.axis / 2, axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.red)
prop2 = cylinder(pos=arm2.pos + arm2.axis / 2, axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.green)
prop3 = cylinder(pos=arm3.pos + arm3.axis / 2, axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.red)
prop4 = cylinder(pos=arm4.pos + arm4.axis / 2, axis=vector(0, 0.1, 0), radius=propeller_radius, color=color.green)

# Update Y position of the propellers
prop1.pos.y = -0.1 
prop2.pos.y = -0.1
prop3.pos.y = -0.1
prop4.pos.y = -0.1

# Group all components into a single quadcopter object
quadcopter = compound([body, arm1, arm2, arm3, arm4, prop1, prop2, prop3, prop4])

scene.camera.pos = vector(3, 2, 5)
scene.camera.axis = vector(-3, -2, -5)

def cvtQuat2YPR(q0: float, q1: float, q2: float, q3: float):
    norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    if norm == 0:
        print("Quaternion normalization failed: Zero norm.")
        return 0, 0, 0  # Default values for invalid quaternion
    q0, q1, q2, q3 = q0 / norm, q1 / norm, q2 / norm, q3 / norm

    # Compute roll, pitch, yaw with clamping for pitch
    roll = math.atan2(2 * ((q2 * q3) + (q0 * q1)), q0**2 - q1**2 - q2**2 + q3**2)
    pitch = math.asin(max(-1, min(1, 2 * ((q1 * q3) - (q0 * q2)))))  # Clamp to [-1, 1]
    yaw = -math.atan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2)

    return yaw, pitch, roll

def main_loop():
    while True:
        ## FETCH DATA FROM THE SERIAL CONNECTION TO ARDUINO BOARD
        while (sHandler.inWaiting()==0):
            pass
        dataPacket=sHandler.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        q0=float(splitPacket[0])
        q1=float(splitPacket[1])
        q2=float(splitPacket[2])
        q3=float(splitPacket[3])
        
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

        rate(50)  # Animation rate

# Start the animation
main_loop()

