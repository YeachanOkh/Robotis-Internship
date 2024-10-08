#!/usr/bin/env python3
import math
import Motorcontrol as motor
import movementcalc as calculation
import numpy as np
import time
import cv2
import rasptoarduino as hand
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# Define motor IDs
BASE_ID = 1
BICEP_ID = 2
FOREARM_ID = 3
WRIST_ID = 4
CLAW_ID = 0

# Define port number for Raspberry Pi
PORT_NUM = '/dev/ttyUSB0'  # for rpi

# Define move mode and address for present position
MOVEARM_MODE = 1
ADDR_PRESENT_POSITION = 132

# List of all motor IDs
ALL_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
MOVE_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]

# Initialize motor port
motor.portInitialization(PORT_NUM, ALL_IDs)
# ctrl.portInitialization(PORT_NUM, ALL_IDs)

# Calculate the angle for the max length reaching out in the x position
max_length_angle = calculation.angle_Calc([375, 0, 73], 0)

# Check movement of motors
def checkMovement(ids):
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in ids:
            firstPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(0.1)
            secondPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            if abs(firstPosition - secondPosition) < 5:
                motorStatus[id] = 1
        if motorStatus == finished:
            print("finished")
            break

# Define various hand movements
def Hello():
    start_time = time.time()
    print("Hello")
    motor.dxlSetVelo([45, 55, 30, 30, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([180, 50, 265], [3, 2, 4])
    time.sleep(0.1)
    hand.handmotor("hello")
    time.sleep(0.2)
    motor.simMotorRun([260], [1])
    time.sleep(0.1)
    motor.simMotorRun([280], [1])
    time.sleep(0.1)
    motor.simMotorRun([260], [1])
    time.sleep(0.1)
    motor.simMotorRun([280], [1])
    time.sleep(0.1)
    motor.simMotorRun([130], [0])

def Yes():
    start_time = time.time()
    print("Yes")
    hand.handmotor("yes")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220, 85, 265], [3, 2, 4])
    time.sleep(0.2)
    motor.simMotorRun([180], [4])
    time.sleep(0.2)
    motor.simMotorRun([265], [4])
    time.sleep(0.2)
    motor.simMotorRun([180], [4])
    time.sleep(0.2)
    motor.simMotorRun([265], [4])


def Fistbump():
    start_time = time.time()
    print("fistbump")
    hand.handmotor("fistbump")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([200, 80, 180], [3, 2, 4])
    motor.dxlSetVelo([55, 55, 55], [2, 3, 4])
    time.sleep(0.1)
    motor.simMotorRun([60, 180, 172], [2, 3, 4])
    time.sleep(0.3)
    motor.simMotorRun([200, 80, 180], [3, 2, 4])
    
    
def Highfive():
    start_time = time.time()
    print("Highfive")
    hand.handmotor("highfive")
    motor.dxlSetVelo([30, 30, 30, 30, 65], [0, 1, 2, 3, 4])
    motor.simMotorRun([265, 160, 40], [4, 3, 2])
    motor.dxlSetVelo([30, 30, 50, 50, 50], [0, 1, 2, 3, 4])
    motor.simMotorRun([20, 135, 245], [2, 3, 4])
    time.sleep(0.2)
    motor.simMotorRun([40, 160, 265], [2, 3, 4])

def Handshake():
    start_time = time.time()
    print("Handshake")
    hand.handmotor("handshake")
    motor.dxlSetVelo([40, 40, 40, 40, 40], [0, 1, 2, 3, 4])
    motor.simMotorRun([185, 65, 0, 150], [3, 2, 0, 4])
    time.sleep(0.5)
    motor.dxlSetVelo([55, 55, 55, 55, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([155, 175], [4, 3])
    time.sleep(0.1)
    motor.simMotorRun([195, 205], [4, 3])
    time.sleep(0.1)
    motor.simMotorRun([155, 175], [4, 3])
    time.sleep(0.1)
    motor.simMotorRun([195, 205], [4, 3])
    

def Thankyou():
    start_time = time.time()
    print("Thank you")
    hand.handmotor("thank you")
    motor.dxlSetVelo([60, 60, 60, 60, 60], [0, 1, 2, 3, 4])
    motor.simMotorRun([130,180],[2,3])
    time.sleep(0.01)
    motor.simMotorRun([180],[4])
    time.sleep(0.01)
    motor.simMotorRun([230],[3])
    time.sleep(0.01)
    motor.simMotorRun([180],[3])
    time.sleep(0.01)
    motor.simMotorRun([230],[3])
    time.sleep(0.01)
    motor.simMotorRun([180],[3])

def No():
    start_time = time.time()
    print("No")
    hand.handmotor("no")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220, 85, 265], [3, 2, 4])

def Goodbye():
    start_time = time.time()
    print("Goodbye")
    hand.handmotor("goodbye")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220, 85, 265], [3, 2, 4])

# Setup initial motor positions
def startsetup():
    print("set up move")
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([90, 270, 130, 264, 270], [0, 1, 2, 3, 4])
    time.sleep(1)
    hand.handmotor("openhand")

# Dictionary mapping commands to functions
Command_dict = {
    "hello": Hello,
    "no": No,
    "thankyou": Thankyou,
    "handshake": Handshake,
    "highfive": Highfive,
    "goodbye": Goodbye,
    "yes": Yes,
    "fistbump": Fistbump,
}

def main(args=None):
    # rclpy.init(args=args)
    # node = MyNode()
    while True:
        command = input("Enter a command: ")
        if command in Command_dict:
            startsetup()
            Command_dict[command]()
            startsetup()
        elif command == "exit":
            print("Exiting program.")
            break
        else:
            print("Invalid command. Please try again.")
    # rclpy.shutdown()

# Run the main function
if __name__ == '__main__':
    main()