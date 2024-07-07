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

BASE_ID = 1
BICEP_ID = 2
FOREARM_ID = 3
WRIST_ID = 4
CLAW_ID = 0

PORT_NUM = '/dev/ttyUSB0'  # for rpi
MOVEARM_MODE = 1
ADDR_PRESENT_POSITION = 132
ALL_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]
MOVE_IDs = [BASE_ID, BICEP_ID, FOREARM_ID, WRIST_ID, CLAW_ID]

motor.portInitialization(PORT_NUM, ALL_IDs)
# ctrl.portInitialization(PORT_NUM, ALL_IDs)


#angle for the max length reaching out in the x pos
max_length_angle = calculation.angle_Calc([375, 0, 73], 0)

#"[%s, %s, %s, %s]" % (int(baseTheta), int(shoulderTheta), int(elbowTheta), int(wristTheta))

# class MyNode(Node):
#     def __init__(self):
#         super().__init__("robotic_arm")

def checkMovement(ids):
    motorStatus = [0] * len(ids)
    finished = [1] * len(ids)
    firstPosition = 0
    secondPosition = 0
    while True:
        for id in (ids):
            firstPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            time.sleep(.1)
            secondPosition = motor.ReadMotorData(id, ADDR_PRESENT_POSITION)
            if (abs(firstPosition - secondPosition) < 5):
                motorStatus[id] = 1
        if (motorStatus == finished):
            print("finished")
            break


def Hello():
    start_time = time.time()
    print("Hello")
    motor.dxlSetVelo([30, 55, 30, 30, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([180,50,265], [3,2,4])
    time.sleep(0.1)
    motor.simMotorRun([240],[1])
    time.sleep(0.1)
    motor.simMotorRun([300],[1])
    time.sleep(0.1)
    motor.simMotorRun([240],[1])
    time.sleep(0.1)
    motor.simMotorRun([300],[1])

    hand.handmotor("hello")

def Yes():
    start_time=time.time()
    print("Yes")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220,85,265], [3,2,4])
    time.sleep(0.2)
    motor.simMotorRun([180],[4])
    time.sleep(0.2)
    motor.simMotorRun([265],[4])
    time.sleep(0.2)
    motor.simMotorRun([180],[4])
    time.sleep(0.2)
    motor.simMotorRun([265],[4])

    hand.handmotor("Yes")

def Fistbump():
    start_time=time.time()
    print("fistbump")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([200,80,180],[3,2,4])
    motor.dxlSetVelo([55,55,55],[2,3,4])
    time.sleep(0.1)
    motor.simMotorRun([60,180,172],[2,3,4])
    time.sleep(0.3)
    motor.simMotorRun([200,80,180],[3,2,4])

    hand.handmotor("fistbump")
    
def Highfive():
    start_time=time.time()
    print("Highfive")
    motor.dxlSetVelo([30, 30, 30, 30, 65], [0, 1, 2, 3, 4])
    motor.simMotorRun([265,160,40],[4,3,2])
    motor.dxlSetVelo([30, 30, 50, 50, 50], [0, 1, 2, 3, 4])
    motor.simMotorRun([20,135,245],[2,3,4])
    time.sleep(0.2)
    motor.simMotorRun([40,160,265],[2,3,4])

    hand.handmotor("highfive")

def Handshake():
    start_time=time.time()
    print("Handshake")
    motor.dxlSetVelo([40, 40, 40, 40, 40], [0, 1, 2, 3, 4])
    motor.simMotorRun([185,65,180,150],[3,2,0,4])
    time.sleep(0.5)
    motor.dxlSetVelo([55, 55, 55, 55, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([155,175],[4,3])
    time.sleep(0.1)
    motor.simMotorRun([195,205],[4,3])
    time.sleep(0.1)
    motor.simMotorRun([155,175],[4,3])
    time.sleep(0.1)
    motor.simMotorRun([195,205],[4,3])

    hand.handmotor("handshake")

def Thankyou():
    start_time=time.time()
    print("Thank you")
    motor.dxlSetVelo([40, 40, 40, 20, 40], [0, 1, 2, 3, 4])
    motor.simMotorRun([90,130,180],[1,2,3])
    time.sleep(0.01)
    motor.dxlSetVelo([40, 40, 40, 65, 40], [0, 1, 2, 3, 4])
    motor.simMotorRun([220],[3])
    time.sleep(0.1)
    motor.simMotorRun([155],[3])
    time.sleep(0.1)
    motor.simMotorRun([220],[3])
    time.sleep(0.1)
    motor.simMotorRun([155],[3])
    time.sleep(0.1)
    motor.simMotorRun([180],[3])

    hand.handmotor("thank you")

def No():
    start_time=time.time()
    print("No")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220,85,265], [3,2,4])

    hand.handmotor("no")

def Goodbye():
    start_time=time.time()
    print("Goodbye")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220,85,265], [3,2,4])

    hand.handmotor("goodbye")

def startsetup():
    print("set up move")
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])
    motor.simMotorRun([90,270,140,265,180], [0,1,2,3,4])

Command_dict = {
    "Hello": Hello,
    "No": No,
    "Thankyou": Thankyou,
    "Handshake": Handshake,
    "Highfive": Highfive,
    "Goodbye": Goodbye,
    "Yes": Yes,
    "Fistbump": Fistbump,
}

def main(args=None):
    # rclpy.init(args=args)
    # node=MyNode()
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

if __name__=='__main__':
    main()
