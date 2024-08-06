#!/usr/bin/env python3
import math
import Motorcontrol as motor
import movementcalc as calculation
import numpy as np
import time
import cv2
import rasptoarduino as hand
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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

class MyNode(Node):
    def __init__(self):
        super().__init__("arm_working")
        self.publisher_ = self.create_publisher(String, 'gesture_done', 10)

        # self.subscription = self.create_subscription(
        #     String,'camera_gesture',self.listener_callback,10)
        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Arm is turning on")
        while True:
            command = input("Enter a command: ")
            if command in Command_dict:
                self.get_logger().info(f'Received command: {command}')
                startsetup()
                Command_dict[command]()
                startsetup()
                self.publish_feedback(f'Command {command} executed successfully. Waiting for next reply.')
            else:
                self.get_logger().info(f'Invalid command received: {command}')


    # def listener_callback(self, msg):

    def publish_feedback(self, feedback):
        msg = String()
        msg.data = feedback
        self.publisher_.publish(msg)
        self.get_logger().info(f'Gesture finished: {msg.data}')

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
    motor.simMotorRun([190, 80, 180], [3, 2, 4])
    motor.dxlSetVelo([55, 55, 55], [2, 3, 4])
    time.sleep(0.1)
    motor.simMotorRun([60, 170, 175], [2, 3, 4])
    time.sleep(0.3)
    motor.simMotorRun([190, 80, 185], [3, 2, 4])
    
def Highfive():
    start_time = time.time()
    print("Highfive")
    hand.handmotor("highfive")
    motor.dxlSetVelo([30, 30, 30, 30, 65], [0, 1, 2, 3, 4])
    motor.simMotorRun([265, 160, 40], [4, 3, 2])

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
    motor.dxlSetVelo([45, 40, 35, 35, 40], [0, 1, 2, 3, 4])
    motor.simMotorRun([110,195],[2,3])
    time.sleep(0.01)
    motor.simMotorRun([180,270],[4,0])
    time.sleep(0.01)
    motor.simMotorRun([220],[3])
    time.sleep(0.01)
    motor.simMotorRun([190],[3])
    time.sleep(0.01)
    motor.simMotorRun([220],[3])
    time.sleep(0.01)
    motor.simMotorRun([190],[3])
    

def No():
    start_time = time.time()
    print("No")
    hand.handmotor("no")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220, 85, 265], [3, 2, 4])

def Goodbye():
    start_time = time.time()
    print("Goodbye")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([220, 85, 265], [3, 2, 4])
    time.sleep(0.5)
    hand.handmotor("goodbye")
    time.sleep(2.5)

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
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

# Run the main function
if __name__ == '__main__':
    main()
