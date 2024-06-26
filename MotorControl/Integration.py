import math
import Motorcontrol as motor
import movementcalc as calculation
import numpy as np
import time
import cv2
import websockets

#Distance from BVM to BTP 16.5 inches

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

# def time_between_moves(velocity: list[int], ids: list[int], startAngles: list[int], midAngles: list[int], endAngles: list[int]) -> None:
#     ctrl.dxlSetVelo(velocity, ids)  # ALWAYS SET SPEED BEFORE ANYTHING
#     time.sleep(0.1)
#     print("Move 1")
#     ctrl.motorRun(startAngles, ids)  # Reset claw looking up
#     start_time = time.time()
#     checkMovement(ids)

#     check_time = time.time()

#     print("Move 2")
#     check_time2 = time.time()
#     ctrl.motorRun(midAngles, ids)
#     checkMovement(ids)

#     print("Move 3")
#     ctrl.motorRun(endAngles, ids)
#     end_time = time.time()

#     # print("total time")
#     # print(end_time-start_time)
#     # print("check time")
#     # print(check_time-start_time)
#     # print("check2 time")
#     # print(check_time2-start_time)
#     # print("check to check time")
#     # print(check_time2-check_time)

# def move(velocity: list[int], ids: list[int], startAngles: list[int], midAngles: list[int], endAngles: list[int]) -> None:
#     # ctrl.dxlSetVelo(velocity, ids)  # ALWAYS SET SPEED BEFORE ANYTHING
#     # time.sleep(0.1)
#     # print("Sleep move")
#     # print("Move 1")
#     # ctrl.motorRun(startAngles, ids)  # Reset claw looking up
#     # time.sleep(3)
#     # print("Move 2")
#     # ctrl.motorRun(midAngles, ids)
#     # time.sleep(3)
#     # print("Move 3")
#     # ctrl.motorRun(endAngles, ids)

#     print("Optimized Move")
#     print("Move 1.2")
#     ctrl.motorRun(startAngles, ids)  # Reset claw looking up
#     time.sleep(1)
#     start_time = time.time()
#     print("Move 2.2")
#     ctrl.motorRun(midAngles, ids)
#     time.sleep(0.1)
#     print("Move 3.2")
#     ctrl.motorRun(endAngles, ids)
#     end_time = time.time()
#     print(end_time)

# def sleepless_move(velocity: list[int], ids: list[int], startAngles: list[int], midAngles: list[int], endAngles: list[int]) -> None:
#     ctrl.dxlSetVelo(velocity, ids)  # ALWAYS SET SPEED BEFORE ANYTHING
#     time.sleep(0.1)
#     print("Sleepless move")
#     print("Move 1")
#     ctrl.motorRun(startAngles, ids)  # Reset claw looking up
#     print("Move 2")
#     ctrl.motorRun(midAngles, ids)
#     print("Move 3")
#     ctrl.motorRun(endAngles, ids)


def debug_gcs_push_in():
    #Push In Battery
    start_time = time.time()
    motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)
    #265, 47, 170

    print("move back to chamber2")
    motor.simMotorRun([180, 56], [2, 3])
    time.sleep(3)

    print("move back to chamber2")
    motor.simMotorRun([180, 70], [2, 3])
    time.sleep(2.5)
    motor.dxlSetVelo([40, 40, 40, 40, 40], [0, 1, 2, 3, 4])  # ALWAYS SET SPEED BEFORE ANYTHING
    time.sleep(0.1)

    print("adjust")
    for i in range(190,380,10):
        print(i)
        initial_push_angle = calculation.angle_Calc([i,-3,72], 0)
        print("push 4 slight")
        motor.simMotorRun(initial_push_angle, [1, 2, 3, 4])
        time.sleep(.1)
        motor.dxlPresPos([0, 1, 2, 3, 4])

    print("push all the way in to chamber")
    motor.simMotorRun(max_length_angle, [1, 2, 3, 4])
    time.sleep(0.15)

    print("open claw")
    motor.simMotorRun([110], [0])  # Reset claw looking up
    time.sleep(0.15)

    gcs_pull_out_angle = calculation.angle_Calc([300,0,60], 0)
    print("move 4 pull away slight")
    motor.simMotorRun(gcs_pull_out_angle, [1, 2, 3, 4])
    time.sleep(1.5)

    print("set up move")
    motor.simMotorRun([98, 225, 260, 47, 272], [0, 1, 2, 3, 4])  # Reset claw looking up
    time.sleep(1)

# def Hello():
#     start_time = time.time()
# if __name__ == "__main__":
#     print("set up move")
#     motor.dxlSetVelo([30, 55, 30, 30, 30], [0, 1, 2, 3, 4])
#     motor.simMotorRun([90,270,140,265,180], [0,1,2,3,4])
#     time.sleep(2)
#     motor.simMotorRun([180,50,265], [3,2,4])
#     time.sleep(0.1)
#     motor.simMotorRun([240],[1])
#     time.sleep(0.1)
#     motor.simMotorRun([300],[1])
#     time.sleep(0.1)
#     motor.simMotorRun([240],[1])
#     time.sleep(0.1)
#     motor.simMotorRun([300],[1])

# def yesno():
#     start_time=time.time()
# if __name__ == "__main__":
#     print("set up move")
#     motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
#     motor.simMotorRun([90,270,140,265,180], [0,1,2,3,4])
#     time.sleep(1)
#     motor.simMotorRun([220,85,265], [3,2,4])
#     time.sleep(0.2)
#     motor.simMotorRun([180],[4])
#     time.sleep(0.2)
#     motor.simMotorRun([265],[4])
#     time.sleep(0.2)
#     motor.simMotorRun([180],[4])
#     time.sleep(0.2)
#     motor.simMotorRun([265],[4])


def handshake():
    start_time=time.time()
if __name__ == "__main__":
    print("set up move")
    motor.dxlSetVelo([30, 30, 30, 30, 55], [0, 1, 2, 3, 4])
    motor.simMotorRun([90,270,140,265,180], [0,1,2,3,4])
    time.sleep(1)
    motor.simMotorRun([165,55,190],[3,2,4])


    


# if __name__ == "__main__":
    # print("set up move")
    # motor.dxlSetVelo([30, 30, 30, 30, 30], [0, 1, 2, 3, 4])
    # motor.simMotorRun([90,270,140,265,180], [0,1,2,3,4])  # ALWAYS SET SPEED BEFORE ANYTHING
    # # time.sleep(3)  
    # motor.simMotorRun([90,295,160,45], [4,1,3,2])  # ALWAYS SET SPEED BEFORE ANYTHING
    # motor.simMotorRun([20],[0])
    # motor.simMotorRun([110,20], [3,0])
    # time.sleep(5)
    # motor.simMotorRun([260,65,160], [4,2,3])
    # velocity = [25, 25, 25, 25, 25]
    # ids = [0, 1, 2, 3, 4]
    # startAngles = [110, 223, 270, 47, 160]
    # midAngles = [110, 223, 250, 60, 220]
    # endAngles = [110, 223, 210, 80, 270]
    # time_between_moves(velocity=velocity, ids=ids, startAngles=startAngles, midAngles=midAngles, endAngles=endAngles)
    # time.sleep(5)
    # move(velocity=velocity, ids=ids, startAngles=startAngles, midAngles=midAngles, endAngles=endAngles)
    # gcs_pullout()
    # debug_gcs_pull_out()
    # time.sleep(5)
    # debug_gcs_push_in()
    
    # debug_bvm_pull_out()
    # time.sleep(5)
    # debug_bvm_push_in()