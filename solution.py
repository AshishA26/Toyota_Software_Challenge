from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
import challenge_helpers as funcs

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 3

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True
RobotVelocity = 0.2394
# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing


    if challengeLevel == 1:
        Padding = 0.5 # Assume standard units 'm'. STC! Test to fine tune
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall
            msg = lidar.checkScan()
            front, _ = lidar.detect_obstacle_in_cone(msg, Padding, 0, 10) 
            if front != -1:
                print("Wall detected! Moving back")
                control.stop_keyboard_control()
                control.set_cmd_vel(-0.2, 0, 0.3)
                control.start_keyboard_control()
                print("Done moving back")

    if challengeLevel == 2:
        min_box_size = 8
        flag = True
        while rclpy.ok():
            # Write your solution here for challenge level 2
            (detected, x1, y1, x2, y2) = camera.ML_predict_stop_sign(camera.rosImg_to_cv2())
            box_size = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
            if (detected == False):
                print("Reseting stopping flag")
                flag = True
            if(detected == True and box_size > min_box_size and flag == True):
                print("Detected stop sign, stopping for 3 seconds")
                flag = False
                control.stop_keyboard_control
                control.set_cmd_vel(0,0,3)
                control.start_keyboard_control
            time.sleep(0.1)
            
    if challengeLevel == 3:
        padding = 0.3
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)
            camera.checkImageRelease()
            aprilTagInfo = camera.estimate_apriltag_pose(camera.rosImg_to_cv2()) # April tags should be 6, 7, 3, 5 from the bottom right corner of Loop B. 
            if aprilTagInfo != []:
                uuid = aprilTagInfo[0][0]
                # Normalize position-Mathew.
                print(uuid)
                if uuid == 7:
                    # instructions.
                    funcs.adjust_apriltag_position(*aprilTagInfo[0][1:], 1, 0, control)
                    print("Done april tag")
                    done_april_tag = False
                    while not done_april_tag:
                        msg = lidar.checkScan()
                        front, _ = lidar.detect_obstacle_in_cone(msg, padding, 0, 10) 
                        if front != -1:
                            control.set_cmd_vel(0, 0, 1)
                            control.rotate(90, 1)
                            done_april_tag=True
                        else:
                            control.set_cmd_vel(0.2, 0, 0.2)
                    


            else:
                # Panic.
                pass




    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
