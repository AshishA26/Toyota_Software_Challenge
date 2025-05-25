from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 2

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = False

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
        min_box_height = 0
        if is_SIM:
            min_box_height = 50
        else:
            min_box_height = 119
        flag = True
        while rclpy.ok():
            camera.checkImageRelease()
            # Write your solution here for challenge level 2
            (detected, x1, y1, x2, y2) = camera.ML_predict_stop_sign(camera.rosImg_to_cv2())
            box_height = abs(y2)
            print(box_height)
            if (detected == False):
                print("Reseting stopping flag")
                flag = True
            if(detected == True and box_height < min_box_height and flag == True):
                print("Detected stop sign, stopping for 3 seconds")
                flag = False
                control.stop_keyboard_control
                control.set_cmd_vel(0,0,3)
                control.start_keyboard_control
            time.sleep(0.1)
            
    if challengeLevel == 3:
        # tag_instructions = {
        #     "tag1": (45, 1),
        #     "tag2": (45, 1),
        #     "tag3": (45, 1)
        # }
        # (tag_id, range, bearing, elevation) = camera.estimate_apriltag_pose(camera.rosImg_to_cv2)
        # (angle, direction) = tag_instructions.get(tag_id)
        # control.rotate(angle, direction)




        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)

            camera.checkImageRelease()
            img = camera.rosImg_to_cv2()
            print(camera.estimate_apriltag_pose(img))
            # Write your solution here for challenge level 3 (or 3.5)

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
