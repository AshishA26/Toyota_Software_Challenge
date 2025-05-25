from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for base robot speed for 1 revolution per second
RobotVelocity = 0.2394

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 0

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

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
        control.start_keyboard_input()
        control.start_keyboard_control()
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing


    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall
            print(lidar.checkScan())

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 2
            
    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)
            aprilTagInfo = camera.estimate_apriltag_pose(camera.rosImg_to_cv2()) # April tags should be 6, 7, 3, 5 from the bottom right corner of Loop B. 
            if aprilTagInfo == []:
                uuid = aprilTagInfo[0][0]
                # Normalize position-Mathew.
                if uuid == 6:
                    # instructions.
                    adjust_position(*aprilTagInfo[0][1:], , , )
                    
                    _=_
                elif uuid == 7:
                    # instructions.
                    _
                elif uuid == 3:
                    # instructions.
                    _
                elif uuid == 5:
                    #instructions.
                    _
            else:
                # Panic.
                _

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


def adjust_position(range, bearing_rad, elevation_rad,  desired_range, desired_bearing_rad, desired_elevation_rad):
            #Calculate Requirered Movement
            (current_x, current_y) = (range * math.cos(bearing_rad * (math.pi / 180)) * math.cos(elevation_rad * (math.pi / 180)), range * math.sin(bearing_rad * (math.pi / 180)) * math.cos(elevation_rad * (math.pi / 180)))
            (desired_x, desired_y) = (desired_range * math.cos(desired_bearing_rad * (math.pi / 180)) * math.cos(desired_elevation_rad * (math.pi / 180)), desired_range * math.sin(desired_bearing_rad * (math.pi / 180)) * math.cos(desired_elevation_rad * (math.pi / 180)))
            (move_x, move_y) = (current_x - desired_x, current_y - desired_y)
            move_range = math.sqrt(move_x^2 + move_y^2)
            move_angle_deg = math.tan(move_y / move_x) * (180 / math.pi)

            #Move (assuming units of range and robot speed cancel out) (sub out 1 for speed)
            control.rotate(move_angle_deg, 1)
            speed = RobotVelocity*1 # RoboVel * Revolutions
            control.set_cmd_vel(speed , 0, move_range / speed) # Radius ≈ 0.038 m 
            control.rotate(move_angle_deg, -1)


# list[tuple[int, float, float, float]
def normalize_pos(tuple):
    return _