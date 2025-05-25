from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

RobotVelocity = 0.2394

def static_obstacle_avoidence(self, padding : float , lidar : Lidar, control : Control):
    # Detects distance to obstacles within padding distance using LiDAR. Move back away from obstacle.
    msg = lidar.checkScan()
    front, _ = lidar.detect_obstacle_in_cone(msg, padding, 0, 10) 
    if front != -1:
        print("Wall detected! Moving back")
        control.stop_keyboard_control()
        control.set_cmd_vel(-0.2, 0, 0.3)
        control.start_keyboard_control()
        print("Done moving back")

def wait_at_stopsign(self, min_box_height : float, camera : Camera, control : Control):
    print("Fill in here")

def adjust_apriltag_position(range_3D : float, bearing_rad : float, elevation_rad : float, desired_range_2d : float, desired_bearing_rad : float, control : Control):
    # Detect orientation relative to april tag. Adjust to the desired orientation.
    print("Inputs: %d, %d, %d, %d, %d\n", range_3D, bearing_rad, elevation_rad, desired_range_2d, desired_bearing_rad)
    elevation_rad *= (math.pi/180) 
    bearing_rad *= (math.pi/180)
    desired_bearing_rad *= math.pi/180
    range_2D = range_3D*math.cos(elevation_rad)
    print("2D-Range: %d\n", range_2D)

    current_x = range_2D*math.cos(desired_bearing_rad)
    current_y = range_2D*math.sin(desired_bearing_rad)

    desired_x = desired_range_2d*math.cos(desired_bearing_rad)
    desired_y = desired_range_2d*math.sin(desired_bearing_rad)
    print("Current x-y: (%d, %d).... Desired x-y: (%d, %d)\n", current_x, current_y, desired_x, desired_y)
    
    move_x = desired_x - current_x
    move_y = desired_y - current_y
    print("Move to (%d, %d)\n", move_x, move_y)
    
    move_distance = math.hypot(move_x, move_y) * 0.25
    move_heading_rad = math.atan2(move_y, move_x)
    move_heading_deg = math.degrees(move_heading_rad)-180
    print("Move %d units, at %d\n", move_distance, move_heading_deg)

    #Move (assuming units of range and robot speed cancel out) (sub out 1 for speed)
    control.rotate(move_heading_deg, -1)
    speed = RobotVelocity*0.2
    control.set_cmd_vel(0.2*0.25, 0, move_distance / speed)
    control.rotate(move_heading_deg, 1)

def apriltag_rotation(camera : Camera, control : Control):
    # Detect april tag and perorm roation.

    # tag_instructions: maps april tag identifiers to required rotation angle (in degrees), direction, desired range from april tag, and desired bearing from april tag (in radians)
    tag_instructions = {
        "1": (45, 1, 100, 4),
        "2": (45, 1, 100, 4),
        "3": (45, 1, 100, 4),
        "4": (45, 1, 100, 4),
        "5": (45, 1, 100, 4),
        "6": (45, 1, 100, 4)
    }
    (tag_id, range, bearing_rad, elevation_rad) = camera.estimate_apriltag_pose(camera.rosImg_to_cv2)
    (angle_deg, direction, desired_range, desired_bearing_rad) = tag_instructions.get(tag_id)
    if (tag_id != None):
        control.set_cmd_vel(0,0,0.5)
        adjust_apriltag_position(range, bearing_rad, elevation_rad, desired_range, desired_bearing_rad)
        control.rotate(angle_deg, direction)
    


