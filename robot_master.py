#!/usr/bin/env python3
import os, sys, math, time
import cv2
import neopixel, board
from adafruit_servokit import ServoKit
from revolute_robot import RevoluteRobot
from vision_manager import FaceTracker

# Initialize RGB LED
led = neopixel.NeoPixel(board.D18, 1)
tracker = FaceTracker(display=True)
kit = ServoKit(channels=16)
program_running = True

claw = True

hard_limits = [(-15, 15), (0, 25), (15, 30)]

def init_robot_pos(robo):
    start_pos = [160, 100, 35, 180, 10, 90]
    s = 0
    # Activate Servos
    for pos in start_pos:
        kit.servo[s].angle = pos
        s += 1
    # Force the starting position
    s = len(start_pos)-1
    for joint in range(len(robo.theta['theta'])):
        robo.theta['theta'][joint] = math.radians(start_pos[s])
        s -= 1
    robo.update_posture()

def init_servos():
    for i in range(1, 6, 1):
        kit.servo[i].set_pulse_width_range(500, 2500)
    kit.servo[0].set_pulse_width_range(1800, 2300)

def build_robot():
    # Create robot object
    robot = RevoluteRobot()
    # Define its joints
    robot.add_joint_link(length=9.5,  min_theta= 0, max_theta=360, theta=90)
    robot.add_joint_link(length=11, min_theta= 0, max_theta=360, theta=180)
    robot.add_joint_link(length=10.5, min_theta= 0, max_theta=360, theta=90)
    robot.add_joint_link(length=10.0, min_theta= 0, max_theta=360, theta=0)
    return robot

def map_range(value, old_min, old_max, new_min, new_max):
    old_span = old_max - old_min
    new_span = new_max - new_min
    scale_val = float(value - old_min) / float(old_span)
    return new_min + (scale_val * new_span)

def update_thetas(thetas):
    # Joint offset relative to kit.servo
    joint = 5
    for theta in thetas:
        angle = math.degrees(theta)
        if angle > 180:
            angle = 180
        if angle < 0:
            angle = 0
        if joint==2:
            if angle > 90:
                angle = 0
            else:
                angle = 90 - angle
        if joint == 3:
            angle = map_range(angle, 0, 180, 90, 210)
            if angle > 180:
                angle = 180
        kit.servo[joint].angle = angle
        joint -= 1

if __name__=="__main__":
    # Initialize RGB LED
    led[0] = (0, 255, 0)
    tracker.start()
    init_servos()
    myRobot = build_robot()
    # Set initial robot position
    init_robot_pos(myRobot)
    myRobot.set_target(0, -15, 25)
    t = 0
    while program_running:
        # If robot is at desired position
        if myRobot.on_target():
            # If tracking a face, get its offset position
            if tracker.tracking and tracker.new_frame:
                tracker.new_frame = False
                off_x, off_y = tracker.get_center_offset()
                if abs(off_x) > 20:
                    if off_x < 0:
                        myRobot.theta['theta'][0] += math.radians(3)
                    else:
                        myRobot.theta['theta'][0] -= math.radians(3)
                    deg = math.degrees(myRobot.theta['theta'][0])
                    if deg < 0:
                        myRobot.theta['theta'][0] = math.radians(0)
                    if deg > 180:
                        myRobot.theta['theta'][0] = math.radians(180)
                    kit.servo[5].angle = math.degrees(myRobot.theta['theta'][0])
                
                if abs(off_y) > 20:
                    if off_y < 0:
                        myRobot.theta['theta'][3] += math.radians(3)
                    else:
                        myRobot.theta['theta'][3] -= math.radians(3)
                    deg = math.degrees(myRobot.theta['theta'][3])
                    deg = map_range(deg, 0, 180, 90, 210)
                    if deg > 180:
                        kit.servo[2].angle = 180
                    else:
                        kit.servo[2].angle = deg
                
                myRobot.update_posture()
                pos = myRobot.end_position["current"]
                myRobot.set_target(pos.x, pos.y, pos.z)
                
                # TODO Fix bug in distance movements
                d = tracker.get_distance()
                if d > 100 and d < 600:
                    if d < 380:
                        pos.y += 1
                        pos.z -= 1
                        if pos.y > 0:
                            pos.y = 0
                        if pos.z < 15:
                            pos.z = 15
                    elif d > 550:
                        pos.y -= 1
                        pos.z += 1
                        if pos.y < -25:
                            pos.y = -25
                        if pos.z > 30:
                            pos.z = 30
                    print(pos.x, pos.y, pos.z)
                    myRobot.set_target(pos.x, pos.y, pos.z)
                
            # Update Claw position
            if tracker.hand and not claw:
                kit.servo[0].angle=160
                claw = True
            elif claw and not tracker.hand:
                kit.servo[0].angle=0
                claw = False

            # Update LED Color
            blue = 255 * (tracker.hand_count * 0.33)
            if tracker.tracking:
                led[0] = (255-blue, 0, blue)
            else:
                led[0] = (0, 255-blue, blue)

        elif not myRobot.on_target():
            myRobot.move_to_target()
            update_thetas(myRobot.theta['theta'])
            time.sleep(0.05)
