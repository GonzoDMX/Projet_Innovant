#!/usr/bin/env python3
import os, sys, math, time
import cv2
import neopixel, board
from adafruit_servokit import ServoKit
from revolute_robot import RevoluteRobot
from vision_manager import FaceTracker

# Initialize RGB LED
led = neopixel.NeoPixel(board.D18, 1)
tracker = FaceTracker(resolution=(800, 600), display=True, brightness=100)
kit = ServoKit(channels=16)
program_running = True

claw = True

hard_limits = [(-15, 15), (0, 25), (15, 30)]

rest_pos = (0, 10, 15)
shutdown = False

def init_robot_pos(robo):
    start_pos = [160, 100, 45, 180, 10, 90]
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

def recover_robot(robo):
    recover = [90.0, 40.0, 90.0, 40.0]
    offset = [1.0, 2.0, 1.0, 0.5]
    while True:
        angles = robo.get_angles()
        joint = 4
        valid = 0
        for i in range(1, len(angles)):
            if abs(angles[i] - recover[i]) < 0.01:
                valid += 1
            else:
                if abs(angles[i] - recover[i]) <= 1:
                    angles[i] = recover[i]
                elif angles[i] > recover[i]:
                    angles[i] -= offset[i]
                elif angles[i] < recover[i]:
                    angles[i] += offset[i]
                if joint == 2:
                    angle = 90 - angles[i]
                    if angle < 0:
                        angle = 0
                    kit.servo[joint].angle = angle
                elif joint == 3:
                    angle = map_range(angles[i], 0, 180, 90, 210)
                    kit.servo[joint].angle = angle
                else:
                    kit.servo[joint].angle = angles[i]
            joint -= 1
        print("Angles: ", angles)    
        robo.set_angles(angles)
        time.sleep(0.05)
        print("Valid", valid)
        if valid == len(angles)-1:
            break
    pos = robo.end_position['current']
    print("Pos: ", pos.x, pos.y, pos.z)
    robo.set_target(pos.x, pos.y, pos.z)
    

if __name__=="__main__":
    # Initialize RGB LED
    led[0] = (0, 255, 0)
    tracker.start()
    init_servos()
    myRobot = build_robot()
    # Set initial robot position
    init_robot_pos(myRobot)
    myRobot.set_target(0, -8, 33)
    first = True
    i_count = 0
    reset_y = 0
    push = False
    pull = False
    while program_running:
        if not myRobot.on_target():
            if i_count > 50:
                print("Recover")
                recover_robot(myRobot)
                i_count = 0
            else:

                myRobot.move_to_target()
                joint = 5
                for theta in myRobot.theta['theta']:
                    angle = math.degrees(theta)
                    #print("Origin {}: {}".format(joint, angle))
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
                    #print("Joint {}: {} degees".format(joint, angle))
                    joint -= 1
                i_count += 1
                time.sleep(0.01)
        else:
            if first:
                myRobot.set_target(0, -10, 20)
                first = False
                continue
            i_count = 0
            # If tracking a face, get its offset position
            if tracker.tracking and tracker.new_frame:
                reset_y = 0
                tracker.new_frame = False
                off_x, off_y = tracker.get_center_offset()
                pos = myRobot.end_position["target"]
                if abs(off_x) > 20:
                    if off_x < 0:
                        pos.x += 0.25
                    else:
                        pos.x -= 0.25

                if abs(off_y) > 20:
                    if off_y < 0:
                        if pos.y >= -10:
                            if pos.z < 40:
                                pos.z += 0.5
                        else:
                            pos.y += 0.5
                    else:
                        pos.z -= 0.5
                myRobot.set_target(pos.x, pos.y, pos.z)
                #print("Coords: {}, {}, {}".format(pos.x, pos.y, pos.z))
                # TODO Fix bug in distance movements

                d = tracker.get_distance()
                if d == 0:
                    d = tracker.get_width()
                    if d > 350:
                        if pos.y >= -4:
                            print("Warning")
                            pull = True
                            led[0] = (255, 255, 255)
                            time.sleep(0.02)
                            led[0] = (0, 0, 0)
                            time.sleep(0.02)
                            led[0] = (255, 255, 255)
                            time.sleep(0.02)
                        else:
                            print("Pull back")
                            pull = True
                            pos.z -= 1
                            pos.y += 2
                            myRobot.set_target(pos.x, pos.y, pos.z)
                elif d < 330:
                    if pos.y >= -4:
                            print("Warning")
                            pull = True
                            led[0] = (255, 255, 255)
                            time.sleep(0.02)
                            led[0] = (0, 0, 0)
                            time.sleep(0.02)
                            led[0] = (255, 255, 255)
                            time.sleep(0.02)
                    else:
                        print("Pull back")
                        pull = True
                        pos.z -= 1
                        pos.y += 2
                        myRobot.set_target(pos.x, pos.y, pos.z)
                print("Position Y: ", pos.y)

            if not pull:
                pos = myRobot.end_position["current"]
                if pos.y > -10 and reset_y > 20:
                    d = tracker.get_distance()
                    if d == 0:
                        d = tracker.get_width()
                        if d < 300:
                            push = True
                    elif d > 0:
                        if d > 450:
                            push = True
                    else:
                        push = True
                if push:
                    pos.y -= 1
                    pos.z += 0.5
                    myRobot.set_target(pos.x, pos.y, pos.z)
                    push = False
                    reset_y = 0
                else:
                    reset_y += 1
                    time.sleep(0.01)
            else:
                pull = False
                
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
                
        if shutdown and myRobot.on_target():
            kit.servo[0].angle=160
            break
        if not tracker.active:
            myRobot.set_target(0,-5,15)
            shutdown = True
            led[0] = (0, 255, 0)
