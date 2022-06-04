#!/usr/bin/env python3

import math, time
import cv2
import numpy as np
from imutils.video.pivideostream import PiVideoStream
from threading import Thread

""" Data class for current observed face """
class Face:
    def __init__(self, x=0, y=0, width=0, height=0):
        self.x = x
        self.y = y
        self.center_x = x + int(width * 0.5)
        self.center_y = y + int(height * 0.5)
        self.width = width
        self.height = height
        self.area = width * height
        self.distance = 0.0

""" Class handles camera, face tracking and distance calculation """
class FaceTracker:
    def __init__(self, resolution=(640, 480), display=False, brightness=50):
        self.display = display
        self.resolution = resolution
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.active = True
        self.brightness = brightness
        # Load classifiers
        self.detect_face = cv2.CascadeClassifier('models/frontalface.xml')
        self.detect_eyes = cv2.CascadeClassifier('models/eye.xml')
        self.detect_palm = cv2.CascadeClassifier('models/palm.xml')
        self.detect_fist = cv2.CascadeClassifier('models/fist.xml')
        # Load calibration data
        self.real_width, self.focal_length = self.load_calibration()
        # True if currently tracking a face
        self.tracking = False
        # True if hand position closed
        self.hand = True
        self.hand_count = 0
        # Holds data on the currently tracked face
        self.face = Face()
        # Used to calculate rolling average of distance
        self.rolling_buffer = []
        # Setup Camera
        self.camera = PiVideoStream(resolution=resolution)
        self.new_frame = False

    ''' Load calibration data from config '''
    def load_calibration(self):
        with open("config/focal_length.conf", "r") as f:
            f_len = f.readlines()
        return float(f_len[0]), float(f_len[1])

    ''' Start threaded class operation '''
    def start(self):
        self.camera.start()
        time.sleep(0.5)
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    ''' Stop class operations and clean up '''
    def stop(self):
        self.active = False
        self.camera.stop()
        if self.display:
            cv2.destroyAllWindows()

    ''' Main function of thread '''
    def update(self):
        while self.active:
            frame = self.camera.read()
            gray_frame = self.clean_grayscale(frame)
            if self.display:
                    cv2.imshow("gray", gray_frame)
            there_is_a_face = self.get_closest_face(gray_frame)
            if there_is_a_face:
                self.tracking = True
                self.face.distance = self.get_distance_from_eyes(frame,
                                                                 gray_frame)
                self.update_hand_state(gray_frame)
                if self.display:
                    cv2.rectangle(frame,
                                  (self.face.x,self.face.y),
                                  (self.face.x + self.face.width,
                                   self.face.y + self.face.height),
                                  (255,0,0), 2)
                    if self.face.distance > 0:
                        cv2.putText(frame,
                                    str(int(self.face.distance))+"mm",
                                    (self.face.x, self.face.y+30+self.face.height),
                                    self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        cv2.putText(frame,
                                    str(int(self.face.width))+"px",
                                    (self.face.x, self.face.y+30+self.face.height),
                                    self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                self.tracking = False
            self.new_frame = True
            if self.display:
                if self.hand:
                    cv2.putText(frame,
                                "Closed",
                                (30, 30),
                                self.font, 2, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    cv2.putText(frame,
                                "Open",
                                (30, 30),
                                self.font, 2, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("Frame", frame)
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                self.stop()
                break

    ''' Returns the face closest to camera '''
    def get_closest_face(self, frame):
        self.face = Face()
        found_faces = self.detect_face.detectMultiScale(frame, 1.3, 5)
        if len(found_faces) > 0:
            for (x, y, w, h) in found_faces:
                if (w * h) > self.face.area:
                    self.face = Face(x, y, w, h)
            return True
        return False

    ''' Calculate the distance from camera to face based on eye width '''
    def get_distance_from_eyes(self, frame, gray_frame):
        # Separate eyes from face
        top_border = self.face.y + int(self.face.height * 0.25)
        bottom_border = self.face.y + int(self.face.height * 0.6)
        gray_eyes = gray_frame[top_border:bottom_border,
                               self.face.x:self.face.x+self.face.width]
        #kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        #gray_eyes = cv2.filter2D(src=gray_eyes, ddepth=-1, kernel=kernel)
        if self.display:
            cv2.imshow("eyes", gray_eyes)
        found_eyes = self.detect_eyes.detectMultiScale(gray_eyes, 1.3, 5)
        eyes = []
        for (x, y, w, h) in found_eyes:
            eyes.append( (x + int(w*0.5), y + int(h*0.5)) )
        if len(eyes) > 1:
            eye_width = self.euclidean_distance(eyes[0], eyes[1])
            distance = self.distance_from_camera(self.focal_length,
                                                 self.real_width,
                                                 eye_width)
            if self.display:
                color_eyes = frame[top_border:bottom_border,
                                   self.face.x:self.face.x+self.face.width]
                for eye in eyes:
                    cv2.circle(color_eyes, eye,
                               radius=10,
                               color=(0,255,0),
                               thickness=1)
                cv2.line(color_eyes, eyes[0], eyes[1],
                         color=(0,255,255),
                         thickness=2)
            return self.rolling_distance(distance)
        return 0

    ''' Updates the hand position state, closed = True, open = False '''
    def update_hand_state(self, frame):
        #if self.hand:
        #    found_hands = self.detect_palm.detectMultiScale(frame, 1.3, 5)
        #else:
        found_hands = self.detect_fist.detectMultiScale(frame, 1.3, 5)
        if len(found_hands) > 0:
            self.hand_count += 1
            if self.hand_count > 3:
                if self.hand:
                    self.hand = False
                else:
                    self.hand = True
                self.hand_count = 0
        else:
            if self.hand_count > 0:
                self.hand_count -= 1

    ''' Generates grayscale of input frame '''
    def clean_grayscale(self, frame):
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grayscale = cv2.equalizeHist(grayscale)
        mask = (255 - grayscale) < self.brightness
        return np.where(mask, 255, grayscale + self.brightness)

    ''' Get distance from camera to observed object '''
    def distance_from_camera(self, focal_length, real_width, observed_width):
        # compute and return the distance from the maker to the camera
        if observed_width == 0:
            return 0
        return (real_width * focal_length) / observed_width

    ''' Return the distance between two points '''
    def euclidean_distance(self, pt1, pt2):
        return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

    ''' Returns distance based on rolling average '''
    def rolling_distance(self, distance):
        buffer_size = 15
        self.rolling_buffer.append(distance)
        if len(self.rolling_buffer) > buffer_size:
            self.rolling_buffer.pop(0)
        return sum(self.rolling_buffer)/ len(self.rolling_buffer)

    ''' Get face offset from center of frame '''
    def get_center_offset(self):
        off_x = self.face.center_x - (self.resolution[0] * 0.5)
        off_y = self.face.center_y - (self.resolution[1] * 0.5)
        #print("Val Y: y:{}, cy:{}, offy:{}".format(self.face.y, self.face.center_y, off_y))
        return (off_x, off_y)

    ''' Get distance to face '''
    def get_distance(self):
        return self.face.distance
    
    def get_width(self):
        return self.face.width
