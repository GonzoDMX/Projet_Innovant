#!/usr/bin/env python3

#from imutils.video.pivideostream import PiVideoStream
#from picamera.array import PiRGBArray
#from picamera import PiCamera
from vision_helper import euclidean_distance, distance_from_camera, brighten_grayscale
import imutils
import math
import cv2
import numpy as np
#import neopixel
#import board

# Set OpenCV Font
font = cv2.FONT_HERSHEY_SIMPLEX

# True if currently tracking a face
tracking_state = False

# Initialize RGB LED
#pixels = neopixel.NeoPixel(board.D18, 1)
#pixels[0] = (0, 255, 0)

# Load classifiers
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

# Setup Camera
#cam = PiVideoStream(resolution=(640, 480))
#cam.start()

cam = cv2.VideoCapture(0)

print('Enter target eye width (in millimeters):')
real_width = input()
real_width = float(real_width)

print('Enter distance to target (in millimeters):')
target_distance = input()
target_distance = float(target_distance)

while 1:
    # Get image from camera
    #image = cam.read()
    ret, image = cam.read()

    # Clean up image for object detection
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_gray = cv2.equalizeHist(image_gray)
    image_gray = brighten_grayscale(image_gray, 100)
        
    # Run face detection
    faces = face_cascade.detectMultiScale(image_gray, 1.3, 5)

    if len(faces) > 0:
        tracking_state = True
        #pixels[0] = (255, 0, 0)
        for (x,y,w,h) in faces:
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
            cx = int(x+w/2)
            cy = int(y+h/2)
            eyes_gray = image_gray[y+int(h*0.25):int(y+h*0.6), x:x+w]
            eyes_color = image[y+int(h*0.25):int(y+h*0.6), x:x+w]

            eyes = eye_cascade.detectMultiScale(eyes_gray, 1.3, 5)
            eye_count = 0
            eye_pts = []
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(eyes_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
                cx = int(ex+ew/2)
                cy = int(ey+eh/2)
                cv2.circle(eyes_color, (cx, cy), radius=3, color=(0, 0, 255), thickness=-1)
                eye_count += 1
                eye_pts.append((cx, cy))
                if eye_count == 2:
                    break
            eye_count = 0

            if len(eye_pts) == 2:
                cv2.line(eyes_color, eye_pts[0], eye_pts[1], (255, 0, 255), 1)
                # Get eye width in pixels
                eye_width = euclidean_distance(eye_pts[0], eye_pts[1])
            else:
                tracking_state = False
    else:
        tracking_state = False
        #pixels[0] = (0, 255, 0)

    cv2.imshow('img', image)
    k = cv2.waitKey(30) & 0xff
    # If escape is pressed
    if k == 27:
        print("Calibration cancelled... Closing.")
        break
    # If space bar is pressed
    if k == 32:
        if tracking_state == True:
            cv2.imwrite("config/calibration_img.png", image)
            focal_length = ((eye_width * target_distance) / real_width)
            print("Calculated focal length: {}".format(focal_length))
            print("Saving...")
            with open("config/focal_length.conf", 'r+') as f:
                f.truncate(0)
                f.seek(0)
                f.write(str(real_width) + '\n' + str(focal_length))
                print("Calibration complete.")
            break
    
# Clean up processes
#cam.stop()
cam.release()
cv2.destroyAllWindows()
