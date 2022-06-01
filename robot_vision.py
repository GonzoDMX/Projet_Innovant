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
claw_state = True
claw_counter = 0

# Used to smooth calculated distance values
smooth_limit = 10
smooth_buffer = []
smooth_index = 0

# Initialize RGB LED
#pixels = neopixel.NeoPixel(board.D18, 1)
#pixels[0] = (0, 255, 0)

# Load classifiers
face_cascade = cv2.CascadeClassifier('models/frontalface.xml')
eye_cascade = cv2.CascadeClassifier('models/eye.xml')
palm_cascade = cv2.CascadeClassifier('models/palm.xml')
fist_cascade = cv2.CascadeClassifier('models/fist.xml')

# Setup Camera
#cam = PiVideoStream(resolution=(640, 480))
#cam.start()

cam = cv2.VideoCapture(0)

with open("config/focal_length.conf", "r") as f:
    f_len = f.readlines()
real_width = float(f_len[0])
focal_length = float(f_len[1])

print("R_Width: {}".format(real_width))
print("F_Length: {}".format(focal_length))

# Main loop
while 1:

    # Get image from camera
    #image = cam.read()
    ret, image = cam.read()

    # Clean up image for object detection
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_gray = cv2.equalizeHist(image_gray)
    image_gray = brighten_grayscale(image_gray, 100)

    # Run face detector algo
    faces = face_cascade.detectMultiScale(image_gray, 1.3, 5)

    # Check if faces are detected
    if len(faces) > 0:
        if not tracking_state:
            # Indicate a face is being tracked
            #pixels[0] = (255,0,0)
            tracking_state = True

        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x,y), (x+w,y+h), (255,0,0), 2)
            cx = int(x+w/2)
            cy = int(y+h/2)
            eyes_gray = image_gray[y+int(h*0.25):int(y+h*0.6), x:x+w]
            eyes_color = image[y+int(h*0.25):int(y+h*0.6), x:x+w]

            eyes = eye_cascade.detectMultiScale(eyes_gray, 1.3, 5)
            eye_count = 0
            eye_pts = []
            for (ex, ey, ew, eh) in eyes:

                cv2.rectangle(eyes_color, (ex,ey), (ex+ew,ey+eh), (0,255,0), 2)
                cx = int(ex+ew/2)
                cy = int(ey+eh/2)
                cv2.circle(eyes_color, (cx,cy), radius=3, color=(0,0,255), thickness=-1)
                eye_count += 1
                eye_pts.append((cx,cy))
                if eye_count == 2:
                    break
            eye_count = 0

            if len(eye_pts) == 2:
                cv2.line(eyes_color, eye_pts[0], eye_pts[1], (255,0,255), 1)
                # Get eye width in pixels
                eye_width = euclidean_distance(eye_pts[0], eye_pts[1])
                real_distance = distance_from_camera(real_width, focal_length, eye_width)
                # Smooth the received distance values
                smooth_buffer.append(real_distance)
                if len(smooth_buffer) < smooth_limit:
                    smoothed = real_distance
                else:
                    smooth_buffer.pop(0)
                    smoothed = sum(smooth_buffer) / len(smooth_buffer)
                cv2.putText(image, str(int(real_distance))+"mm", (10,450), font, 2, (0, 255, 0), 2, cv2.LINE_AA)

        if claw_state:
            hands = palm_cascade.detectMultiScale(image_gray, 1.3, 5)
            cv2.putText(image, "Closed", (500,20), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            hands = fist_cascade.detectMultiScale(image_gray, 1.3, 5)
            cv2.putText(image, "Opened", (500,20), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        if len(hands) > 0:
            for (x, y, w, h) in hands:
                cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,255), 2)
                claw_counter += 1
                if claw_counter > 10:
                    if claw_state:
                        claw_state = False
                    else:
                        claw_state = True
                    claw_counter = 0
        else:
            if claw_counter > 0:
                claw_counter -= 1

    # If no face is detected
    else:
        if tracking_state:
            #pixels[0] = (0,255,0)
            tracking_state = False

    cv2.imshow('img',image)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break


# Clean up
#cam.stop()
cam.release()
cv2.destroyAllWindows()
