import cv2
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import serial
from optparse import OptionParser
import math
import datetime
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
servo1=GPIO.PWM(11,50)
servo2=GPIO.PWM(13,50)
servo1N=45
servo2N=40
def angle(dutyCycle):
	_angle=float(dutyCycle)/10+2.5
	return _angle

pos1=angle(servo1N)
pos2=angle(servo1N)

faceCascade = cv2.CascadeClassifier("haarcascade_frontalface.xml")
profileCascade = cv2.CascadeClassifier("haarcascade_profileface.xml")
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(320, 240))
screenx=160
screeny=120
servo1.start(angle(servo1N))
servo2.start(angle(servo2N))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
        pt1 = (int(x), int(y))
        pt2 = (int((x + w)), int((y + h)))
        x1 = pt1[0]
        x2 = pt2[0]
        y1 = pt1[1]
        y2 = pt2[1]
        normal=pt2
        centrex = x1+((x2-x1)/2)
        centrey = y1+((y2-y1)/2)
        centre = (centrex, centrey)
        print centre
        if(pos1>0 and pos2>0):
			if(centrex<screenx and centrey<screeny):
				move1=pos1 + .2
				servo1.start(move1)
				time.sleep(.025)
				pos1=move1
				move2=pos2 - .2
				servo2.start(move2)
				time.sleep(.025)
				pos2=move2
			if(centrex>screenx and centrey>screeny):
				move1=pos1 - .2
				servo1.start(move1)
				time.sleep(.025)
				pos1=move1
				move2=pos2 + .2
				servo2.start(move2)
				time.sleep(.025)
				pos2=move2
			if(centrex<screenx and centrey>screeny):
				move1=pos1 + .2
				servo1.start(move1)
				time.sleep(.025)
				pos1=move1
				move2=pos2 + .2
				servo2.start(move2)
				time.sleep(.025)
				pos2=move2
			if(centrex>screenx and centrey<screeny):
				move1=pos1 - .2
				servo1.start(move1)
				time.sleep(.025)
				pos1=move1
				move2=pos2 - .2
				servo2.start(move2)
				time.sleep(.025)
				pos2=move2
				
    cv2.imshow('Video', image)
    cv2.moveWindow('Video' ,0, 0)
    rawCapture.truncate(0)  

    if cv2.waitKey(1) & 0xFF == ord('q'):
		GPIO.cleanup()
		break

cv2.destroyAllWindows()
