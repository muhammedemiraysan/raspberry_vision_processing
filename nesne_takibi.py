from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import imutils
 
defaultSpeed = 50
windowCenter = 320
centerBuffer = 30
pwmBound = float(50)
cameraBound = float(320)
kp = pwmBound / cameraBound
leftBound = int(windowCenter - centerBuffer)
rightBound = int(windowCenter + centerBuffer)
error = 0
ballPixel = 0
 
#GPIO
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
#Pin definitions
rightFwd = 35
rightRev = 37
leftFwd = 40#  31 di
leftRev = 33
 
#GPIO initialization
GPIO.setup(leftFwd, GPIO.OUT)
GPIO.setup(leftRev, GPIO.OUT)
GPIO.setup(rightFwd, GPIO.OUT)
GPIO.setup(rightRev, GPIO.OUT)
 
#Disable movement at startup
GPIO.output(leftFwd, False)
GPIO.output(leftRev, False)
GPIO.output(rightFwd, False)
GPIO.output(rightRev, False)
 
#PWM Initialization
 
rightMotorFwd = GPIO.PWM(rightFwd, 50)
leftMotorFwd = GPIO.PWM(leftFwd, 50)
rightMotorRev = GPIO.PWM(rightRev, 50)
leftMotorRev = GPIO.PWM(leftRev, 50)
rightMotorFwd.start(defaultSpeed)
leftMotorFwd.start(defaultSpeed)
leftMotorRev.start(defaultSpeed)
rightMotorRev.start(defaultSpeed)
def updatePwm(rightPwm, leftPwm):
rightMotorFwd.ChangeDutyCycle(rightPwm)
leftMotorFwd.ChangeDutyCycle(leftPwm)
 
def pwmStop():
rightMotorFwd.ChangeDutyCycle(0)
rightMotorRev.ChangeDutyCycle(0)
leftMotorFwd.ChangeDutyCycle(0)
leftMotorRev.ChangeDutyCycle(0)
 
#Camera setup
camera = PiCamera()
camera.resolution = (640, 480) #640 480 idi
camera.framerate = 40 #15 di
rawCapture = PiRGBArray(camera, size = (640, 480))  #640 480 idi
 
time.sleep(0.1)
 
lower_yellow = np.array([35, 86, 6])  # 13 50 0 turuncu 29 86 6 yesil #29 yerine 60 iyi
upper_yellow = np.array([64, 255, 255]) # 30 255 255 turuncu 64 255 255 yesil
 
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
 
image = frame.array
output = image.copy()
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
mask = cv2.erode(mask, None, iterations=2)
mask = cv2.dilate(mask, None, iterations=2)
output = cv2.bitwise_and(output, output, mask=mask)
#output = cv2.dilate(output, None, iterations=2)
#output = cv2.erode(output, None, iterations=2)
gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
#_, binary = cv2.threshold(gray, 1, 255, cv2. THRESH_BINARY)
circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 3, 500, minRadius = 10, maxRadius = 200, param1 = 100,  param2 = 60)
#cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
ballPixel = 0
 
if circles is not None:
circles = np.round(circles[0, :]).astype("int")
for (x, y, radius) in circles:
 
cv2.circle(output, (x, y), radius, (0, 255, 0), 4)
#cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 
if radius > 10:
ballPixel = x
else:
ballPixel = 0
 
#cv2.imshow("output", output)
#cv2.namedWindow("window")
key = cv2.waitKey(1) & 0xFF
rawCapture.truncate(0)
 
 
if ballPixel == 0:
print "top yok"
error = 0
pwmStop()
elif (ballPixel < leftBound) or (ballPixel > rightBound):
error = windowCenter - ballPixel
pwmOut = abs(error * kp)
 
turnPwm = pwmOut + defaultSpeed
if  ballPixel < (leftBound):
 
if radius > 50 and ballPixel < 110:
print "sol1"
 
updatePwm(50, 25)
else:
updatePwm(turnPwm, 25)
print "sol2"
elif ballPixel > (rightBound):
 
if radius > 50 and ballPixel > 540:
 
print "sag11"
updatePwm(25, defaultSpeed)
else:
updatePwm(50, turnPwm)
print "sag2"
else:
print "orta"
if (radius < 40):
updatePwm(defaultSpeed, defaultSpeed)
else:
pwmStop()
 
if key == ord('q'):
break
 
cv2.destroyAllWindows()
camera.close()
pwmStop()
GPIO.cleanup()
