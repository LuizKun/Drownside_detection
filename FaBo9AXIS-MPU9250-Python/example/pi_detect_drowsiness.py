# USAGE
# python pi_detect_drowsiness.py --cascade haarcascade_frontalface_default.xml --shape-predictor shape_predictor_68_face_landmarks.dat
# python pi_detect_drowsiness.py --cascade haarcascade_frontalface_default.xml --shape-predictor shape_predictor_68_face_landmarks.dat --alarm 1

# import the necessary packages
import sys
sys.path.insert(0, "/usr/local/lib/python3.5/dist-packages")
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import VideoStream
from imutils import face_utils
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2
import RPi.GPIO as GPIO
import time
import threading

def pin_int():
	global speaker,led_system,led_status,chair_mode1,chair_mode2
	speaker=11
	led_system=33
	led_status=13
	chair_mode1=29
	chair_mode2=31
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(speaker,GPIO.OUT, initial = 0)
	GPIO.setup(led_system,GPIO.OUT, initial = 0)
	GPIO.setup(led_status,GPIO.OUT, initial = 1)
	GPIO.setup(chair_mode1,GPIO.OUT, initial = 0)
	GPIO.setup(chair_mode2,GPIO.OUT, initial = 0)
#==========Define actions function==========================
global count
global x,y,detect_mode,old_status
count=0
detect_mode=0
old_status=0
def led_blinking(count,pulse):
	for i in range(count):
		time.sleep(pulse)
		GPIO.output(led_system,GPIO.HIGH)
		time.sleep(pulse)
		GPIO.output(led_system,GPIO.LOW)
def status_blinking():
	#detect_mode=0
	#if (detect_mode==0):
	time.sleep(0.5)
	GPIO.output(led_status,GPIO.LOW)
	time.sleep(0.5)
	GPIO.output(led_status,GPIO.HIGH)
def speaker_alert(count,pulse):
	for i in range(count):
		time.sleep(pulse)
		GPIO.output(speaker,GPIO.HIGH)
		time.sleep(pulse+0.3)
		GPIO.output(speaker,GPIO.LOW)
def led_blinking(count,pulse):
	for i in range(count):
		time.sleep(pulse)
		GPIO.output(led_system,GPIO.HIGH)
		time.sleep(pulse)
		GPIO.output(led_system,GPIO.LOW)



def drownside_alert():
	global old_status
	print ("=========Starting Alert System=============")
	while True:
		if (old_status == 1):
			print ( "=Downside detected==" )
			print ( "======>Triggered Speaker==")
			GPIO.output(led_status,GPIO.LOW)
			GPIO.output(chair_mode2,GPIO.HIGH)
			GPIO.output(chair_mode1,GPIO.LOW)
			for i in range(3):
				time.sleep(0.2)
				GPIO.output(speaker,GPIO.HIGH)
				GPIO.output(led_system,GPIO.HIGH)
				time.sleep(0.2+0.3)
				GPIO.output(led_system,GPIO.LOW)
				GPIO.output(speaker,GPIO.LOW)
			GPIO.output(chair_mode2,GPIO.LOW)
		#time.sleep(5)
		if (old_status == 2):
                	print ( "==== No Downside detected ========" )
               		print ( "==== >Alert System Stopped" )
                	GPIO.output(speaker,GPIO.LOW)
                	GPIO.output(led_system,GPIO.LOW)
                	GPIO.output(led_status,GPIO.HIGH)
                	GPIO.output(chair_mode1,GPIO.LOW)
                	GPIO.output(chair_mode2,GPIO.LOW)
		if (old_status==0):
			status_blinking()

#===========================================================
def euclidean_dist(ptA, ptB):
	# compute and return the euclidean distance between the two
	# points
	return np.linalg.norm(ptA - ptB)

def eye_aspect_ratio(eye):
	# compute the euclidean distances between the two sets of
	# vertical eye landmarks (x, y)-coordinates
	A = euclidean_dist(eye[1], eye[5])
	B = euclidean_dist(eye[2], eye[4])

	# compute the euclidean distance between the horizontal
	# eye landmark (x, y)-coordinates
	C = euclidean_dist(eye[0], eye[3])

	# compute the eye aspect ratio
	ear = (A + B) / (2.0 * C)

	# return the eye aspect ratio
	return ear
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--cascade", required=True,
	help = "path to where the face cascade resides")
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
ap.add_argument("-a", "--alarm", type=int, default=0,
	help="boolean used to indicate if TraffHat should be used")
args = vars(ap.parse_args())

# check to see if we are using GPIO/TrafficHat as an alarm
#if args["alarm"] > 0:
#	from gpiozero import TrafficHat
#	th = TrafficHat()
#	print("[INFO] using TrafficHat alarm...")
 
# define two constants, one for the eye aspect ratio to indicate
# blink and then a second constant for the number of consecutive
# frames the eye must be below the threshold for to set off the
# alarm
EYE_AR_THRESH = 0.3  # do nhay cua mat , mat nho thi tang len mat to thi giam xuong tu 0.2 den 0.33
EYE_AR_CONSEC_FRAMES = 4 # so lan do mat nham, muon nhanh thi giam ve 2 muon cham thi tang len 8 -10  
# xong nho save file roi ra desktop chay

# initialize the frame counter as well as a boolean used to
# indicate if the alarm is going off
global COUNTER,ALARM_ON,status_ret
COUNTER=0
ALARM_ON= False
status_ret=0
# load OpenCV's Haar cascade for face detection (which is faster than
# dlib's built-in HOG detector, but less accurate), then create the
# facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = cv2.CascadeClassifier(args["cascade"])
predictor = dlib.shape_predictor(args["shape_predictor"])

# grab the indexes of the facial landmarks for the left and
# right eye, respectively
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera()
#camera.resolution = (640, 480)
#camera.framerate = 32
#vs= PiRGBArray(camera, size=(640, 480))
#=============intit GPIO PIN================
pin_int()
print("[INFO] GPIO Init...")
x = threading.Thread(target=drownside_alert)
x.start()
#===========================================
# start the video stream thread
#==============================================
print("[INFO] starting video stream thread...")
#vs = VideoStream(src=0).start()
vs = VideoStream(usePiCamera=True,resolution=(640,480),framerate=32).start()
#vs= PiRGBArray(camera, size=(640, 480))
time.sleep(1.0)

# loop over frames from the video stream
while True:
	# grab the frame from the threaded video file stream, resize
	# it, and convert it to grayscale
	# channels)
	
	frame = vs.read()
	frame = imutils.resize(frame, width=450)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the grayscale frame
	rects = detector.detectMultiScale(gray, scaleFactor=1.1, 
		minNeighbors=5, minSize=(30, 30),
		flags=cv2.CASCADE_SCALE_IMAGE)

	# loop over the face detections
	for (x, y, w, h) in rects:
		# construct a dlib rectangle object from the Haar cascade
		# bounding box
		rect = dlib.rectangle(int(x), int(y), int(x + w),
			int(y + h))

		# determine the facial landmarks for the face region, then
		# convert the facial landmark (x, y)-coordinates to a NumPy
		# array
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)

		# extract the left and right eye coordinates, then use the
		# coordinates to compute the eye aspect ratio for both eyes
		leftEye = shape[lStart:lEnd]
		rightEye = shape[rStart:rEnd]
		leftEAR = eye_aspect_ratio(leftEye)
		rightEAR = eye_aspect_ratio(rightEye)

		# average the eye aspect ratio together for both eyes
		ear = (leftEAR + rightEAR) / 2.0

		# compute the convex hull for the left and right eye, then
		# visualize each of the eyes
		leftEyeHull = cv2.convexHull(leftEye)
		rightEyeHull = cv2.convexHull(rightEye)
		cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
		cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

		# check to see if the eye aspect ratio is below the blink
		# threshold, and if so, increment the blink frame counter
		if ear < EYE_AR_THRESH:
			COUNTER += 1

			# if the eyes were closed for a sufficient number of
			# frames, then sound the alarm
			if COUNTER >= EYE_AR_CONSEC_FRAMES:
				# if the alarm is not on, turn it on
#				if not ALARM_ON:
#					ALARM_ON = True
#
#					# check to see if the TrafficHat buzzer should
#					# be sounded
#					if args["alarm"] > 0:
#						th.buzzer.blink(0.1, 0.1, 10,
#							background=True)
				# draw an alarm on the frame
				#ALARM_ON = True
				detect_mode=1
				status_ret=1
				cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),
				cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
				if (detect_mode==1):
					if (detect_mode != old_status):
						old_status=detect_mode
						#x = threading.Thread(target=drownside_alert, args=[1,0,])
						#x.start()
						#COUNTER = 0
		# otherwise, the eye aspect ratio is not below the blink
		# threshold, so reset the counter and alarm
		else:
			COUNTER = 0
			if (detect_mode==1):
				#ALARM_ON = False
				detect_mode=3
				old_status=0
				#x = threading.Thread(target=drownside_alert, args=[2,0,])
				#x.start()
				detect_mode=0
		# draw the computed eye aspect ratio on the frame to help
		# with debugging and setting the correct eye aspect ratio
		# thresholds and frame counter
		cv2.putText(frame, "EAR: {:.3f}".format(ear), (300, 30),
		cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
	# show the frame
	cv2.imshow("Frame", frame)	
	#if (detect_mode==0):
		#status_blinking()
	#GPIO.output(led_status,GPIO.LOW)
	#time.sleep(0.5)
	key = cv2.waitKey(1) & 0xFF
	#GPIO.output(led_status,GPIO.HIGH)
	# if the `q` key was pressed, break from the loop
	#if key == ord("q"):
	#	break

# do a bit of cleanup
#cv2.destroyAllWindows()
vs.stop()

