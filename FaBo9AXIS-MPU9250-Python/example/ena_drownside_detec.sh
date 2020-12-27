#!/bin/sh
#export DISPLAY=":0"
echo "=======================>Start detect mode<==========================="
python3 /home/pi/Desktop/FaBo9AXIS-MPU9250-Python/example/pi_detect_drowsiness.py -c /home/pi/Desktop/FaBo9AXIS-MPU9250-Python/example/haarcascade_frontalface_default.xml -p /home/pi/Desktop/FaBo9AXIS-MPU9250-Python/example/shape_predictor_68_face_landmarks.dat
