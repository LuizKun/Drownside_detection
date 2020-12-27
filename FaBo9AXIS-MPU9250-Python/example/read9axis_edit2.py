# coding: utf-8
import FaBo9Axis_MPU9250
import time
import sys
import RPi.GPIO as GPIO
import subprocess
import array
import math
import threading
#day la doan code de do su chuyen dong
#==========define varriables==========
number_sampling =100 #====define number of sampling impliments=============
debug=0
Datax=[0]
Datay=[0]
Dataz=[0]
p1 = [0,0,0]
p2 = [0,0,0]
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
detect_mode=0
old_status=0
count=0
count_tmp=0
min_velocity=2 #=== 3m/s define velocity
stop_count=0
#========define velocity values============
Vo=0
V=0
def pin_int():
  global speaker,led_system,led_status,led_status,chair_mode1,chair_mode2
  speaker=11
  led_system=33
  led_status=13
  chair_mode1=29
  chair_mode2=31
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(speaker,GPIO.OUT, initial = 0)
  GPIO.setup(led_system,GPIO.OUT, initial = 0)
  GPIO.setup(led_status,GPIO.OUT, initial = 0)
  GPIO.setup(chair_mode1,GPIO.OUT, initial = 0)
  GPIO.setup(chair_mode2,GPIO.OUT, initial = 0)
  GPIO.output(led_status,GPIO.HIGH)
def Average(lst):
  return sum(lst)/len(lst)
def get_accel_sampling():
    #=======clear array============
    global accelx_aver
    global accely_aver
    global accelz_aver
    Datax=[]
    Datay=[]
    Dataz=[]
    #=====================
    for count in range (0,100):
      accel = mpu9250.readAccel()
      if debug==1:
        print ("ax=", ( accel['x'] ))
        print ("ay=", ( accel['y'] ))
        print ("az=", ( accel['z'] ))
      Datax.append(accel['x'])
      Datay.append(accel['y'])
      Dataz.append(accel['z'])
      time.sleep(0.05)
    print("\n") 
    accelx_aver=Average(Datax)
    accely_aver=Average(Datay)
    accelz_aver=Average(Dataz)
##        accelx_aver=np.average(Datax)
    if debug==1:
      print ("Datax: ",(accelx_aver))
      print ("Datay: ",(accely_aver))
      print ("Dataz: ",(accelz_aver))
    print("\n")
#==================define threading function===================
def drownside_detec(mode):
    if (mode == 1):
      print ("===================Triggered drownside detect========")
      subprocess.call(['/home/pi/Desktop/FaBo9AXIS-MPU9250-Python/example/check_detect_mode_backup.sh'])
    if (mode ==2):
      print ("===================Disable drownside detect========")
      subprocess.call(['/home/pi/Desktop/FaBo9AXIS-MPU9250-Python/example/dis_drownside_detect.sh'])
    #return
#================Init Alert system=====================    
pin_int()
for i in range(15):
  time.sleep(0.5)
  GPIO.output(led_status,GPIO.LOW)
  time.sleep(0.5)
  GPIO.output(led_status,GPIO.HIGH)
#================Init Alert system=====================
try:
    while True:
      get_accel_sampling ()
      if debug==1 :
        print ("=============>Insert data to P2===============> ")
      p2 =[accelx_aver,accely_aver,accelz_aver]
      if debug==1:
        print ("Datax p2[0]: ",p2[0])
        print ("Datay p2[1]: ",p2[1])
        print ("Dataz p2[2]: ",p2[2])
      real_accel = math.sqrt( math.pow(p2[0]-p1[0],2) + math.pow(p2[1]-p1[1],2) + math.pow(p2[2]-p1[2],2) )  #calculate real accel
      print (" Real Accel is : ",real_accel)
      p1 =[accelx_aver,accely_aver,accelz_aver]
      V=Vo + real_accel * (number_sampling * 0.05) #==== V=Vo*at======
      print (" Real Velocity is : ",V)
    #======================= actions======================
      if (V >= min_velocity):
        detect_mode=1
        if (old_status != detect_mode):
	              #count_tmp=count_tmp+1
	              #if (count_tmp >1):
          print ("================Object is moving ==============")
          GPIO.output(led_status,GPIO.LOW)
          old_status =1
          print ("================Detect mode:",(detect_mode))
          x = threading.Thread(target=drownside_detec, args=[1,])
          x.start()
          time.sleep(10)
      else: 
          if (detect_mode==1):
            stop_count= stop_count+1;
            print ("================stop_count:",(stop_count))
            if ((stop_count >= 9)and(detect_mode==1)):
              print ("================Object is stopping ==============")
              detect_mode=0
              old_status=0
              GPIO.output(led_status,GPIO.HIGH)
              stop_count=0
              x = threading.Thread(target=drownside_detec, args=([2,]))
              x.start()
              time.sleep(2)
		#stop_count=0
      time.sleep(0.5)
##        accel = mpu9250.readAccel()
##        print "ax=", ( accel['x'] )
##        print "ay=", ( accel['y'] )
##        print "az=", ( accel['z'] )
##        ##        p2 =np.array([accel['x'],accel['y'],accel['z']])
##        squared_dist = np.sum((p1-p2)**2, axis=0)
##        dist = np.sqrt(squared_dist)
##        #gyro = mpu9250.readGyro()
##        #print " gx = " , ( gyro['x'] )
##        #print " gy = " , ( gyro['y'] )
##        #print " gz = " , ( gyro['z'] )
##
##        #mag = mpu9250.readMagnet()
##        #print " mx = " , ( mag['x'] )
##        #print " my = " , ( mag['y'] )
##        #print " mz = " , ( mag['z'] )
##        print "Distance ", dist
##        p1 =np.array([accel['x'],accel['y'],accel['z']])
##        time.sleep(5)

except KeyboardInterrupt:
  sys.exit()
