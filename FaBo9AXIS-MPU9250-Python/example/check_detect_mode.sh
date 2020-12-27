#!/bin/bash
PID=`ps -eaf | grep drownside_detec.sh | grep -v grep | awk '{print $2}'`
if [ -n "$PID" ]
then
	echo "Downside detect mode running with PID: $PID"
	pidpython=`ps -eaf | grep pi_detect_drowsiness.py | grep -v grep | awk '{print $2}'`
	if [ -n "$pidofpython"]
	 then
		echo "Downside-detect process is running"
		echo "No more action required"
	fi
	
else
echo "No downside detect running"
./home/pi/Desktop/FaBo9AXIS-MPU9250-Python/example/ena_drownside_detec.sh
fi
exit 0
