#!/bin/bash
PID=`ps -eaf | grep drownside_detec.sh | grep -v grep | awk '{print $2}'`
if [[ "" != "$PID" ]]; then
	echo "Downside detect mode running with PID: $PID"
	pidpython=`ps -eaf | grep pi_detect_drowsiness.py | grep -v grep | awk '{print $2}'`
	echo "Kill all Downside-detect process"
	sudo kill -9 $pidpython
	sudo kill -9 $PID
else
echo "No downside detect running"
fi
exit 0
