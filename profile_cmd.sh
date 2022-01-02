#!/bin/bash
function publish 
{
	echo "rostopic publishing to /rtp/$1 ..."
	rostopic pub /rtp/$1 rtp/ProfileCmd "cmd: $2" -1
	echo "rostopic pub /rtp/$1 rtp/ProfileCmd \"cmd: $2\" -1"
}

if [ $1 == "start" ]; then
	echo "start profiling..."
	publish "profile_cmd" $1
elif [ $1 == "end" ]; then
	echo "end to profiling..."
	publish "profile_cmd" $1
fi

