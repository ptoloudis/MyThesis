#!/bin/bash

# Get the PID of the MyDrone process
PID=$(ps aux | grep '[M]yDrone' | awk '{print $2}')

# Check if PID is not empty
if [ -n "$PID" ]; then
    echo "Killing process with PID: $PID"
    kill $PID
    if [ $? -eq 0 ]; then
        echo "Process $PID has been terminated."
    else
        echo "Failed to kill process $PID."
    fi
else
    echo "No MyDrone process found."
fi

