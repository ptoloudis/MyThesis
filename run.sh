#!/bin/bash

cd JSON
# rm -rf Gazebo
rm -rf unity

# mkdir Gazebo
mkdir unity

cd ..

sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --osd --add-param-file=param.param

echo "Finish"
