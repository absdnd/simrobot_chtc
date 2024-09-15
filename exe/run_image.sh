#!/bin/bash
echo "Running the command: $@"
cd /opt/data/BadgerRLSystem
echo $PWD
xvfb-run -a Build/Linux/SimRobot/Develop/SimRobot Config/Scenes/1v1config_$1.ros2
# $("$@")
# Moving data file to root directory # 
# mv "Config/