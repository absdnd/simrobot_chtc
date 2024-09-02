#!/bin/bash
echo "Running the command: $@"
cd /opt/data/BadgerRLSystem
export SCRATH_DIR=env | grep -i scratch
xvfb-run Build/Linux/SimRobot/Develop/SimRobot Config/Scenes/ThreeRobots.ros2
# $("$@")
# Moving data file to root directory # 
# mv "Config/
