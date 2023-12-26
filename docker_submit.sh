#!/bin/bash

git clone --recursive "https://github.com/Badger-RL/BadgerRLSystemCodeRelease2023.git"
cd BadgerRLSystemCodeRelease2023 && mkdir cache
cp -r ../NeuralControl.cpp Src/Modules/BehaviorControl/SkillBehaviorControl/Skills/Walk/NeuralControl.cpp
export CCACHE_DIR="$PWD/cache"
Make/Linux/generate
Make/Linux/compile Nao
Make/Linux/compile bush
Make/Linux/compile SimRobot
mkdir -p "data/trajectories"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/"
export QTDIR="/usr/lib/x86_64-linux-gnu/"
echo -e "gc ready\ngc set\ngc playing" > Config/Scenes/OneRobotFast.con
xvfb-run Build/Linux/SimRobot/Develop/SimRobot Config/Scenes/OneRobotFast.ros2
tar -czf trajectory.tar.gz "data/trajectories/"