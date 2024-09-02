#!/bin/bash
echo "Running the command: $@"
cd /opt/data/BadgerRLSystem
$("$@")
# export SCRATH_DIR=env | grep -i scratch
# $("$@")
# Moving data file to root directory # 
# mv "Config/
