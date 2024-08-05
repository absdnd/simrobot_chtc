#!/bin/bash
echo "Running the command: $@"
cd /tmp/BadgerRLSystem
$("$@")
# Moving data file to root directory # 
mv "Config/
