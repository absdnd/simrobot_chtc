import numpy as np
import os
import sys
import argparse

BASE_RUN_CMD = "xvfb-run -a Build/Linux/SimRobot/Develop/SimRobot"
# Config files to execute the simulation
config_files = []
for i in range(100):
    config_files.append(f"Config/Scenes/config_{i}.ros2")


breakpoint()
'''
Getting parser from argparse
- run-name: Name of the run
- interactive: Whether to run the job interactively
- image: Name of the image to run
'''
def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-name', default="", type=str)
    parser.add_argument('--interactive', default=False, type=bool)
    parser.add_argument('--image', default="simrobot.sif", type=str)
    return parser

if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()
    if args.run_name == "":
        args.run_name = "simrobot_test"

    cmd_list = []
    for config_file in config_files:
        full_cmd = BASE_RUN_CMD + f" {config_file}"
        cmd_list.append(full_cmd)
    
    with open("data/jobs/" + args.run_name + ".txt" , "w") as f:
        for cmd in cmd_list:
            f.write(f"{cmd}\n")

    # Full command to run
    full_cmd = ""
    if args.interactive:
        full_cmd = f"condor_submit -i exe/run_image.sub run_name={args.run_name} commands_file=data/jobs/{args.run_name}.txt image={args.image}"
    else:
        full_cmd = f"condor_submit exe/run_image.sub run_name={args.run_name} commands_file=data/jobs/{args.run_name}.txt image={args.image}"

    os.system(full_cmd)