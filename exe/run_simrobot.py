import numpy as np
import os
import sys

BASE_RUN_CMD = "xvfb-run Build/Linux/SimRobot/Develop/SimRobot"
config_files = ["Config/Scenes/ThreeRobots.ros2"]


if __name__ == "__main__":

    run_name = sys.argv[1]    
    cmd_list = []
    for config_file in config_files:
        full_cmd = BASE_RUN_CMD + f" {config_file}"
        cmd_list.append(full_cmd)
    
    with open("data/jobs/" + run_name + ".txt" , "w") as f:
        for cmd in cmd_list:
            f.write(f"{cmd}\n")

    
    os.system(f'condor_submit -i exe/run_image.sub run_name={run_name} commands_file=data/jobs/{run_name}.txt')
