import numpy as np
import os
import subprocess
import argparse

BASE_RUN_CMD = "python run_eval.py"


# Parser to process arguments # 
def get_parser():
    parser = argparse.ArgumentParser(description='Run a job on the CHTC')
    parser.add_argument('--num-jobs', type=int, required=True, help='The output directory')
    parser.add_argument('--type', type=str, default='random', help='The type of job to run')
    parser.add_argument('--iter', type=int, default=1, help='The number of iterations to run')
    parser.add_argument('--run-name', type=str, default='test', help='The name of the run')
    parser.add_argument('--interactive', action='store_true', help='Run the docker image interactively')
    return parser

# Command List to execute # 
def generate_cmd_list(args):
    cmd_list = []
    for i in range(args.num_jobs):
        cmd_list.append(BASE_RUN_CMD + f" --type={args.type}" + f" --iter={args.iter}")

    return cmd_list

def write_cmds_to_file(run_name):
    with open(f"{run_name}.txt" , "w") as f:
        for cmd in cmd_list:
            f.write(f"{cmd}\n")


if __name__ == '__main__':
    parser = get_parser()
    args = parser.parse_args()
    
    cmd_list = generate_cmd_list(args)
    write_cmds_to_file(args.run_name)

    # Run an interactive job to see if everything works as expected # 
    if not args.interactive:
        run_cmd = f'condor_submit run_image.sub commands_file={args.run_name}.txt'
    else:
        run_cmd = f'condor_submit -i run_image.sub commands_file={args.run_name}.txt'
    
    os.system(run_cmd)
    