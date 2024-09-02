''''
A script to average results of a specific run executed on CHTC
'''

import os
import argparse
import sys


BASE_RESULT_DIR = "data/results"
def extract_run_results (data_files, format=["success", "time"]):
    success_list,time_list = [], []
    for file in data_files: 
        with open(f"{BASE_RESULT_DIR}/{file}", "r") as f:
            lines = f.readlines()

        lines = [line.strip().split() for line in lines]
        success_list.extend([float(line[0]) for line in lines])
        time_list.extend([float(line[1]) for line in lines])

    return success_list, time_list    
            
if __name__ == "__main__":
    run_name = sys.argv[1]
    files = os.listdir(BASE_RESULT_DIR)
    files = [f for f in files if f.startswith(run_name)]

    success_list, time_list = extract_run_results(files)
    print("Execution results for: ", run_name)
    print(f"Success rate: {sum(success_list)/len(success_list)}")
    print(f"Average time: {sum(time_list)/len(time_list)}")