# import numpy as np
import argparse
import os

# Parser Argument # 
def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', default="data/BadgerRLSystem", type=str)
    parser.add_argument('--branch', default="RoboCup2024_container", type=str)
    parser.add_argument('--interactive', default=False, type=bool)
    parser.add_argument('--prepare-data', default=True, type=bool)
    return parser

def log(s):
    print(s)

if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()
    
    PWD = os.getcwd()


    '''
    Prepare the data by cd'ing into the folder and pulling latest changes
    - Checkout to Robocup2024 branch
    - Pull latest changes and compresss the folder
    '''

    if args.prepare_data:    
        print("Preparing data...")
        os.system(
            f"cd {args.folder} && git pull origin {args.branch} && git checkout {args.branch} && cd {PWD}"
        )

        os.system(
        f"tar -czf {args.folder}.tar.gz {args.folder}"
        )



    if args.interactive: 
        os.system(
            "condor_submit -i build/direct_build.sub"
        )
    else: 
        os.system(
            "condor_submit build/direct_build.sub"
        )

