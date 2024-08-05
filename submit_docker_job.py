import argparse
import os


## Parser for command line arguments ### 
def get_parser():
    parser = argparse.ArgumentParser(description='Submit a docker job to CHTC')
    parser.add_argument('--install', action='store_true', help='Install the docker image')
    parser.add_argument('--interactive', action='store_true', help='Run the docker image interactively')
    args = parser.parse_args()
    return args


### Main function to submit the job ###
if __name__ == "__main__": 
    args = get_parser()
    if args.install:
        submit_file = "install_from_scratch.sub"
    else:
        submit_file = "run_built_image.sub"
    
    print("Compressing files...")
    os.system(
        "tar -czf BadgerRLSystem.tar.gz BadgerRLSystem"
    )
    
    if not args.interactive:
        run_cmd = "condor_submit " + submit_file
    else: 
        run_cmd = "condor_submit -i " + submit_file
    
    os.system(
        f"condor_submit {run_cmd}"
    )
    print(f"Submitted job with command: {run_cmd}")