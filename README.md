
# A Setup to Run Simrobot remotely on Remote Cluster

## Installation

### Clone the repository
From the root directory, of the repository run the following command. First make sure you checkout to the desired branch and pull latest changes. You can run the following: 

`git clone --recursive https://github.com/Badger-RL/BadgerRLSystem.git`
`cd BadgerRLSystem`
`git checkout -b $BRANCH_NAME`
`tar -czf BadgerRLSystem.tar.gz BadgerRLSystem`


### Build the Image

To build the image run the command `python build/run_build.py` use the argument `--interactive=True` to interactively build the container. Additionally, specifying `--prepare-data=True` pulls the latest changes and compiles it.

If running interactively, run the command `apptainer build simrobot.sif simrobot.def` to build the container. After the job runs, this will generate an image `simrobot.sif` which will be moved to `data/simrobot.sif` after exiting the job. 

### Run the Built Image 

The image of the job `simrobot.sif` is produced in the same directory as the job submission directory. We can now run the image using `run_image.sub`. This will use `xvfb-run` to execute Simrobot headlessly from the current directory. To run the code, specify the second argument in the run-name. 

`python exe/run_simrobot.py simrobot_test`


