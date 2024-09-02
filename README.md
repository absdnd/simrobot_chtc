
# A Setup to Run Simrobot remotely on Remote Cluster

## Installation

### Clone the repository
From the root directory, of the repository run the following command. First make sure you checkout to the desired branch and pull latest changes. You can run the following: 

`git clone --recursive https://github.com/Badger-RL/BadgerRLSystem.git`
`cd BadgerRLSystem`
`git checkout -b $BRANCH_NAME`
`tar -czf BadgerRLSystem.tar.gz BadgerRLSystem`


### Build the Image

Create a folder called `containers/` in the root repository to build the container. To build the image run the command `python build/run_build.py` use the argument `--interactive=True` to interactively build the container. Additionally, specifying `--prepare-data=True` pulls the latest changes and compiles it.

If running interactively, run the command `apptainer build simrobot.sif simrobot.def` to build the container. After the job runs, this will generate an image `simrobot.sif` which will be moved to `containers/simrobot.sif` after exiting the job. 

### Run Build Image

To run the built image, use the command `python exe/run_simrobot.py`. This currently runs a single configuration `ThreeRobots.ros2`. To run multiple config files create them manually or using a script in the directory `Config/Scenes/ThreeRobots.ros2`. Each config file will be assigned a separate run. The job commands would be a `.txt` files saved in the `data/jobs`. The output and logs would be specific to each config within the run and would be of the format `data/output/$RUN_NAME_$PROCESS_ID.out` and `data/logs/$RUN_NAME_$PROCESS_ID.log`. `$RUN_NAME` is the name assigned to the run on execution and `$PROCESS_ID` represents the job id being used. The command for execution is: 

`python exe/run_simrobot.py $RUN_NAME`


