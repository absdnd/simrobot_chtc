
# A Setup to Run Simrobot remotely on Remote Cluster

## Installation

### Clone the repository
From the root directory, of the repository run the following command. First make sure you checkout to the desired branch and pull latest changes: `bash prepare.sh $BRANCH_NAME`. Alternatively, you can run the following: 

`git clone --recursive https://github.com/Badger-RL/BadgerRLSystem.git`
`cd BadgerRLSystem`
`git checkout -b $BRANCH_NAME`
`tar -czf BadgerRLSystem.tar.gz BadgerRLSystem`


### Build the Image
```condor_submit -i build.sub```

This will create a build job for building simrobot. To build the container execute, 

`apptainer build simrobot.sif simrobot.def`


This will create a container called `simrobot.sif` from the definition file `simrobot.def`. You can move this file to `/staging/` or use it directly for the next step. 

### Run the Built Image 

The image of the job `simrobot.sif` is produced in the same directory as the job submission directory. We can now run the image using `run_image.sub`. This will use `xvfb-run` to execute Simrobot headlessly from the current directory. 

```condor_submit run_image.sub```


