
# A submission setup to install and run the SimRobot docker image

## Installation
From your root directory, run the following command:
```condor_submit docker_submit.sub```

## Expected Output  
The output of the docker image will be a `trajectory.json.gz` file in the root folder. Unzip this file to get the trajectories.json file.

### Visualizing the Trajectory 
To visualize the trajectory uzip the tar file with the following command from the root directory:
```python3 visualize.py```