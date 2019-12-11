# AWS RoboMaker Sample Hamster Simulation

## Requirements

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html) - Used for building and bundling the application.

## Build

### Pre-build commands

```bash
sudo apt-get update
rosdep update
```

### Simulation

```bash
cd simulation_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run

Launch the simulation with the following commands:

```bash
source simulation_ws/install/local_setup.sh
roslaunch hamster_vehicle_gazebo hamster_multi_vehicle.launch
```
By default 2 hamsters appears, to change robot count - edit hamster_multi_vehicle.launch in order to instructions inside.

## Using this sample with RoboMaker

You first need to install colcon-ros-bundle. Python 3.5 or above is required.

```bash
pip3 install -U setuptools
pip3 install colcon-ros-bundle
```

After colcon-ros-bundle is installed you need to build your robot or simulation, then you can bundle with:

```bash

# Bundling Simulation Application
cd simulation_ws
source install/local_setup.sh
colcon bundle
```

This produces the artifacts `simulation_ws/bundle/output.tar` respectively.
You'll need to upload these to an s3 bucket, then you can use these files to
[create a simulation application](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html),
and [create a simulation job](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html) in RoboMaker.

