# TIAGo ROXANNE

## Installation

### TIAGo Workspace
To have a system up and running for **TIAGo**, follow the instruction in this [tutorial](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS).

Once completed there's a workspace named ``tiago_public_ws`` ready for TIAGo simulation, using **ROS Melodic** and **Ubuntu 18.04**.

### ROXANNE Workspace
As with TIAGo you need to prepare the **ROXANNE** workspace, you can do this by following the git instructions of [roxanne_rosjava](https://github.com/pstlab/roxanne_rosjava/blob/master/README.md).

At the end of the configuraiton there's a workspace named ``ws`` and it should be possible to run ROXANNE and send and execute **planning requests**.

### roxanne_rosjava_msgs Package
Since tiago_roxanne uses **ROXANNE messages** to communicate with the planner it's necessary that it can access the **roxanne_rosjava_msg** package, to do so, copy the folder corresponding to the package inside the ``tiago_public_ws`` folder.

In the ``CMakeLists.txt`` file, comment, with ``#``, or remove the line ``catkin_rosjava_setup()``.

Then build the package from ``tiago_public_ws`` with ``catkin build roxanne_rosjava_msgs``.

### tiago_roxanne Package
At this point configure the **tiago_roxanne** package in the ``tiago_public_ws`` workspace by cloning this repository.
```
cd ~/tiago_public_ws/src
mkdir tiago_roxanne
cd tiago_roxanne
git clone https://github.com/Xhst/tiago_roxanne.git
```
And then by building the package.
```
cd ~/tiago_public_ws/
catkin build tiago_roxanne
```

## Simulation
To run the simulation execute:
```
~/tiago_public_ws/src/tiago_roxanne/bash/setup.bash
```