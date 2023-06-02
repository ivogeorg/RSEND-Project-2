# RSEND Project 2: Ball Chaser

<img src="/assets/Project-2-no-rviz-robot-model.png" width="450"/>

*Note: This repository is to be cloned as `src` under `/home/workspace/catkin_ws`, after which in `/home/workspace/catkin_ws/src` the command `catkin_init_workspace` can be run.*

## Directory structure
```
.RSEND-Project-1
|-- assets
|-- ball_chaser
|-- |-- CMakeLists.txt
|-- |-- include
|-- |-- |-- ball_chaser
|-- |-- launch
|-- |-- |-- ball_chaser.launch
|-- |-- package.xml
|-- |-- src
|-- |-- |-- drive_bot.cpp
|-- |-- |-- process_image.cpp
|-- |-- srv
|-- |-- |-- DriveToTarget.srv
|-- my_robot
|-- |-- CMakeLists.txt
|-- |-- external_models
|-- |-- |-- ...
|-- |-- launch
|-- |-- |-- robot_description.launch
|-- |-- |-- world.launch
|-- |-- meshes
|-- |-- |-- hokuyo.dae
|-- |-- package.xml
|-- |-- urdf
|-- |-- |-- my_robot.gazebo
|-- |-- |-- my_robot.xacro
|-- |-- worlds
|-- |-- |-- OfficeSpace.world
|-- README.md
```
1. The directory `assets` contains screenshots.
2. The directory `external-models` contains models for the OfficeSpace world.

## Setup
1. To visualize the full OfficeSpace world, `export GAZEBO_MODEL_PATH=/home/workspace/catkin_ws/src/my_robot/external_models:
$GAZEBO_MODEL_PATH` needs to be added to the `/home/workspace/.student_bashrc` file.
2. There is no `.gitignore` as all the catkin workspace setup is above the level of this repository in the directory tree:
   ```
   /home/workspace/catkin_ws
                   |-- src   <-- Clone of this repository
   ```
3. To clarify the point above, here are the steps to get this project working:
   1. `cd /home/workspace`.
   2. `mkdir catkin_ws`.
   3. `cd catkin_ws`.
   4. `git clone https://github.com/ivogeorg/RSEND-Project-2.git src`.
   5. `cd src`.
   6. `catking_init_workspace`.
