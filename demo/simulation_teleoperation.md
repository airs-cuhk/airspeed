# Real-World Teleoperation

## Prerequisits
### Software
  * Ubuntu 22.04
  * ROS Humble
  * python 3.10
### Motion capture device
  * Noitom HybridDataServer
### Simulator
  * Isaac sim

## Modules
* airspeed_converter: the data conversion module, converting teleoperation data into posture data
* airspeed_simulation_interface: the simulation interface module
* airspeed_data_collection: the data collection module
* airspeed_dataset_construction: the dataset construction module


## System Setup
1. Download AIRSPEED package from GitHub
```shell
# ${DIR_AIRSPEED} is the directory you create for AIRSPEED pakcage
mkdir -p ${DIR_AIRSPEED}/src

cd ${DIR_AIRSPEED}/src
git clone https://github.com/airs-admin/airspeed.git
```

2. Install ROS2 Humble
```
cd ${DIR_AIRSPEED}/src
bash script/software_setup/install_ros_humble.sh
```

3. Import usd file
* Open Isaac sim via “omnivores launcher”，drag the utils/simulation_platforms/Isaac_Sim/airspeed_teleop.usd file into the scene. Then, click the “play” button on the left hand side.



# Quick Starts

## Compile
* Noitom HybridDataServer
```
cd ${DIR_AIRSPEED}/src/utils/devices/Noitom
g++ -c mocapapi_server.cpp -I. -o mocapapi_server.o
g++ mocapapi_server.o -L. -lMocapApi -Wl,-rpath,. -o mocapapi_server
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
```

* airspeed
```
cd ${DIR_AIRSPEED}
source /opt/ros/humble/setup.bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_services --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_converter --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_simulation_interface --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_data_collection --symlink-install
```

## Run
1. Noitom HybridDataServer
```
cd ${DIR_AIRSPEED}/src/utils/devices/Noitom
./mocapapi_server
```
2. airspeed
* airspeed_converter
```
cd ${DIR_AIRSPEED}
source install/local_setup.bash
ros2 launch airspeed_converter run_airspeed_converter.launch.py
```
* airspeed_simulation_interface
```
cd ${DIR_AIRSPEED}
source install/local_setup.bash
ros2 run airspeed_simulation_interface airspeed_simulation_interface.py
```
* airspeed_data_collection
```
cd ${DIR_AIRSPEED}
source install/local_setup.bash
ros2 launch airspeed_data_collection run_airspeed_data_collection.launch.py
```
* airspeed_dataset_construction
```
python airspeed_dataset_construction.py
```

# Acknowledgement
* https://github.com/pnmocap/MocapApi
