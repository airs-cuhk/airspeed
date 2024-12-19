# Real-World Teleoperation

## Prerequisits
### Software
  * Ubuntu 22.04
  * ROS Humble
  * python 3.10
### Motion capture device
  * Noitom HybridDataServer
### Robotic arm
  * Elephant Robotics mycobot pro 630


## Modules
* airspeed_converter: the data conversion module, converting teleoperation data into posture data
* airspeed_robot_interface: the robot interface module
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

3. Install Elephant Robotics
* Download and install the mycobot pro 630 SDK from [Elephant Robotics](https://docs.elephantrobotics.com/docs/pro630-cn/6-SDKDevelopment/python/PyhtonAPI.html)
* install the mycobot pro 630 requirements to the AIRSPEED package
```
cp -r ${DIR_AIRSPEED}/src/utils/robots/Elephant ${DIR_ELEPHANT}/src/utils/robots/Elephant_pro

cd ${DIR_ELEPHANT}/src/utils/robots/Elephant_pro
pip install -r requirements.txt
```


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

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_converter --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_robot_interface --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select airspeed_data_collection --symlink-install
```

## Run
1. Noitom HybridDataServer
```
cd ${DIR_AIRSPEED}/src/utils/devices/Noitom
./mocapapi_server
```
2. Elephant Robot
```
cd ${DIR_ELEPHANT}/src/utils/robots/Elephant_pro
python receiver.py
```
3. airspeed
* airspeed_converter
```
cd ${DIR_AIRSPEED}
source install/local_setup.bash
ros2 launch airspeed_converter run_airspeed_converter.launch.py
```
* airspeed_robot_interface
```
cd ${DIR_AIRSPEED}
source install/local_setup.bash
ros2 launch airspeed_robot_interface run_airspeed_robot_interface.launch.py
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
