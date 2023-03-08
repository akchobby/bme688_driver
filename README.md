# BME 688 driver

This repository provides a ROS 2 wrapper around the bme680 python library to publish gas sensor information on ROS.

# Pre-requisites

Ensure that you have the [bme680 library (forked version)]() installed in the machine follwing the steps under Hover Games 3.

# Installation

* Create a ROS 2 workspace,  steps for it can be found in [this link](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#create-a-new-directory)
* Clone the gas_sensor_msgs and this repo by the below commands

```bash
cd <path_to_ros2_workspace>/src
git clone git@github.com:akchobby/gas_sensor_msgs.git
git clone git@github.com:akchobby/bme688_driver.git
```

* Now the packages are ready to be built.

```bash
cd <path_to_ros2_workspace>/
source /opt/ros/<your_ROS2_Distro>/setup.bash
colcon build --symlink-install # allows python scripts to chaged without rebuild
```
* After the build source the pkg

```bash
source <path_to_ros2_workspace>/install/setup.bash
```
# Operation

The sensor setup can be modified in using the sensor_config.yaml file. To launch the driver, after sourcing the package
run :

```bash
ros2 run bme688_drive bme688_driver
```

Note after any chage to the config file don't forget to colcon build the package.



