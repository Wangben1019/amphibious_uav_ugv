# 项目-两栖

## 系统架构

```bash
├── 3DLidar
│   ├── FAST_LIO
│   ├── livox_ros_driver
│   └── livox_ros_driver2
├── mavros_utils
│   ├── mavlink
│   └── mavros
├── system_bringup
│   ├── CMakeLists.txt
│   ├── config
│   ├── launch
│   ├── package.xml
│   ├── rviz
│   └── src
├── uav_utils
│   ├── control_tuner
│   ├── local_planner
│   ├── vehicle_simulator
│   ├── waypoint_example
│   └── waypoint_rviz_plugin
└── ugv_utils
    ├── joystick_drivers
    ├── navigation
    ├── slam_gmapping
    └── user_package
```

## Install

### Install ros

**鱼香ros一键安装**

```
wget http://fishros.com/install -O fishros && . fishros
```

### Install mavros

[官网文档](https://docs.px4.io/main/zh/ros/mavros_installation.html)

**直接安装mavros以安装好依赖，用于编译**

```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

./install_geographiclib_datasets.sh
```

### Install other prerequisites

#### 方法1：

 ```bash
 sudo apt install libusb-dev
 
 sudo apt install libpcap-dev
 
 sudo apt install ros-noetic-teb-local-planner
 
 sudo apt install ros-noetic-pointcloud-to-laserscan
 
 sudo apt install tf2*
 
 sudo apt install libqt5serialport5-dev
 
 # 安装communication_rely两个package
 
 # 安装Livox的两个SDK
 ```

#### 方法2：

```bash
chmod +x install_lib.sh
./install_lib.sh
```

