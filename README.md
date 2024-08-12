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

#### 方法1（建议）：

```bash
chmod +x install_lib.sh
./install_lib.sh
```

#### 方法2：

 ```bash
 sudo apt install libusb-dev
 
 sudo apt install libpcap-dev
 
 sudo apt install ros-noetic-teb-local-planner
 
 sudo apt install ros-noetic-pointcloud-to-laserscan
 
 sudo apt install tf2*
 
 sudo apt install libqt5serialport5-dev
 
 # 安装communication_rely两个package
 
 # 安装Livox的两个SDK
 
 # 这四个均在目录prerequisites_lib下
 ```

## Build

### 未使用clangd

`catkin build`

### 使用clangd

```bash
# 编译程序并生成compile_commands.json
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1
# 将所有的compile_commands.json集成到build目录下，并更改部分内容
cd build
cat ./*/compile_commands.json > ./compile_commands.json
sed -i 's/\]\[/\,/g' compile_commands.json
```

## Run

### 更改雷达ID

更改`./amphibious_uav_ugv/src/3DLidar/livox_ros_driver2/config`中的5个ip

### 首先刷新环境变量

```bash
source ./devel/setup.bash
```

**以下指令均为并行，选择其一启动即可**

### 运行整个两栖系统

```bash
roslaunch system_bringup bringup_all_system.launch
```

### 单独运行里程计

```bash
roslaunch system_bringup odom_bringup_.launch
```

### 单独运行无人车导航（已启动里程计）

```bash
roslaunch system_bringup system_bringup_ugv_alone.launch
```

### 单独运行无人机导航（已启动里程计）

```bash
roslaunch system_bringup system_bringup_uav_alone.launch
```

### 单独运行无人车导航（未启动里程计，可配合单独运行里程计使用）

```bash
roslaunch system_bringup system_bringup_ugv_without_odom.launch
```

### 单独运行无人机导航（未启动里程计，可配合单独运行里程计使用）

```bash
roslaunch system_bringup system_bringup_uav_without_odom.launch
```

