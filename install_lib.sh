#!/bin/bash

echo -e "\033[1;32mStart Configuration!!!!!!!!!!!!\033[0m"

sudo apt install libusb-dev

sudo apt install libpcap-dev

sudo apt install ros-noetic-teb-local-planner

sudo apt install ros-noetic-pointcloud-to-laserscan

sudo apt install tf2*

sudo apt install libqt5serialport5-dev

###################### Install Communication Moudle Start #######################################################
echo -e "\033[1;32mStart install communication prerequisites\033[0m"
echo -e "\033[1;32mInstall spdlog\033[0m"
cd ./prerequisites_lib/communication_rely/spdlog
mkdir build && cd build
cmake ..
make -j6
sudo make install
echo -e "\033[1;32mspdlog Install Finish!\033[0m"
echo -e "\033[1;32mInstall communication_lib\033[0m"
cd ../../communication_lib
mkdir build && cd build
cmake ..
echo 'export PATH=/usr/local/KdrobotCppLibs/bin/Release${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=/usr/local/KdrobotCppLibs/lib/Release/cmake${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}' >> ~/.bashrc
make -j6
sudo make install 
echo -e "\033[1;32mcommunication_lib Install Finish\033[0m"
###################### Install Communication Moudle End #######################################################

###################### Install Livox SDK Moudle Start #########################################################
echo -e "\033[1;32mInstall Livox SDK1\033[0m"
cd ../../../lidarsdk/Livox-SDK
mkdir build && cd build
cmake ..
make -j6
sudo make install
echo -e "\033[1;32mLivox SDK1 Install Finish!\033[0m"
echo -e "\033[1;32mInstall Livox SDK2\033[0m"
cd ../../Livox-SDK2
mkdir build && cd build
cmake ..
make -j6
sudo make install
echo -e "\033[1;32mLivox SDK2 Install Finish!\033[0m"
###################### Install Livox SDK Moudle End #########################################################

echo -e "\033[1;32mConfiguration Complete!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
