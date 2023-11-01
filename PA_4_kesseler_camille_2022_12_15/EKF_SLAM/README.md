### This program implements 2D EKF SLAM with simple example data.
### Date: 2022/11/29
### Author: Jiseong CHUNG

### This program is tested in Ubuntu 18.04. But it probably runs any version of Ubuntu.
### This program is incomplete version to support students' programming assignment.

### File structure
EKF_SLAM
├─bin
├─data
│  ├─sensor_data.dat
│  └─world.dat
├─include
│  ├─Common.h
│  ├─Map.h
│  ├─Measurement.h
│  └─EKFSLAM.h       -------------- you have to implement this!
├─src
│  ├─CMakeLists.txt
│  ├─Map.cpp
│  ├─Measurement.cpp
│  ├─EKFSLAM.cpp     -------------- you have to implement this!
│  └─main.cpp        -------------- you have to implement this!
└─CMakeLists.txt


### Installation and compile
cd {EXTRACTED_PATH}/EKF_SLAM
mkdir build
cd build
cmake ..
make -j4

### Running
cd ../bin
./test_EKF ../data/world.dat ../data/sensor_data.dat

### Note
You can feel free to add or modify the codes that you need to use(e.g. Visualization.h, Visualization.cpimage.pngimage.pngp, Tools.h ...)