### This program implements Bundle adjustment with simple example data.
### Date: 2022/12/16
### Author: Jiseong CHUNG

### This program is tested in Ubuntu 20.04. But it probably runs any version of Ubuntu.
### This program is incomplete version to support students' programming assignment.

### Dependencies
OS: Ubuntu 20.04(WSL2)
gcc: 9.4
CMake: 3.15
Ceres: 2.1.0(latest)
Eigen: 3.2.4
CSparse: 5.1.2
Sophus: 22.10(latest)

### File structure
BA
├─bin
├─data
│  ├─data_gnd
│  │  ├─camera_poses.csv
│  │  ├─camera.csv
│  │  ├─keypoints.csv
│  │  ├─points_ids.csv
│  │  └─points.csv
│  └─data_noisy
│     ├─camera_poses.csv
│     ├─camera.csv
│     ├─keypoints.csv
│     ├─points_ids.csv
│     └─points.csv
├─include
│  ├─Common.h
│  ├─Feature.h
│  ├─Frame.h
│  ├─MapPoint.h
│  ├─Reader.h
│  └─ProjectionFactor.h       -------------- you have to implement this!
├─src
│  ├─CMakeLists.txt
│  ├─Feature.cpp
│  ├─Frame.cpp
│  ├─MapPoint.cpp
│  ├─Reader.cpp
│  └─main.cpp                 -------------- you have to implement this!
└─CMakeLists.txt


### Installation and compile
cd {EXTRACTED_PATH}/BA
mkdir build
cd build
cmake ..
make -j4

### Running
cd ../bin
./test_BA ../data/data_noisy/

### Note
You can feel free to add or modify the codes that you need to use(e.g. Visualization.h, Visualization.cpp, Tools.h ...)