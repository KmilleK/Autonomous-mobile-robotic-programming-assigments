cmake_minimum_required( VERSION 2.8 )
project( PA_3 )

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} )

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable( PA_feature ORB_feature.cpp )
target_link_libraries(PA_feature ${OpenCV_LIBS})

add_executable(PA_epipolar Epipolar_geometry.cpp)
target_link_libraries(PA_epipolar ${OpenCV_LIBS} Eigen3::Eigen)
