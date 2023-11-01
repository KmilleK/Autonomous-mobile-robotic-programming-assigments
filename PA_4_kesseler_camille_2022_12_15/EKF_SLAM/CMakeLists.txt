cmake_minimum_required (VERSION 3.5)
project(EKF_SLAM)

#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


###########################################
## Dependencies

# Eigen
include_directories("/usr/include/eigen3")


###########################################


include_directories(${PROJECT_SOURCE_DIR}/include)

set(s_ekf src/main.cpp)
add_executable(test_EKF ${s_ekf})
target_link_libraries(test_EKF EKF_SLAM )
add_subdirectory(src)