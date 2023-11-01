cmake_minimum_required(VERSION 3.8)
project(ba_example)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")



list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



## Dependencies

# Eigen
include_directories("/usr/include/eigen3")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

# fmt
find_package(fmt REQUIRED)


set(THIRD_PARTY_LIBS
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        fmt::fmt
        )

include_directories(${PROJECT_SOURCE_DIR}/include)

set(s_ba src/main.cpp)
add_executable(test_BA ${s_ba})
target_link_libraries(test_BA ba_example)

add_subdirectory(src)