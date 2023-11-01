cmake_minimum_required( VERSION 2.8 )
project( PA_2 )

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH /home/ckesseler/slambook2/slambook2/3rdparty/g2o/cmake_modules)
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
#include_directories( "/usr/include/eigen3" )

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(OpenGL_GL_PREFERENCE "LEGACY")    # warming open GL cleaning 
find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)

add_executable( PE_Ceres Pose_estimation_ceres.cpp )
target_link_libraries(PE_Ceres ${CERES_LIBRARIES})


add_executable( PE_Ceres_loop Pose_estimation_loop_ceres.cpp )
target_link_libraries(PE_Ceres_loop ${CERES_LIBRARIES})

add_executable( PE_g2o Pose_estimation_g2o.cpp )
target_link_libraries(PE_g2o ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} Eigen3::Eigen)

add_executable( PE_g2o_loop Pose_estimation_loop_g2o.cpp )
target_link_libraries(PE_g2o_loop ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} Eigen3::Eigen)

add_executable( PE_complex Complex_g2o.cpp)
target_link_libraries(PE_complex ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} Eigen3::Eigen)


# Drawing for the complexe problem 
add_executable(Drawing drawing_GT.cpp)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(Drawing ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

add_executable(Drawing2 drawing_GT_IN.cpp)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(Drawing2 ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

add_executable(Drawing3 drawing_GT_OPTI.cpp)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(Drawing3 ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )