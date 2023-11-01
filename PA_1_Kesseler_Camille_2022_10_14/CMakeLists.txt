cmake_minimum_required( VERSION 2.8 )
project( HomeworkP )

set(CMAKE_CXX_FLAGS "-std=c++14")

include_directories( "/usr/include/eigen3" )

set(OpenGL_GL_PREFERENCE "LEGACY")    # warming open GL cleaning 
find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)

add_executable( HomeworkP1 Problem1.cpp )
add_executable( HomeworkP2 Problem2.cpp )
add_executable( HomeworkP3 Problem3.cpp )
add_executable( HomeworkP4 Problem4.cpp )
add_executable( HomeworkP5 Problem5.cpp )

add_executable(Drawing drawing_GT.cpp)

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(Drawing ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )


add_executable(Drawing2 drawing_GT_IN.cpp)

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(Drawing2 ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

add_executable(Drawing3 drawing_GT_OPTI.cpp)

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(Drawing3 ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )