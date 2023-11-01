file(GLOB_RECURSE all_sources "./*.cpp")

add_library(EKF_SLAM SHARED ${all_sources})
target_link_libraries(EKF_SLAM ${THIRD_PARTY_LIBS})