file(GLOB_RECURSE all_sources "./*.cpp")

#add_library(ba_example SHARED ${all_sources})
add_library(ba_example  ${all_sources})
set_property(TARGET ba_example PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ba_example ${THIRD_PARTY_LIBS})
