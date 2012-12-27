cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

rosbuild_add_library(orcdchomp
   src/orcdchomp.cpp
   src/orcdchomp_mod.cpp
   src/orcdchomp_rdata.cpp
   src/orcwrap.cpp
   src/libcd/chomp.c
   src/libcd/grid.c
   src/libcd/grid_flood.c
   src/libcd/kin.c
   src/libcd/mat.c
   src/libcd/os.c
   src/libcd/spatial.c
   src/libcd/util.c
   src/libcd/util_shparse.c)
target_link_libraries(orcdchomp blas lapacke lapack gsl)
