cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED roslib urdf srdfdom openrave_catkin)
catkin_package()

find_package(Boost REQUIRED COMPONENTS filesystem system)

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

include_directories(
    include/${PROJECT_NAME}
    ${Boost_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${YamlCpp_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)
add_definitions(--std=c++0x)

openrave_plugin("${PROJECT_NAME}_plugin"
    src/urdf_loader.cpp
    src/or_serialization.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    ${YamlCpp_LIBRARIES}
    ${catkin_LIBRARIES}
)
