cmake_minimum_required(VERSION 2.8.3)
project(or_urdf)

find_package(catkin REQUIRED COMPONENTS
    openrave_catkin
    roslib
    srdfdom
    urdf
)
catkin_package()

find_package(Boost REQUIRED COMPONENTS
    filesystem
    system
)
find_package(OpenRAVE REQUIRED)

include_directories(
    "include/${PROJECT_NAME}"
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
)

openrave_plugin("${PROJECT_NAME}_plugin"
    src/urdf_loader.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif(CATKIN_ENABLE_TESTING)

