cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED roslib urdf srdfdom openrave_catkin)
catkin_package()

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(OpenRAVE REQUIRED)

include_directories(
    include/${PROJECT_NAME}
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
