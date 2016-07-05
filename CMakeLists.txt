cmake_minimum_required(VERSION 2.8.3)
project(or_urdf)

find_package(catkin REQUIRED COMPONENTS
    roslib
    srdfdom
    urdf
)

find_package(Boost REQUIRED COMPONENTS
    filesystem
    system
)
find_package(OpenRAVE REQUIRED)

catkin_package()
catkin_add_env_hooks("20.${PROJECT_NAME}"
   SHELLS sh
   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/catkin-env-hooks"
)

include_directories(
    "include/${PROJECT_NAME}"
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
)

add_library("${PROJECT_NAME}_plugin" MODULE
    src/urdf_loader.cpp
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
    PREFIX ""
    COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
    LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
    LIBRARY_OUTPUT_DIRECTORY
    "${CATKIN_DEVEL_PREFIX}/lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}")
target_link_libraries("${PROJECT_NAME}_plugin"
    ${catkin_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif(CATKIN_ENABLE_TESTING)

install(TARGETS "${PROJECT_NAME}_plugin"
    LIBRARY DESTINATION "lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

