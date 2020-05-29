cmake_minimum_required(VERSION 2.8.3)
project(hubo_slam)

set(PACKAGE_DEPENDENCIES
        cartographer_ros
    )

find_package(catkin REQUIRED)

catkin_package(
    
    )

include_directories(

    )

install(DIRECTORY launch configuration_files urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
