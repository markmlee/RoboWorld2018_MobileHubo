cmake_minimum_required(VERSION 2.8.3)
project(drc_hubo_sensorpack)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  message_generation
  sensor_msgs
  object_detector
  smach
  tf
  
)


add_message_files(
   FILES 
   Planning_data.msg
   Planning_command.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drc_hubo_sensorpack
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

###################################
       ## time sync node ##
###################################

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_timesync src/drc_hubo_sensorpack_node.cpp)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME}_timesync ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_timesync
#   ${catkin_LIBRARIES}
# )


###################################
       ## ros_podo LAN node ##
###################################

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_communication src/ros_podo_node.cpp)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_communication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_communication
   ${catkin_LIBRARIES}
 )




