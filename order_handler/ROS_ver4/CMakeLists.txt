cmake_minimum_required(VERSION 2.8.3)
project(android_ros_pkg)

find_package(catkin REQUIRED
  roscpp
  std_msgs	
)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

##메시지 선언:MsgTutorial.msg
##add_message_files(FILES MsgTutorial.msg)
add_message_files(FILES orders.msg)
add_message_files(FILES workDone.msg)
##의존하는 메시지 설정
generate_messages(DEPENDENCIES std_msgs)

##catkin 패키지 옵션
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  LIBRARIES android_ros_pkg
)

include_directories(${catkin_INCLUDE_DIRS})

##order_publisher 노드에 대한 빌드 옵션
add_executable(order_publisher src/order_publisher.cpp)
add_dependencies(order_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(order_publisher ${catkin_LIBRARIES})

##order_subscriber 노드에 대한 빌드 옵션 
add_executable(order_subscriber src/order_subscriber.cpp)
add_dependencies(order_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(order_subscriber ${catkin_LIBRARIES})


##drink_publisher 노드에 대한 빌드 옵션
##add_executable(drink_publisher src/drink_pub.cpp)
##add_dependencies(drink_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
##target_link_libraries(drink_publisher ${catkin_LIBRARIES})

##drink_subscriber 노드에 대한 빌드 옵션 
##add_executable(drink_subscriber src/drink_subscriber.cpp)
##add_dependencies(drink_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
##target_link_libraries(drink_subscriber ${catkin_LIBRARIES})


##test1_node 노드에 대한 빌드 옵션
##add_executable(test1_node src/test1_node.cpp)
##target_link_libraries(test1_node ${catkin_LIBRARIES})

##install(DIRECTORY launch
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
##install(DIRECTORY meshes
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



