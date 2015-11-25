
cmake_minimum_required(VERSION 2.8.3)
project(rob_proj)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(include ${OpenCV_INCLUDE_DIRS})


## Declare a catkin package
catkin_package()


add_library(${PROJECT_NAME} 
  src/FaceDetector.cpp
  )

add_executable(rob src/rob.cpp)

target_link_libraries(${PROJECT_NAME}   
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
  #${PCL_LIBRARIES} 
  #${Boost_LIBRARIES}
)


target_link_libraries(rob ${PROJECT_NAME})



add_dependencies(rob rob_proj_generate_messages_cpp)

