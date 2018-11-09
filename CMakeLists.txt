cmake_minimum_required(VERSION 2.8.3)
project(pr2_arm_controllers)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

## Generate services in the 'srv' folder
# add_service_files(
#         FILES
#         move_baxter_arm.srv	
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(baxter_mover_utils
#         src/baxter_mover.cpp
#         src/libs/helpers_methods.cpp
#         )

# target_link_libraries(baxter_mover_utils ${catkin_LIBRARIES}) 

## Mark executables and/or libraries for installation
# install(TARGETS baxter_mover_utils
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable (controller_push_primitive src/controller_push_primitive.cpp)
target_link_libraries(controller_push_primitive ${catkin_LIBRARIES})
add_executable(controller_grasping_primitive src/controller_grasping_primitive.cpp)
target_link_libraries(controller_grasping_primitive ${catkin_LIBRARIES})
add_executable(controller_button_primitive src/controller_button_primitive.cpp)
target_link_libraries(controller_button_primitive ${catkin_LIBRARIES})
add_executable(controller_joystick_primitive src/controller_joystick_primitive.cpp)
target_link_libraries(controller_joystick_primitive ${catkin_LIBRARIES})
#~ add_executable(controller_lego_demo src/controller_lego_demo.cpp)
#~ target_link_libraries(controller_lego_demo ${catkin_LIBRARIES})



