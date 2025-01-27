
cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  std_msgs 
  message_generation 
  actionlib_msgs 
  actionlib 
  kortex_driver
)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/include)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)

add_executable(SimpleMoveCPP src/SimpleMove.cpp)
add_dependencies(SimpleMoveCPP ${catkin_EXPORTED_TARGETS})
add_library(RobotControl src/RobotControl.cpp)
target_link_libraries(SimpleMoveCPP ${catkin_LIBRARIES})
target_link_libraries(RobotControl ${catkin_LIBRARIES})
target_link_libraries(SimpleMoveCPP RobotControl)

# if(CATKIN_ENABLE_TESTING)
# 	find_package(rostest REQUIRED)
# 	catkin_add_gtest(kortex_examples_tests src/tests/kortex_examples_tests.cc)
#     target_link_libraries(kortex_examples_tests ${catkin_LIBRARIES})
# endif()

# Install

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS src/actuator_config/example_actuator_configuration.py
#                                src/full_arm/example_cartesian_poses_with_notifications.py
#                                src/full_arm/example_full_arm_movement.py
#                                src/full_arm/example_waypoint_action_client.py
#                                src/move_it/example_move_it_trajectories.py
#                                src/vision_config/example_vision_configuration.py 
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(
  DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/
)
install(TARGETS 
    SimpleMoveCPP
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)