cmake_minimum_required(VERSION 3.10)
project(humanlike_moving_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++17 -lstdc++fs")  

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  xacro
  roslib
)

#find_package(Torch REQUIRED)
#find_package(Scipy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.8 REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    roslib
)



###########
## Build ##
###########

include_directories(include
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# IK server
add_executable(IK_server
  src/IK_server.cpp
)

target_link_libraries(IK_server
  ${catkin_LIBRARIES}
)

# dataset_creator
add_executable(dataset_creator
  src/dataset_creator.cpp
)

target_link_libraries(dataset_creator
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

# test_creator
add_executable(test_creator
  src/test_creator.cpp
)

target_link_libraries(test_creator
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)




#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/human_poses.py
  scripts/human_poses_test.py
  scripts/NN_engine.py
  scripts/NN_trainer.py
  scripts/main.py
  scripts/controller.py
  scripts/Panda_trajectory_planner.py
  scripts/GUI.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
