cmake_minimum_required(VERSION 3.0.2)
project(ros_landing)

find_package(catkin REQUIRED 
    COMPONENTS 
    roscpp
    geometry_msgs 
    mavros_msgs
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    message_filters
)

find_package(OpenCV 4 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   mavros_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_landing
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(altitudeProcessing src/VisionProcessing/AltitudeDataProcessing.cpp src/VisionProcessing/AltitudeDataProcessingBase.cpp)
# target_link_libraries(altitudeProcessing ${OpenCV_LIBS})

add_library(frameProcessing src/VisionProcessing/FrameProcessing.cpp)
target_link_libraries(frameProcessing ${OpenCV_LIBS})

add_library(objectProcessing src/VisionProcessing/ObjectParameterAnalysis.cpp)
target_link_libraries(objectProcessing ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_control_node src/uav_control.cpp)
target_link_libraries(${PROJECT_NAME}_control_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_vision_processing src/vision_processing.cpp)
target_link_libraries(${PROJECT_NAME}_vision_processing frameProcessing objectProcessing altitudeProcessing ${catkin_LIBRARIES} ${OpenCV_LIBS})

# catkin_install_python(PROGRAMS
#   src/uav_teleop_control.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )