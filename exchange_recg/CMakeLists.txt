cmake_minimum_required(VERSION 2.8.3)
project(exchange_recg)

add_compile_options(-std=c++14)

find_package(
        catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        geometry_msgs
)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES windMill
        #  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}_node
        src/main.cpp
        src/manager.cpp
        src/triangle.cpp
        src/edge.cpp
        src/square.cpp
        src/utils.cpp
        src/auxPoints.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)
