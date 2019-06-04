cmake_minimum_required(VERSION 2.8.3)
project(position_to_velocity)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        tf2_eigen
        reef_msgs
        nav_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs tf2_eigen reef_msgs
        DEPENDS eigen
)

include_directories(include ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        )

add_executable(${PROJECT_NAME}_node
            src/position_to_velocity_node.cpp
        include/position_to_velocity/position_to_velocity.h
        src/position_to_velocity.cpp)

add_dependencies(${PROJECT_NAME}_node reef_msgs_generate_messages_cpp reef_msgs)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} reef_msgs)

