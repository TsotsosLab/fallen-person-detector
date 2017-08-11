##
## Created by markus d. solbach
## solbach@eecs.yorku.ca
##
cmake_minimum_required(VERSION 2.8.3)
project(fallen_person_detector)

add_definitions(-std=c++11)

find_package(OpenCV 3.1 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        openpose_ros
        roscpp
        sensor_msgs
        std_msgs
        pcl_ros
        pcl_conversions
        image_transport
        message_filters
        HINTS "${CMAKE_PREFIX_PATH}"
        )

find_package(Boost REQUIRED COMPONENTS
        signals
        )

catkin_package(INCLUDE_DIRS src
        CATKIN_DEPENDS
        openpose_ros
        image_transport
        message_filters
        std_msgs sensor_msgs
        )
include_directories(${roscpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  ${roscpp_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/fallen_person_detector.cpp src/ground_floor_finder.cpp src/ground_floor_finder.h src/fallen_person_detector.h src/pcl_util.cpp src/pcl_util.h)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

########################################################################################################################
################################################# Some Debugging Code ##################################################
# MESSAGE(STATUS ${openpose_ros_DIR})
# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#     message(STATUS "dir='${dir}'")
# endforeach()
