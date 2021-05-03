cmake_minimum_required(VERSION 2.8.3)
project(mytest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wpedantic -O2 -msse2 -pg")

option(ZED "image size" ON)
if (ZED)
    add_definitions(-D_ZED_CAM_)
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        sensor_msgs
        std_msgs
        pcl_ros
        jsk_recognition_msgs
	image_transport
	geometry_msgs
        )
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(PCL 1.7 REQUIRED COMPONENTS common io)


catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES Detection
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime pcl_ros jsk_recognition_msgs geometry_msgs
   #DEPENDS system_lib
)



#set(PCL_INCLUDE_DIRS /usr/include/pcl-1.7)
include_directories(include
        ${OpenCV_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
#        ${EIGEN3_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
        )

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# 校正
add_executable(rectify 
              #src/stereo_calibrator_node.cpp
               src/stereo_calibrator.cpp
              src/IiBasicDef.cpp
              src/IIStereoCalibrator.cpp
)
target_link_libraries(rectify
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
# 立体校正接受图片测试
add_executable(image_listener 
              src/accept_rect.cpp
)
target_link_libraries(image_listener
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#TDA4
add_executable(TDA4 
              src/TDA4.cpp
)
target_link_libraries(TDA4
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#　语义分割
add_executable(segment 
              src/seg.cpp
)
target_link_libraries(segment
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# 匹配
add_executable(match
              src/stereo_disparity_node.cpp
              src/stereo_disparity.cpp
              src/IiBasicDef.cpp
              src/IIOpencvDisparityCalculator.cpp
)
target_link_libraries(match
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# 3D-Box
add_executable(main
        main.cpp
        src/IiBasicDef.cpp
        src/colorCCA.cpp
	src/dbscan.cpp
        src/objectdetection_buildminbox.cpp
        src/objectdetection_dbscan.cpp
)

target_link_libraries(main
        ${OpenCV_LIBS}
        gflags
        ${PYTHON_LIBRARIES}
        ${catkin_LIBRARIES}
	${Pangolin_LIBRARIES}
#        ${PCL_LIBRARIES}
)
