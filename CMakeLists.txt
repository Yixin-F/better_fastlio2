cmake_minimum_required(VERSION 3.10)

project(fast_lio_sam)

# Set the C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# Processor count logic
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Include directories and dependencies
find_package(OpenMP QUIET)
if (OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
endif()

find_package(PythonLibs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
  cv_bridge
  image_transport
  darknet_ros_msgs
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(GTSAM REQUIRED QUIET)

# Other setup
add_message_files(DIRECTORY msg FILES Pose6D.msg cloud_info.msg)
add_service_files(DIRECTORY srv FILES save_map.srv save_pose.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs nav_msgs sensor_msgs)

# Catkin package
catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime DEPENDS EIGEN3 PCL GTSAM INCLUDE_DIRS include)

# Include directories
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} include)

# Add libraries and executables
add_library(ikd_Tree include/ikd-Tree/ikd_Tree.cpp)
add_library(preprocess src/preprocess.cpp)
add_library(sc include/sc-relo/Scancontext.cpp)
add_library(incremental_mapping include/multi-session/Incremental_mapping.cpp include/sc-relo/Scancontext.cpp)
add_library(pose_estimator include/online-relo/pose_estimator.cpp include/sc-relo/Scancontext.cpp)
add_library(tgrs include/dynamic-remove/tgrs.cpp)

# Executables and linkage with stdc++fs
add_executable(fastlio_sam_mapping src/laserMapping.cpp)
target_link_libraries(fastlio_sam_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBRARIES} ikd_Tree preprocess sc gtsam stdc++fs)

add_executable(multi_session src/multi_session.cpp)
target_link_libraries(multi_session ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${GeographicLib_LIBRARIES} incremental_mapping gtsam stdc++fs)

add_executable(online_relo src/online_relocalization.cpp)
target_link_libraries(online_relo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBRARIES} pose_estimator incremental_mapping gtsam stdc++fs)

add_executable(object_update src/object_update.cpp)
target_link_libraries(object_update ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBRARIES} tgrs gtsam stdc++fs)
