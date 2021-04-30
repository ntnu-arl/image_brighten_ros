cmake_minimum_required(VERSION 2.8.3)
project(image_brighten)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  genmsg
  tf
  image_transport
  message_generation
  dynamic_reconfigure
  cv_bridge
  nodelet
)

set("OpenCV_DIR" "/usr/share/OpenCV") #FORCE SYSTEM VERSION

find_package( OpenCV REQUIRED )

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    config/ImageBrighten.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_library(dehaze src/dehaze.cpp)
add_library(guided_filter src/guided_filter.cpp)

## Nodelet
add_library(${PROJECT_NAME}_nodelet
    src/image_brighten_nodelet.cpp
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  dehaze
  guided_filter
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  dehaze
  guided_filter
)
