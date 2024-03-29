cmake_minimum_required(VERSION 2.8.3)
project(choose_powerline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  vision_msgs
  message_filters
  cv_bridge
  image_transport
  visualization_msgs
  tf2
)

# 查找OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
  # ...
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(min_rects_processor_node src/min_rects_processor.cpp)
target_link_libraries(min_rects_processor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(mouse_node src/mouse.cpp)
target_link_libraries(mouse_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)