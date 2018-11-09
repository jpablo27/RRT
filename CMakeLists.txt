cmake_minimum_required(VERSION 2.8.3)
project(rrt)




find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)




catkin_package()




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
	include include
)


link_directories()

add_definitions( -std=c++0x -g -O3)

add_executable(rrt src/main.cpp include/rrt.cpp)

target_link_libraries(rrt ${catkin_LIBRARIES})
