cmake_minimum_required(VERSION 2.8.3)
project(hoverboard_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/main.cpp
  src/hover.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(hoverboard_driver ${PROJECT_NAME}_gencfg)

