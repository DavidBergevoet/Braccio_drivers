cmake_minimum_required(VERSION 3.7.2)
project(braccio_kinematics)

add_compile_options(-std=c++14 -Wall -Wconversion -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  braccio_interface
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

file(GLOB source_files
	src/*.cpp
)
add_executable(${PROJECT_NAME}
	${source_files}
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


