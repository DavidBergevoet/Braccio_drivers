cmake_minimum_required(VERSION 3.7.2)
project(braccio_vision)

add_compile_options(-std=c++14 -Wall -Wextra -Wconversion)

find_package(catkin REQUIRED COMPONENTS
			roscpp
			rospy
			message_generation
			std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   ImagePoint.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB source_files
	src/*.cpp
)
add_executable(${PROJECT_NAME}
	${source_files}
)
target_link_libraries(${PROJECT_NAME}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
