cmake_minimum_required(VERSION 2.8.3)
project(hironx_rpc_msgs)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	geometry_msgs
	message_generation
	std_msgs
	tork_rpc_util
)

add_action_files(DIRECTORY action)
add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs tork_rpc_util)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hironx_rpc_msgs
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
