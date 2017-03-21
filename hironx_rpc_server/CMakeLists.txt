cmake_minimum_required(VERSION 2.8.3)
project(hironx_rpc_server)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hironx_rpc_server
#  CATKIN_DEPENDS hironx_ros_bridge hironx_rpc_msgs tork_rpc_util
#  DEPENDS system_lib
)

install(DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/rpc.test)
endif()

roslint_python(script/sample_rpc_script.py src/hironx_rpc_server/hironx_rpc_server.py src/hironx_rpc_server/sample_rpc.py)
