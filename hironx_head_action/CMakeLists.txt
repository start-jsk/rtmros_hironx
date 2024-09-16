# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hironx_head_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   actionlib 
   geometry_msgs 
   kdl_parser
   message_filters 
   pr2_controllers_msgs 
   roscpp 
   sensor_msgs 
   trajectory_msgs 
   tf 
   tf_conversions)

find_package(orocos_kdl REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS}
   ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(head_action src/point_frame.cpp)
target_link_libraries(head_action ${Boost_LIBRARIES} 
   ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(head_action ${catkin_EXPORTED_TARGETS})

install(TARGETS head_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY test launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS)

# System tests are done in the downstream, hironx_ros_bridge pkg.
# Because system tests require some launch files hironx_ros_bridge provides, but
# then hironx_ros_bridge depends on this package so that circular dependency 
# occurs. That's why tests are moved there to avoid circular.
