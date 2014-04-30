cmake_minimum_required(VERSION 2.8.3)
project(hironx_moveit_config)

find_package(catkin REQUIRED COMPONENTS hironx_ros_bridge)

# generate model file
file(MAKE_DIRECTORY "models/meshes")
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/HiroNX.urdf
  COMMAND echo "== generate urdf file from kawada-hironx.dae"
  COMMAND rosrun collada_urdf collada_to_urdf ${hironx_ros_bridge_SOURCE_DIR}/models/kawada-hironx.dae -A --mesh_output_dir ${PROJECT_SOURCE_DIR}/models/meshes --mesh_prefix 'package://hironx_moveit_config/models/meshes'
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/"
#  DEPENDS ${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae
  )
add_custom_target(${PROJECT_NAME}_HiroNX_urdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/HiroNX.urdf)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".svn" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".svn" EXCLUDE)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".svn" EXCLUDE)
