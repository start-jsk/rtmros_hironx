cmake_minimum_required(VERSION 2.8.3)
project(hironx_ros_bridge)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# rosbuild_find_ros_package(hrpsys_ros_bridge)
# include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)

# rosbuild_download_data(https://github.com/rdiankov/collada_robots/raw/master/kawada-hironx.zae models/kawada-hironx.zae be4b0015914d33a5aaa24ee055bcdbc8)
# add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae
#   COMMAND unzip -u ${PROJECT_SOURCE_DIR}/models/kawada-hironx.zae
#   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
#   DEPENDS ${PROJECT_SOURCE_DIR}/models/kawada-hironx.zae)
# compile_collada_model(${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae)
