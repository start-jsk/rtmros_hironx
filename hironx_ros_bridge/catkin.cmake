cmake_minimum_required(VERSION 2.8.3)
project(hironx_ros_bridge)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge)

catkin_package()

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae
  COMMAND ${catkin_EXTRAS_DIR}/test/download_checkmd5.py
  https://github.com/rdiankov/collada_robots/raw/master/kawada-hironx.zae
  ${PROJECT_SOURCE_DIR}/models/kawada-hironx.zae
  be4b0015914d33a5aaa24ee055bcdbc8
  COMMAND unzip -u ${PROJECT_SOURCE_DIR}/models/kawada-hironx.zae
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  VERBATIM)
# commit generated files due to  https://code.google.com/p/rtm-ros-robotics/issues/detail?id=187
#compile_collada_model(${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(install_code "
  execute_process(COMMAND sed -i s@${CMAKE_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share@ $ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/hironx_ros_bridge/models/kawada-hironx.xml)
  ")
message("post process ${install_code}")
install(CODE ${install_code}) # to use CMAKE_SOURCE_DIR
