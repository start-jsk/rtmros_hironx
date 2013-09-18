cmake_minimum_required(VERSION 2.8.3)
project(hironx_ros_bridge)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge)

catkin_package(
    DEPENDS # TODO
    CATKIN-DEPENDS hrpsys_ros_bridge #
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

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

# set HIRONX_ROS_BRIDGE and OPENHRP3 for configure_file
set(HIRONX_ROS_BRIDGE ${PROJECT_SOURCE_DIR})
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENHRP3 ${openhrp3_PREFIX}/share/openhrp3)
 
configure_file(models/kawada-hironx.RobotHardware.conf.in       ${PROJECT_SOURCE_DIR}/models/kawada-hironx.RobotHardware.conf)
configure_file(models/kawada-hironx_nosim.RobotHardware.conf.in ${PROJECT_SOURCE_DIR}/models/kawada-hironx_nosim.RobotHardware.conf)
configure_file(models/kawada-hironx.xml.in                      ${PROJECT_SOURCE_DIR}/models/kawada-hironx.xml)
configure_file(models/kawada-hironx_nosim.xml.in                ${PROJECT_SOURCE_DIR}/models/kawada-hironx_nosim.xml)
configure_file(models/kawada-hironx.conf.in                     ${PROJECT_SOURCE_DIR}/models/kawada-hironx.conf)
configure_file(models/kawada-hironx_nosim.conf.in               ${PROJECT_SOURCE_DIR}/models/kawada-hironx_nosim.conf)
add_custom_target(model_files ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/kawada-hironx.RobotHardware.conf ${PROJECT_SOURCE_DIR}/models/kawada-hironx_nosim.RobotHardware.conf ${PROJECT_SOURCE_DIR}/models/kawada-hironx.xml ${PROJECT_SOURCE_DIR}/models/kawada-hironx_nosim.xml ${PROJECT_SOURCE_DIR}/models/kawada-hironx.conf ${PROJECT_SOURCE_DIR}/models/kawada-hironx_nosim.conf)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN "*.in" EXCLUDE)
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

install(CODE "
  file(GLOB _xml_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/hironx_ros_bridge/models/*.xml)
  file(GLOB _conf_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/hironx_ros_bridge/models/*.conf)
  foreach(_file \${_xml_files};\${_conf_files})
    message(\"++ sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
    message(\"sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
    execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file})
  endforeach()
  ")

