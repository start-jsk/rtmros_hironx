cmake_minimum_required(VERSION 2.8.3)
project(hironx_ros_bridge)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge)

catkin_package(
    DEPENDS # TODO
    CATKIN-DEPENDS hrpsys_ros_bridge #
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

catkin_python_setup()

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

# set ROBOT_NAME and OPENHRP3 for configure_file
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENHRP3 ${openhrp3_PREFIX}/share/openhrp3)  # for longfloor.wrl
set(ROBOT_NAME kawada-hironx)

configure_file(conf/RobotHardware.conf.in       ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.RobotHardware.conf)
configure_file(conf/nosim.RobotHardware.conf.in ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.RobotHardware.conf)
configure_file(conf/xml.in                      ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml)
configure_file(conf/nosim.xml.in                ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml)
configure_file(conf/conf.in                     ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.conf)
configure_file(conf/nosim.conf.in               ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.conf)
add_custom_target(${PROJECT_NAME}_model_files ALL DEPENDS ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.RobotHardware.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.RobotHardware.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.conf)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY conf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

install(CODE "
  file(GLOB _xml_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_SOURCE_DIR}/conf/*.xml)
  file(GLOB _conf_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_SOURCE_DIR}/conf/*.conf)
  foreach(_file \${_xml_files};\${_conf_files})
    message(\"++ sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
    message(\"sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
    execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file})
  endforeach()
  ")

