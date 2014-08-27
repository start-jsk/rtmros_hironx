cmake_minimum_required(VERSION 2.8.3)
project(hironx_ros_bridge)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge pr2_controllers_msgs roslib rostest)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    CATKIN_DEPENDS std_msgs
    CATKIN_DEPENDS hrpsys_ros_bridge pr2_controllers_msgs roslib #
    INCLUDE_DIRS include
    LIBRARIES ros_client_cpp
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
set(PKG_CONFIG_PATH "${openhrp3_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}") # for openrtm3.1.pc
execute_process(
  COMMAND pkg-config --variable=idl_dir openhrp3.1
  OUTPUT_VARIABLE OPENHRP_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT RESULT EQUAL 0)
  set(OPENHRP_FOUND FALSE)
endif()
# OPENHRP_IDL_DIR = <openhrp3>/share/OpenHRP-3.1/idl/
set(OPENHRP_SAMPLE_DIR ${OPENHRP_IDL_DIR}/../../../)
set(OPENHRP3 ${OPENHRP_SAMPLE_DIR})  # for longfloor.wrl
set(ROBOT_NAME kawada-hironx)

message("Configure ${ROBOT_NAME} related file with OPENHRP3=${OPENHRP3}")
configure_file(conf/RobotHardware.conf.in       ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.RobotHardware.conf)
configure_file(conf/nosim.RobotHardware.conf.in ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.RobotHardware.conf)
configure_file(conf/xml.in                      ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml)
configure_file(conf/nosim.xml.in                ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml)
configure_file(conf/conf.in                     ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.conf)
configure_file(conf/nosim.conf.in               ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.conf)
add_custom_target(${PROJECT_NAME}_model_files ALL DEPENDS ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.RobotHardware.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.RobotHardware.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.conf)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#add_definitions("-std=c++0x")
add_library(
  ros_client_cpp include/ros_client.cpp
)

add_executable(
    acceptancetest_hironx_cpp src/acceptancetest_hironx.cpp
)

target_link_libraries(
  acceptancetest_hironx_cpp
  ros_client_cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

install(TARGETS acceptancetest_hironx_cpp
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".svn" EXCLUDE)
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS PATTERN ".svn" EXCLUDE)
install(DIRECTORY conf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".svn" EXCLUDE)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".svn" EXCLUDE)
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS PATTERN ".svn" EXCLUDE)

install(CODE "
  message(\"++ glob files under \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/conf/\")
  file(GLOB _xml_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/conf/*.xml)
  file(GLOB _conf_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/conf/*.conf)
  message(\"++ sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
  if (EXISTS ${openhrp3_SOURCE_DIR})
    message(\"++sed -i s@${openhrp3_SOURCE_DIR}/share/OpenHRP-3.1@${CMAKE_INSTALL_PREFIX}/share/openhrp3/share/OpenHRP-3.1@g \${_conf_file}\")
  endif()
  foreach(_file \${_xml_files};\${_conf_files})
    message(\"  ++ fix \${_file}\")
    execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file})
    if (EXISTS ${openhrp3_SOURCE_DIR})
      execute_process(COMMAND sed -i s@${openhrp3_SOURCE_DIR}/share/OpenHRP-3.1@${CMAKE_INSTALL_PREFIX}/share/openhrp3/share/OpenHRP-3.1@g \${_file})
    endif()
    execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file})
  endforeach()
  ")

install(TARGETS acceptancetest_hironx_cpp ros_client_cpp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

add_rostest(test/test-hironx.test)
add_rostest(test/test-hironx-ros-bridge.test)
