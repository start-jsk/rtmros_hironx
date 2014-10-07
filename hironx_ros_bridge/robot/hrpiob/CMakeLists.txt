cmake_minimum_required(VERSION 2.6)
set(CMAKE_INSTALL_PREFIX /opt/nextage-open CACHE STRING "install prefix")

#
# BUILD_TYPE
#
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

#if (NOT ROBOT)
#  message(FATAL_ERROR "please define ROBOT")
#endif()

#set(ROBOT CACHE STRING "robot type")


#
# CXX
#
if(QNXNTO)
SET (CMAKE_CXX_COMPILER   QCC )
SET (CMAKE_CXX_FLAGS      "-V4.4.2,gcc_ntox86_cpp -Wall")
endif()

#
# set path
#
#include_directories(/opt/nextage-open/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${CMAKE_CURRENT_BINARY_DIR})
#link_directories(/opt/nextage-open/lib)

#if(QNXNTO)
include_directories(/opt/jsk/include /usr/pkg/include)
link_directories(/opt/jsk/lib /usr/pkg/lib)
#endif()

# hrpsys-base
#include(FindPkgConfig)

execute_process(
  COMMAND pkg-config --cflags-only-I hrpsys-base
  OUTPUT_VARIABLE HRPSYSBASE_CXX_FLAGS
  RESULT_VARIABLE RESULT)

message("HRPSYSBASE_CXX_FLAGS: ${HRPSYSBASE_CXX_FLAGS}")

add_definitions(${HRPSYSBASE_CXX_FLAGS})


#
# set target
#
##add_library(NextageOpenIo      SHARED iob.cpp)
add_library(hrpIo      SHARED iob.cpp)

#set_target_properties(hrpIo PROPERTIES PUBLIC_HEADER nextage-open.hpp)

#
# set libraries
#
if(NOT QNXNTO)
#target_link_libraries(NextageOpenIo rt boost_thread)
target_link_libraries(hrpIo rt boost_thread)
else()
#target_link_libraries(NextageOpenIo boost_thread)
target_link_libraries(hrpIo boost_thread)
endif()

#
# install
#
#install(TARGETS iob unlock_iob
#install(TARGETS NextageOpenIo
install(TARGETS hrpIo
	RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
	LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
	PUBLIC_HEADER DESTINATION include CONFIGURATIONS Release Debug
)
