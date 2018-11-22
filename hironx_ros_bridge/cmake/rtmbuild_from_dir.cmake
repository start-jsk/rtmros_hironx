# Based on rtmbuild/cmake/rtmbuild.cmake
cmake_minimum_required(VERSION 2.8.3)

#set(DEBUG_RTMBUILD_CMAKE TRUE)

set(use_catkin TRUE)

# for rosbuild
if(NOT COMMAND _rtmbuild_genbridge_init)
  include(${rtmbuild_PACKAGE_PATH}/cmake/servicebridge.cmake)
  set(use_catkin FALSE)
endif()


##
## GLOBAL VARIABLES
##
## openrtm_aist_INCLUDE_DIRS
## openrtm_aist_LIBRARIES
## openhrp3_INCLUDE_DIRS
## openhrp3_LIBRARIES
## idl2srv_EXECUTABLE
## rtmskel_EXECUTABLE
## ${PROJECT_NAME}_idl_files
## ${PROJECT_NAME}_autogen_files
## ${PROJECT_NAME}_autogen_msg_files
## ${PROJECT_NAME}_autogen_srv_files
## ${PROJECT_NAME}_autogen_interfaces
## rtm_idlc, rtm_idlflags, rtm_idldir
## rtm_cxx,  rtm_cflags
## hrp_idldir


# Based on _rtmbuild_get_idls in rtmbuild/cmake/servicebridge.cmake
# Change points from original _rtmbuild_get_idls:
# - get idl dir name as argument
# get idl dir name as argument
# macro(_rtmbuild_get_idls)
macro(_rtmbuild_get_idls_from_dir _idl_dir)
  # get idl dir name as argument
  # file(GLOB _idl_files "${PROJECT_SOURCE_DIR}/idl/*.idl")  ## get full path
  file(GLOB _idl_files "${PROJECT_SOURCE_DIR}/${_idl_dir}/*.idl")  ## get full path
  foreach(_idl_file ${_idl_files})
    # copy from rosbuild_get_msgs to avoid .#Foo.idl, by emacs
    if(${_idl_file} MATCHES "/[^\\.]+\\.idl$")
      list(APPEND ${PROJECT_NAME}_idl_files ${_idl_file})
    endif()
  endforeach(_idl_file)
endmacro(_rtmbuild_get_idls_from_dir)

#
# setup global variables
#
# Change points from original rtmbuild_init:
# - get idl dir name as argument
# - use idl2srv.py in hironx_ros_bridge
# - use _rtmbuild_get_idls_from_dir instead of _rtmbuild_get_idls
# get idl dir name as argument
# macro(rtmbuild_init)
macro(rtmbuild_init_from_dir _idl_dir)
  # get idl dir name as argument
  # set(_extra_message_dependencies ${ARGV0})
  set(_extra_message_dependencies ${ARGV1})
  if(NOT use_catkin) ## rosbuild_init cleans all variable so we first defind project and call rosbuild_init later with ROSBUILD_DONT_REDEFINE_PROJECT=TRUE
    get_filename_component(_project ${CMAKE_SOURCE_DIR} NAME)
    project(${_project})

    rosbuild_find_ros_package(openrtm_aist)
    rosbuild_find_ros_package(openhrp3)
    set(ENV{PKG_CONFIG_PATH} ${openrtm_aist_PACKAGE_PATH}/lib/pkgconfig:${openhrp3_PACKAGE_PATH}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
    message("[rtmbuild_init] - ENV{PKG_CONFIG_PATH} -  > $ENV{PKG_CONFIG_PATH}")
  endif()
  #
  # use pkg-config to set --cflags --libs plus rtm-related flags
  #
  find_package(PkgConfig)
  pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
  pkg_check_modules(openhrp3 openhrp3.1)
  message("[rtmbuild_init] Building package ${CMAKE_SOURCE_DIR} ${PROJECT_NAME}")
  message("[rtmbuild_init] - CATKIN_TOPLEVEL = ${CATKIN_TOPLEVEL}")
  if(DEBUG_RTMBUILD_CMAKE)
    message("[rtmbuild_init] - openrtm_aist_INCLUDE_DIRS -> ${openrtm_aist_INCLUDE_DIRS}")
    message("[rtmbuild_init] - openrtm_aist_LIBRARIES    -> ${openrtm_aist_LIBRARIES}")
    message("[rtmbuild_init] - openhrp3_INCLUDE_DIRS -> ${openhrp3_INCLUDE_DIRS}")
    message("[rtmbuild_init] - openhrp3_LIBRARIES    -> ${openhrp3_LIBRARIES}")
  endif()

  # use idl2srv.py in hironx_ros_bridge
  # if(EXISTS ${rtmbuild_SOURCE_PREFIX}) # catkin
  #   set(idl2srv_EXECUTABLE ${rtmbuild_SOURCE_PREFIX}/scripts/idl2srv.py)
  # elseif(EXISTS ${rtmbuild_PACKAGE_PATH}) ## for rosbuild
  #   set(idl2srv_EXECUTABLE ${rtmbuild_PACKAGE_PATH}/scripts/idl2srv.py)
  # else()
  #   pkg_check_modules(rtmbuild rtmbuild REQUIRED)
  #   set(idl2srv_EXECUTABLE ${rtmbuild_PREFIX}/share/rtmbuild/scripts/idl2srv.py)
  # endif()
  set(idl2srv_EXECUTABLE ${PROJECT_SOURCE_DIR}/scripts/idl2srv.py)
  message("[rtmbuild_init] - idl2srv_EXECUTABLE     -> ${idl2srv_EXECUTABLE}")

  execute_process(COMMAND pkg-config openrtm-aist --variable=prefix      OUTPUT_VARIABLE rtm_prefix    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(EXISTS ${rtm_prefix}/bin/rtm-skelwrapper)
    set(_rtm_exe_path ${rtm_prefix}/bin)
  else()
    set(_rtm_exe_path ${rtm_prefix}/lib/openrtm_aist/bin)
  endif()
  set(rtmskel_EXECUTABLE PATH=${_rtm_exe_path}:$ENV{PATH} PYTHONPATH=${openrtm_aist_PREFIX}/lib/openrtm-1.1/py_helper:$ENV{PYTHONPATH} ${_rtm_exe_path}/rtm-skelwrapper)
  message("[rtmbuild_init] - rtmskel_EXECUTABLE     -> ${rtmskel_EXECUTABLE}")

  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_idlc     OUTPUT_VARIABLE rtm_idlc     OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_idlflags OUTPUT_VARIABLE rtm_idlflags OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(rtm_idlflags "${rtm_idlflags} -Wbuse_quotes") # IDLs in hrpsys-base needs this option because of https://github.com/start-jsk/rtmros_common/issues/861. We can remove this after openrtm-aist.pc is updated.
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_idldir   OUTPUT_VARIABLE rtm_idldir   OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_cxx      OUTPUT_VARIABLE rtm_cxx      OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_cflags   OUTPUT_VARIABLE rtm_cflags   OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_libs     OUTPUT_VARIABLE rtm_libs     OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openhrp3.1   --variable=idl_dir      OUTPUT_VARIABLE hrp_idldir   OUTPUT_STRIP_TRAILING_WHITESPACE)
  separate_arguments(rtm_idlflags)
  separate_arguments(rtm_cflags)
  separate_arguments(rtm_libs)
  set(rtm_cxx "c++") ## openrtm-aist --variable=rtm_cxx sometimes returns /usr/lib/ccache/c++
  message("[rtmbuild_init] - rtm_idlc               -> ${rtm_idlc}")
  message("[rtmbuild_init] - rtm_idlflags           -> ${rtm_idlflags}")
  message("[rtmbuild_init] - rtm_idldir             -> ${rtm_idldir}")
  message("[rtmbuild_init] - rtm_cxx                -> ${rtm_cxx}")
  message("[rtmbuild_init] - rtm_cflags             -> ${rtm_cflags}")
  message("[rtmbuild_init] - rtm_libs               -> ${rtm_libs}")
  message("[rtmbuild_init] - hrp_idldir             -> ${hrp_idldir}")

  ##
  ## get idl files and store to _idl_list
  message("[rtmbuild_init] Generating bridge compornents from ${PROJECT_SOURCE_DIR}/idl")
  set(${PROJECT_NAME}_idl_files "")
  # use _rtmbuild_get_idls_from_dir instead of _rtmbuild_get_idls
  # _rtmbuild_get_idls() ## set ${PROJECT_NAME}_idl_files
  _rtmbuild_get_idls_from_dir(${_idl_dir}) ## set ${PROJECT_NAME}_idl_files
  message("[rtmbuild_init] - ${PROJECT_NAME}_idl_files : ${${PROJECT_NAME}_idl_files}")
  if(NOT ${PROJECT_NAME}_idl_files)
    message(AUTHOR_WARNING "[rtmbuild_init] - no idl file is defined")
  endif()

  ## generate msg/srv/cpp from idl
  set(${PROJECT_NAME}_autogen_msg_files "")
  set(${PROJECT_NAME}_autogen_srv_files "")
  _rtmbuild_genbridge_init()
  message("[rtmbuild_init] - ${PROJECT_NAME}_autogen_msg_files  : ${${PROJECT_NAME}_autogen_msg_files}")
  message("[rtmbuild_init] - ${PROJECT_NAME}_autogen_srv_files  : ${${PROJECT_NAME}_autogen_srv_files}")
  message("[rtmbuild_init] - ${PROJECT_NAME}_autogen_interfaces : ${${PROJECT_NAME}_autogen_interfaces}")
  set(rtmbuild_${PROJECT_NAME}_autogen_msg_files ${${PROJECT_NAME}_autogen_msg_files}) 

  ##
  ## rosbulid_init for rosbuild
  if(NOT use_catkin)
    set(ROSBUILD_DONT_REDEFINE_PROJECT TRUE)
    rosbuild_init()
  endif()

  if(use_catkin)
    add_message_files(DIRECTORY msg FILES "${${PROJECT_NAME}_autogen_msg_files}")
    add_service_files(DIRECTORY srv FILES "${${PROJECT_NAME}_autogen_srv_files}")
    generate_messages(DEPENDENCIES std_msgs ${_extra_message_dependencies})
  else()
    rosbuild_genmsg()
    rosbuild_gensrv()
  endif()

  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${openhrp3_LIBRARY_DIRS})

endmacro(rtmbuild_init_from_dir)

# add_custom_command to compile idl/*.idl file into c++
# Change points from original rtmbuild_genidl:
# - use idl dir name given as argument
# - add suffix given as argument to lib of CORBA skeleton and stub
# - generate idl python in dir given as argument
# use idl dir name given as argument
# add suffix given as argument to lib of CORBA skeleton and stub
# generate idl python in dir given as argument
# macro(rtmbuild_genidl)
macro(rtmbuild_genidl_from_dir _idl_dir _lib_suffix _output_idl_py_dir)
  message("[rtmbuild_genidl_from_dir] add_custom_command for idl files in package ${PROJECT_NAME}")

  set(_autogen "")

  if (use_catkin)
    set(_output_cpp_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set(_output_lib_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    # generate idl python in dir given as argument
    # set(_output_python_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME})
    set(_output_idl_py_dir_abs ${PROJECT_SOURCE_DIR}/${_output_idl_py_dir})
  else()
    # use idl dir name given as argument
    # set(_output_dir ${PROJECT_SOURCE_DIR}/idl_gen)
    # set(_output_cpp_dir ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${PROJECT_NAME})
    # set(_output_lib_dir ${PROJECT_SOURCE_DIR}/idl_gen/lib)
    set(_output_dir ${PROJECT_SOURCE_DIR}/${_idl_dir}_gen)
    set(_output_cpp_dir ${PROJECT_SOURCE_DIR}/${_idl_dir}_gen/cpp/${PROJECT_NAME})
    set(_output_lib_dir ${PROJECT_SOURCE_DIR}/${_idl_dir}_gen/lib)
    # generate idl python in dir given as argument
    # set(_output_python_dir ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME})
    set(_output_idl_py_dir_abs ${PROJECT_SOURCE_DIR}/${_output_idl_py_dir})
    # use idl dir name given as argument
    # include_directories(${PROJECT_SOURCE_DIR}/idl_gen/cpp/)
    include_directories(${PROJECT_SOURCE_DIR}/${_idl_dir}_gen/cpp/)
  endif()

  set(_output_idl_py_files "")
  set(_output_idl_hh_files "")
  # use idl dir name given as argument
  # file(MAKE_DIRECTORY ${_output_cpp_dir}/idl)
  file(MAKE_DIRECTORY ${_output_cpp_dir}/${_idl_dir})
  file(MAKE_DIRECTORY ${_output_lib_dir})
  link_directories(${_output_lib_dir})

  message("[rtmbuild_genidl_from_dir] - _output_cpp_dir : ${_output_cpp_dir}")
  message("[rtmbuild_genidl_from_dir] - _output_lib_dir : ${_output_lib_dir}")
  # generate idl python in dir given as argument
  # message("[rtmbuild_genidl] - _output_python_dir : ${_output_python_dir}")
  message("[rtmbuild_genidl_from_dir] - _output_idl_py_dir_abs: ${_output_idl_py_dir_abs}")

  ## RTMBUILD_${PROJECT_NAME}_genrpc) depends on each RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genrpc)
  if(NOT ${PROJECT_NAME}_idl_files)
    message(AUTHOR_WARNING "[rtmbuild_genidl_from_dir] - no idl file is defined")
  endif()
  foreach(_idl_file ${${PROJECT_NAME}_idl_files})
    get_filename_component(_idl_name ${_idl_file} NAME_WE)
    message("[rtmbuild_genidl_from_dir] - _idl_file : ${_idl_file}")
    message("[rtmbuild_genidl_from_dir] - _idl_name : ${_idl_name}")

    ## use idl dir name given as argument
    ## set(_input_idl ${PROJECT_SOURCE_DIR}/idl/${_idl})
    # set(_input_idl ${PROJECT_SOURCE_DIR}/${_idl_dir}/${_idl})

    # use idl dir name given as argument
    # set(_output_idl_hh ${_output_cpp_dir}/idl/${_idl_name}.hh)
    set(_output_idl_hh ${_output_cpp_dir}/${_idl_dir}/${_idl_name}.hh)
    # generate idl python in dir given as argument
    # set(_output_idl_py ${_output_python_dir}/${_idl_name}_idl.py)
    set(_output_idl_py ${_output_idl_py_dir_abs}/${_idl_name}_idl.py)
    # use idl dir name given as argument
    # set(_output_stub_h ${_output_cpp_dir}/idl/${_idl_name}Stub.h)
    # set(_output_skel_h ${_output_cpp_dir}/idl/${_idl_name}Skel.h)
    # set(_output_stub_cpp ${_output_cpp_dir}/idl/${_idl_name}Stub.cpp)
    # set(_output_skel_cpp ${_output_cpp_dir}/idl/${_idl_name}Skel.cpp)
    set(_output_stub_h ${_output_cpp_dir}/${_idl_dir}/${_idl_name}Stub.h)
    set(_output_skel_h ${_output_cpp_dir}/${_idl_dir}/${_idl_name}Skel.h)
    set(_output_stub_cpp ${_output_cpp_dir}/${_idl_dir}/${_idl_name}Stub.cpp)
    set(_output_skel_cpp ${_output_cpp_dir}/${_idl_dir}/${_idl_name}Skel.cpp)
    # add suffix given as argument to lib of CORBA skeleton and stub
    # set(_output_stub_lib ${_output_lib_dir}/lib${_idl_name}Stub.so)
    # set(_output_skel_lib ${_output_lib_dir}/lib${_idl_name}Skel.so)
    # list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS lib${_idl_name}Stub.so lib${_idl_name}Skel.so)
    set(_output_stub_lib ${_output_lib_dir}/lib${_idl_name}Stub_${_lib_suffix}.so)
    set(_output_skel_lib ${_output_lib_dir}/lib${_idl_name}Skel_${_lib_suffix}.so)
    list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS lib${_idl_name}Stub_${_lib_suffix}.so lib${_idl_name}Skel_${_lib_suffix}.so)
    # call the  rule to compile idl
    if(DEBUG_RTMBUILD_CMAKE)
      message("[rtmbuild_genidl_from_dir] ${_output_idl_hh}\n -> ${_idl_file} ${${_idl}_depends}")
      message("[rtmbuild_genidl_from_dir] ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}\n -> ${_output_idl_hh}")
      message("[rtmbuild_genidl_from_dir] ${_output_stub_lib} ${_output_skel_lib}\n -> ${_output_stub_cpp} ${_output_stub_h} ${_output_skel_cpp} ${_output_skel_h}")
    endif()
    # cpp
    add_custom_command(OUTPUT ${_output_idl_hh}
      # use idl dir name given as argument
      # COMMAND ${rtm_idlc} ${rtm_idlflags} -C${_output_cpp_dir}/idl ${_idl_file}
      COMMAND ${rtm_idlc} ${rtm_idlflags} -C${_output_cpp_dir}/${_idl_dir} ${_idl_file}
      DEPENDS ${_idl_file})
    add_custom_command(OUTPUT ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}
      # use idl dir name given as argument
      # COMMAND cp ${_idl_file} ${_output_cpp_dir}/idl
      COMMAND cp ${_idl_file} ${_output_cpp_dir}/${_idl_dir}
      COMMAND rm -f ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}
      COMMAND ${rtmskel_EXECUTABLE} --include-dir="" --skel-suffix=Skel --stub-suffix=Stub  --idl-file=${_idl_file}
      # use idl dir name given as argument
      # WORKING_DIRECTORY ${_output_cpp_dir}/idl
      WORKING_DIRECTORY ${_output_cpp_dir}/${_idl_dir}
      DEPENDS ${_output_idl_hh})
    add_custom_command(OUTPUT ${_output_stub_lib} ${_output_skel_lib}
      COMMAND ${rtm_cxx} ${rtm_cflags} -I. -shared -o ${_output_stub_lib} ${_output_stub_cpp} ${rtm_libs}
      COMMAND ${rtm_cxx} ${rtm_cflags} -I. -shared -o ${_output_skel_lib} ${_output_skel_cpp} ${rtm_libs}
      DEPENDS ${_output_stub_cpp} ${_output_stub_h} ${_output_skel_cpp} ${_output_skel_h})
    list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS ${_output_stub_lib} ${_output_skel_lib})
    if(use_catkin)
      install(PROGRAMS ${_output_stub_lib} ${_output_skel_lib} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    endif()
    # python
    list(APPEND _output_idl_py_files ${_output_idl_py})
    # cpp
    list(APPEND _output_idl_hh_files ${_output_idl_hh})
    #
    list(APPEND _autogen ${_output_stub_lib} ${_output_skel_lib} ${_output_idl_py})

    # add custom target
    add_custom_target(RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc DEPENDS ${_output_stub_lib} ${_output_skel_lib})
    add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc)
    # genrpc may depends on any idl (generate all .hh filesbefore compiling rpc https://github.com/fkanehiro/hrpsys-base/pull/886)
    add_dependencies(RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc RTMBUILD_${PROJECT_NAME}_genhh)

  endforeach(_idl_file)
  # python
  # generate idl python in dir given as argument
  # add_custom_target(RTMBUILD_${PROJECT_NAME}_genpy DEPENDS ${_output_idl_py_files})
  # add_custom_command(OUTPUT ${_output_idl_py_files}
  #   COMMAND mkdir -p ${_output_python_dir}
  #   COMMAND echo \"${rtm_idlc} -bpython -I${rtm_idldir} -C${_output_python_dir} ${${PROJECT_NAME}_idl_files}\"
  #   COMMAND ${rtm_idlc} -bpython -I${rtm_idldir} -C${_output_python_dir} ${${PROJECT_NAME}_idl_files}
  #   COMMENT "Generating python/idl from ${${PROJECT_NAME}_idl_files}"
  #   DEPENDS ${${PROJECT_NAME}_idl_files})
  # add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_genpy)
  string(RANDOM _rand_str)
  set(_rtm_idlc_py_dir /tmp/rtm_idlc_py_${PROJECT_NAME}_${_rand_str})
  set(_rtm_idlc_idl_py_files "")
  foreach(_output_idl_py ${_output_idl_py_files})
    string(REPLACE ${_output_idl_py_dir_abs} ${_rtm_idlc_py_dir} _rtm_idlc_idl_py ${_output_idl_py})
    list(APPEND _rtm_idlc_idl_py_files ${_rtm_idlc_idl_py})
  endforeach(_output_idl_py)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genpy DEPENDS ${_output_idl_py_files})
  add_custom_command(OUTPUT ${_output_idl_py_files}
    COMMAND mkdir -p ${_rtm_idlc_py_dir}
    COMMAND echo \"${rtm_idlc} -bpython -I${rtm_idldir} -C${_rtm_idlc_py_dir} ${${PROJECT_NAME}_idl_files}\"
    COMMAND ${rtm_idlc} -bpython -I${rtm_idldir} -C${_rtm_idlc_py_dir} ${${PROJECT_NAME}_idl_files}
    COMMAND mkdir -p ${_output_idl_py_dir_abs}
    COMMAND cp ${_rtm_idlc_idl_py_files} ${_output_idl_py_dir_abs}
    COMMAND rm -rf ${_rtm_idlc_py_dir}
    COMMENT "Generating python/idl from ${${PROJECT_NAME}_idl_files}"
    DEPENDS ${${PROJECT_NAME}_idl_files})
  add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_genpy)
  # cpp (generate all .hh filesbefore compiling rpc https://github.com/fkanehiro/hrpsys-base/pull/886)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genhh DEPENDS ${_output_idl_hh_files})
  add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_genhh)
  ##

  if(_autogen)
    if(DEBUG_RTMBUILD_CMAKE)
      message("[rtmbuild_genidl_from_dir] ADDITIONAL_MAKE_CLEAN_FILES : ${_autogen}")
    endif()
    # Also set up to clean the srv_gen directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${_autogen})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(rtmbuild_genidl_from_dir)
