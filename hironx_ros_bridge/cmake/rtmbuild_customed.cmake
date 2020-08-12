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
## See rtmbuild/cmake/rtmbuild.cmake for GLOBAL VARIABLES
##

# add_custom_command to compile idl/*.idl file into c++
# Change points from original rtmbuild_genidl:
# - don't generate python
# - don't overwrite lib of CORBA skeleton and stub from other pkgs to avoid compile error in those pkgs
macro(rtmbuild_genidl_customed)
  message("[rtmbuild_genidl_customed] add_custom_command for idl files in package ${PROJECT_NAME}")

  set(_autogen "")

  if (use_catkin)
    set(_output_cpp_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    # don't overwrite lib of CORBA skeleton and stub from other pkgs to avoid compile error in those pkgs
    # set(_output_lib_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    set(_output_lib_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})
    # don't generate python
    # set(_output_python_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME})
    unset(_output_python_dir)
  else()
    set(_output_dir ${PROJECT_SOURCE_DIR}/idl_gen)
    set(_output_cpp_dir ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${PROJECT_NAME})
    set(_output_lib_dir ${PROJECT_SOURCE_DIR}/idl_gen/lib)
    # don't generate python
    # set(_output_python_dir ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME})
    unset(_output_python_dir)
    include_directories(${PROJECT_SOURCE_DIR}/idl_gen/cpp/)
  endif()

  # don't generate python
  # set(_output_idl_py_files "")
  unset(_output_idl_py_files)
  set(_output_idl_hh_files "")
  file(MAKE_DIRECTORY ${_output_cpp_dir}/idl)
  file(MAKE_DIRECTORY ${_output_lib_dir})
  link_directories(${_output_lib_dir})

  message("[rtmbuild_genidl_customed] - _output_cpp_dir : ${_output_cpp_dir}")
  message("[rtmbuild_genidl_customed] - _output_lib_dir : ${_output_lib_dir}")
  # don't generate python
  # message("[rtmbuild_genidl] - _output_python_dir : ${_output_python_dir}")

  ## RTMBUILD_${PROJECT_NAME}_genrpc) depends on each RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genrpc)
  if(NOT ${PROJECT_NAME}_idl_files)
    message(AUTHOR_WARNING "[rtmbuild_genidl_customed] - no idl file is defined")
  endif()
  foreach(_idl_file ${${PROJECT_NAME}_idl_files})
    get_filename_component(_idl_name ${_idl_file} NAME_WE)
    message("[rtmbuild_genidl_customed] - _idl_file : ${_idl_file}")
    message("[rtmbuild_genidl_customed] - _idl_name : ${_idl_name}")

    # set(_input_idl ${PROJECT_SOURCE_DIR}/idl/${_idl})

    set(_output_idl_hh ${_output_cpp_dir}/idl/${_idl_name}.hh)
    # don't generate python
    # set(_output_idl_py ${_output_python_dir}/${_idl_name}_idl.py)
    unset(_output_idl_py)
    set(_output_stub_h ${_output_cpp_dir}/idl/${_idl_name}Stub.h)
    set(_output_skel_h ${_output_cpp_dir}/idl/${_idl_name}Skel.h)
    set(_output_stub_cpp ${_output_cpp_dir}/idl/${_idl_name}Stub.cpp)
    set(_output_skel_cpp ${_output_cpp_dir}/idl/${_idl_name}Skel.cpp)
    set(_output_stub_lib ${_output_lib_dir}/lib${_idl_name}Stub.so)
    set(_output_skel_lib ${_output_lib_dir}/lib${_idl_name}Skel.so)
    list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS lib${_idl_name}Stub.so lib${_idl_name}Skel.so)
    # call the  rule to compile idl
    if(DEBUG_RTMBUILD_CMAKE)
      message("[rtmbuild_genidl_customed] ${_output_idl_hh}\n -> ${_idl_file} ${${_idl}_depends}")
      message("[rtmbuild_genidl_customed] ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}\n -> ${_output_idl_hh}")
      message("[rtmbuild_genidl_customed] ${_output_stub_lib} ${_output_skel_lib}\n -> ${_output_stub_cpp} ${_output_stub_h} ${_output_skel_cpp} ${_output_skel_h}")
    endif()
    # cpp
    add_custom_command(OUTPUT ${_output_idl_hh}
      COMMAND ${rtm_idlc} ${rtm_idlflags} -C${_output_cpp_dir}/idl ${_idl_file}
      DEPENDS ${_idl_file})
    add_custom_command(OUTPUT ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}
      COMMAND cp ${_idl_file} ${_output_cpp_dir}/idl
      COMMAND rm -f ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}
      COMMAND ${rtmskel_EXECUTABLE} --include-dir="" --skel-suffix=Skel --stub-suffix=Stub  --idl-file=${_idl_file}
      WORKING_DIRECTORY ${_output_cpp_dir}/idl
      DEPENDS ${_output_idl_hh})
    add_custom_command(OUTPUT ${_output_stub_lib} ${_output_skel_lib}
      COMMAND ${rtm_cxx} ${rtm_cflags} -I. -shared -o ${_output_stub_lib} ${_output_stub_cpp} ${rtm_libs}
      COMMAND ${rtm_cxx} ${rtm_cflags} -I. -shared -o ${_output_skel_lib} ${_output_skel_cpp} ${rtm_libs}
      DEPENDS ${_output_stub_cpp} ${_output_stub_h} ${_output_skel_cpp} ${_output_skel_h})
    list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS ${_output_stub_lib} ${_output_skel_lib})
    if(use_catkin)
      # don't overwrite lib of CORBA skeleton and stub from other pkgs to avoid compile error in those pkgs
      # install(PROGRAMS ${_output_stub_lib} ${_output_skel_lib} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
      install(PROGRAMS ${_output_stub_lib} ${_output_skel_lib} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})
    endif()
    # python
    # don't generate python
    # list(APPEND _output_idl_py_files ${_output_idl_py})
    # cpp
    list(APPEND _output_idl_hh_files ${_output_idl_hh})
    #
    # don't generate python
    # list(APPEND _autogen ${_output_stub_lib} ${_output_skel_lib} ${_output_idl_py})
    list(APPEND _autogen ${_output_stub_lib} ${_output_skel_lib})

    # add custom target
    add_custom_target(RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc DEPENDS ${_output_stub_lib} ${_output_skel_lib})
    add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc)
    # genrpc may depends on any idl (generate all .hh filesbefore compiling rpc https://github.com/fkanehiro/hrpsys-base/pull/886)
    add_dependencies(RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc RTMBUILD_${PROJECT_NAME}_genhh)

  endforeach(_idl_file)
  # python
  # don't generate python
  # add_custom_target(RTMBUILD_${PROJECT_NAME}_genpy DEPENDS ${_output_idl_py_files})
  # add_custom_command(OUTPUT ${_output_idl_py_files}
  #   COMMAND mkdir -p ${_output_python_dir}
  #   COMMAND echo \"${rtm_idlc} -bpython -I${rtm_idldir} -C${_output_python_dir} ${${PROJECT_NAME}_idl_files}\"
  #   COMMAND ${rtm_idlc} -bpython -I${rtm_idldir} -C${_output_python_dir} ${${PROJECT_NAME}_idl_files}
  #   COMMENT "Generating python/idl from ${${PROJECT_NAME}_idl_files}"
  #   DEPENDS ${${PROJECT_NAME}_idl_files})
  # add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_genpy)
  # cpp (generate all .hh filesbefore compiling rpc https://github.com/fkanehiro/hrpsys-base/pull/886)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genhh DEPENDS ${_output_idl_hh_files})
  add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_genhh)
  ##

  if(_autogen)
    if(DEBUG_RTMBUILD_CMAKE)
      message("[rtmbuild_genidl_customed] ADDITIONAL_MAKE_CLEAN_FILES : ${_autogen}")
    endif()
    # Also set up to clean the srv_gen directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${_autogen})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(rtmbuild_genidl_customed)
